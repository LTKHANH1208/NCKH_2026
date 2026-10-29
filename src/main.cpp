#include <Arduino.h>
#include "esp_camera.h"
#include <Wire.h>
#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>

/*
 * ESP32-CAM AI Thinker (OV2640) - Arduino IDE
 * Tim cac vung bien anh, chon vat the lon nhat, chon huong di
 * va tra 5 byte cho bo dieu khien dong co qua I2C slave 0x42.
 *
 * I2C: GPIO13=SDA, GPIO14=SCL. KHONG su dung the microSD.
 * Dien ap I2C 3.3V; SDA/SCL can pull-up len 3.3V.
 *
 * Packet: [X_H, X_L, Y_H, Y_L, MOVE]
 * X/Y: int16_t big-endian; -1 = khong thay doi tuong.
 * MOVE: 0=STOP, 1=LEFT, 2=FORWARD, 3=RIGHT.
 */

static constexpr int FRAME_W = 320;
static constexpr int FRAME_H = 240;
static constexpr size_t IMAGE_SIZE = FRAME_W * FRAME_H;
static constexpr int MAX_OBJECTS = 5;
static constexpr int MIN_AREA = 40;
static constexpr int MAX_AREA = 30000;
static constexpr int EDGE_THRESHOLD = 35;
static constexpr int MOTION_THRESHOLD = 15;
static constexpr uint32_t DEBUG_EVERY_N_FRAMES = 10;

// Toi da 160 doan 1-pixel tren mot dong 320-pixel.
// Chi giu nhan cua dong hien tai va dong truoc: toi da 320 nhan tam.
static constexpr int MAX_RUNS_PER_ROW = (FRAME_W + 1) / 2;
static constexpr int MAX_LIVE_LABELS = MAX_RUNS_PER_ROW * 2;
static constexpr int RUN_JOIN_DISTANCE = 5;  // Giu dung nguong code cu.

static constexpr uint8_t I2C_ADDRESS = 0x42;
static constexpr int I2C_SDA_PIN = 13;
static constexpr int I2C_SCL_PIN = 14;

enum MoveCommand : uint8_t {
  CMD_STOP = 0,
  CMD_LEFT = 1,
  CMD_FORWARD = 2,
  CMD_RIGHT = 3
};

struct ObjectInfo {
  int16_t centerX;
  int16_t centerY;
  int area;
  int minX;
  int maxX;
  int minY;
  int maxY;
};

struct RegionInfo {
  float edgeDensity;
  float motionDensity;
  float score;
};

struct Segment {
  int xStart;
  int xEnd;
  int label;
};

struct Component {
  int area;
  uint32_t sumX;
  uint32_t sumY;
  int minX;
  int maxX;
  int minY;
  int maxY;
};

static ObjectInfo objects[MAX_OBJECTS];
static RegionInfo regions[3];
static int objectCount = 0;
static int lastRegion = 1;
static int bestRegion = -1;
static int16_t targetX = -1;
static int16_t targetY = -1;
static MoveCommand moveCommand = CMD_STOP;

static uint8_t *gray = nullptr;
static uint8_t *prevGray = nullptr;
static uint8_t *mask = nullptr;
static uint8_t *morphBuffer = nullptr;
static bool hasPreviousFrame = false;
static bool cameraReady = false;

// Mutex giu goi 5 byte nhat quan khi I2C callback va loop chay khac task/core.
static portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t i2cPacket[5] = {0xFF, 0xFF, 0xFF, 0xFF, CMD_STOP};

static void onI2CRequest() {
  uint8_t copy[5];
  portENTER_CRITICAL(&packetMux);
  memcpy(copy, i2cPacket, sizeof(copy));
  portEXIT_CRITICAL(&packetMux);
  Wire.write(copy, sizeof(copy));
}

static void publishPacket() {
  uint16_t x = static_cast<uint16_t>(targetX);
  uint16_t y = static_cast<uint16_t>(targetY);
  uint8_t next[5] = {
    static_cast<uint8_t>(x >> 8), static_cast<uint8_t>(x & 0xFF),
    static_cast<uint8_t>(y >> 8), static_cast<uint8_t>(y & 0xFF),
    static_cast<uint8_t>(moveCommand)
  };

  portENTER_CRITICAL(&packetMux);
  memcpy(i2cPacket, next, sizeof(next));
  portEXIT_CRITICAL(&packetMux);
}

static void failSafeStop() {
  targetX = -1;
  targetY = -1;
  bestRegion = -1;
  moveCommand = CMD_STOP;
  hasPreviousFrame = false;
  publishPacket();
}

static bool allocateBuffers() {
  gray = static_cast<uint8_t *>(heap_caps_malloc(IMAGE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  prevGray = static_cast<uint8_t *>(heap_caps_malloc(IMAGE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  mask = static_cast<uint8_t *>(heap_caps_malloc(IMAGE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  morphBuffer = static_cast<uint8_t *>(heap_caps_malloc(IMAGE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

  if (!gray || !prevGray || !mask || !morphBuffer) {
    Serial.println("ERROR: khong du PSRAM de cap phat bo dem.");
    return false;
  }
  memset(prevGray, 0, IMAGE_SIZE);
  return true;
}

static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  // So do chan chinh thuc cua AI Thinker ESP32-CAM.
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sccb_sda = 26;
  config.pin_sccb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;

  config.xclk_freq_hz = 20000000;
  // Xu ly anh truc tiep dang 8-bit grayscale: nhanh va it ton bo nho hon RGB565.
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA; // 320 x 240
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t result = esp_camera_init(&config);
  if (result != ESP_OK) {
    Serial.printf("ERROR: esp_camera_init = 0x%X\n", result);
    return false;
  }
  return true;
}

static void detectEdges() {
  memset(mask, 0, IMAGE_SIZE);
  for (int y = 1; y < FRAME_H - 1; ++y) {
    int base = y * FRAME_W;
    for (int x = 1; x < FRAME_W - 1; ++x) {
      int pos = base + x;
      int gx = abs(static_cast<int>(gray[pos - 1]) - gray[pos + 1]);
      int gy = abs(static_cast<int>(gray[pos - FRAME_W]) - gray[pos + FRAME_W]);
      mask[pos] = ((gx + gy) >= EDGE_THRESHOLD) ? 1 : 0;
    }
  }
}

static void dilate3x3() {
  memset(morphBuffer, 0, IMAGE_SIZE);
  for (int y = 1; y < FRAME_H - 1; ++y) {
    for (int x = 1; x < FRAME_W - 1; ++x) {
      bool found = false;
      for (int dy = -1; dy <= 1 && !found; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (mask[(y + dy) * FRAME_W + (x + dx)]) {
            found = true;
            break;
          }
        }
      }
      morphBuffer[y * FRAME_W + x] = found ? 1 : 0;
    }
  }
  memcpy(mask, morphBuffer, IMAGE_SIZE);
}

static void erode3x3() {
  memset(morphBuffer, 0, IMAGE_SIZE);
  for (int y = 1; y < FRAME_H - 1; ++y) {
    for (int x = 1; x < FRAME_W - 1; ++x) {
      bool all = true;
      for (int dy = -1; dy <= 1 && all; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (!mask[(y + dy) * FRAME_W + (x + dx)]) {
            all = false;
            break;
          }
        }
      }
      morphBuffer[y * FRAME_W + x] = all ? 1 : 0;
    }
  }
  memcpy(mask, morphBuffer, IMAGE_SIZE);
}

static void cleanNoise() {
  // Giu NGUYEN chuoi xu ly cua code goc: Opening -> Closing.
  // Opening = erosion -> dilation.
  erode3x3();
  dilate3x3();
  // Closing = dilation -> erosion.
  dilate3x3();
  erode3x3();
}

static int regionStart(int r) { return r * FRAME_W / 3; }
static int regionEnd(int r) { return (r + 1) * FRAME_W / 3; }

static void calculateEdgeDensities() {
  for (int r = 0; r < 3; ++r) {
    int count = 0;
    const int x0 = regionStart(r);
    const int x1 = regionEnd(r);
    for (int y = 0; y < FRAME_H; ++y) {
      for (int x = x0; x < x1; ++x) {
        count += (mask[y * FRAME_W + x] != 0);
      }
    }
    regions[r].edgeDensity = static_cast<float>(count) / ((x1 - x0) * FRAME_H);
  }
}

// Luu 5 thanh phan lon nhat, khong phu thuoc thu tu quet anh.
static void keepCandidate(const Component &c) {
  if (c.area < MIN_AREA || c.area > MAX_AREA) return;
  ObjectInfo found = {
    static_cast<int16_t>(c.sumX / c.area),
    static_cast<int16_t>(c.sumY / c.area),
    c.area, c.minX, c.maxX, c.minY, c.maxY
  };

  if (objectCount < MAX_OBJECTS) {
    objects[objectCount++] = found;
    return;
  }
  int smallest = 0;
  for (int i = 1; i < objectCount; ++i) {
    if (objects[i].area < objects[smallest].area) smallest = i;
  }
  if (found.area > objects[smallest].area) objects[smallest] = found;
}

static int findRoot(int label, int *parent) {
  while (parent[label] != label) {
    parent[label] = parent[parent[label]];
    label = parent[label];
  }
  return label;
}

static int uniteRoots(int a, int b, int *parent, Component *components) {
  a = findRoot(a, parent);
  b = findRoot(b, parent);
  if (a == b) return a;

  if (components[a].area < components[b].area) {
    int tmp = a;
    a = b;
    b = tmp;
  }
  parent[b] = a;
  components[a].area += components[b].area;
  components[a].sumX += components[b].sumX;
  components[a].sumY += components[b].sumY;
  components[a].minX = min(components[a].minX, components[b].minX);
  components[a].maxX = max(components[a].maxX, components[b].maxX);
  components[a].minY = min(components[a].minY, components[b].minY);
  components[a].maxY = max(components[a].maxY, components[b].maxY);
  return a;
}

static void addSegment(Component &c, int x0, int x1, int y) {
  uint32_t length = x1 - x0 + 1;
  c.area += length;
  c.sumX += static_cast<uint32_t>(x0 + x1) * length / 2;
  c.sumY += static_cast<uint32_t>(y) * length;
  c.minX = min(c.minX, x0);
  c.maxX = max(c.maxX, x1);
  c.minY = min(c.minY, y);
  c.maxY = max(c.maxY, y);
}

// Run-length connected components, giong huong code cu:
// - chi duyet doan pixel lien tiep trong tung dong, khong BFS tung pixel;
// - noi doan cua hai dong ke nhau neu cach nhau toi da 5 pixel;
// - ket thuc va luu vung khong con xuat hien o dong ke tiep;
// - nen nhan moi dong, nen khong bao gio bi tran gioi han 32 nhan.
static void findObjects() {
  static Segment previous[MAX_RUNS_PER_ROW];
  static Segment current[MAX_RUNS_PER_ROW];
  static Component components[MAX_LIVE_LABELS];
  static Component compacted[MAX_RUNS_PER_ROW];
  static int parent[MAX_LIVE_LABELS];
  static int newLabel[MAX_LIVE_LABELS];
  static bool active[MAX_LIVE_LABELS];

  objectCount = 0;
  int previousCount = 0;
  int nextLabel = 0;

  for (int y = 0; y < FRAME_H; ++y) {
    int currentCount = 0;
    int x = 0;

    // Tach tung run [xStart, xEnd] (cac run da sap xep theo X).
    while (x < FRAME_W) {
      while (x < FRAME_W && mask[y * FRAME_W + x] == 0) ++x;
      if (x >= FRAME_W) break;
      int xStart = x;
      while (x < FRAME_W && mask[y * FRAME_W + x] != 0) ++x;
      // Khong the qua 160 run tren 320 pixel nhi phan.
      current[currentCount++] = {xStart, x - 1, -1};
    }

    int previousStart = 0;
    for (int i = 0; i < currentCount; ++i) {
      Segment &run = current[i];
      while (previousStart < previousCount &&
             previous[previousStart].xEnd + RUN_JOIN_DISTANCE < run.xStart) {
        ++previousStart;
      }

      int label = -1;
      for (int p = previousStart; p < previousCount; ++p) {
        const Segment &old = previous[p];
        if (old.xStart > run.xEnd + RUN_JOIN_DISTANCE) break;
        int oldRoot = findRoot(old.label, parent);
        label = (label < 0) ? oldRoot : uniteRoots(label, oldRoot, parent, components);
      }

      if (label < 0) {
        // Toi da 160 nhan dang hoat dong + 160 run moi.
        if (nextLabel >= MAX_LIVE_LABELS) {
          Serial.println("ERROR: CCL internal label limit");
          objectCount = 0;
          return;
        }
        label = nextLabel++;
        parent[label] = label;
        components[label] = {0, 0, 0, run.xStart, run.xEnd, y, y};
      } else {
        label = findRoot(label, parent);
      }
      addSegment(components[label], run.xStart, run.xEnd, y);
      run.label = label;
    }

    memset(active, 0, sizeof(active));
    for (int i = 0; i < currentCount; ++i) {
      int root = findRoot(current[i].label, parent);
      current[i].label = root;
      active[root] = true;
    }

    // Moi root da bien mat khoi dong hien tai se khong the ket noi nua.
    for (int i = 0; i < nextLabel; ++i) {
      if (parent[i] == i && !active[i]) keepCandidate(components[i]);
    }

    // Nen ve <= 160 root de tai su dung nhan trong dong tiep theo.
    for (int i = 0; i < nextLabel; ++i) newLabel[i] = -1;
    int activeCount = 0;
    for (int i = 0; i < currentCount; ++i) {
      int root = current[i].label;
      if (newLabel[root] == -1) {
        newLabel[root] = activeCount;
        compacted[activeCount++] = components[root];
      }
      current[i].label = newLabel[root];
    }
    memcpy(components, compacted, activeCount * sizeof(Component));
    for (int i = 0; i < activeCount; ++i) parent[i] = i;
    nextLabel = activeCount;
    previousCount = currentCount;
    memcpy(previous, current, currentCount * sizeof(Segment));
  }

  // Ket thuc anh: chot cac vung van dang hoat dong o dong cuoi.
  for (int i = 0; i < nextLabel; ++i) keepCandidate(components[i]);
}

static void selectTarget() {
  targetX = -1;
  targetY = -1;
  if (objectCount == 0) return;

  int largest = 0;
  for (int i = 1; i < objectCount; ++i) {
    if (objects[i].area > objects[largest].area) largest = i;
  }
  targetX = objects[largest].centerX;
  targetY = objects[largest].centerY;
}

static float calculateMotion(int beginX, int endX) {
  uint32_t moving = 0;
  for (int y = 0; y < FRAME_H; ++y) {
    int base = y * FRAME_W;
    for (int x = beginX; x < endX; ++x) {
      int pos = base + x;
      if (abs(static_cast<int>(gray[pos]) - prevGray[pos]) > MOTION_THRESHOLD) {
        ++moving;
      }
    }
  }
  return static_cast<float>(moving) / ((endX - beginX) * FRAME_H);
}

static void chooseMovement() {
  moveCommand = CMD_STOP;
  bestRegion = -1;
  if (targetX < 0 || targetX >= FRAME_W) return;

  float bestScore = -1000.0f;
  for (int r = 0; r < 3; ++r) {
    int x0 = regionStart(r);
    int x1 = regionEnd(r);
    float center = (x0 + x1 - 1) * 0.5f;
    regions[r].motionDensity = calculateMotion(x0, x1);
    regions[r].score =
        0.50f * (1.0f - fabsf(targetX - center) / FRAME_W) +
        0.25f * regions[r].edgeDensity +
        0.25f * regions[r].motionDensity;
    float adjusted = regions[r].score - ((r == lastRegion) ? 0.0f : 0.05f);
    if (adjusted > bestScore) {
      bestScore = adjusted;
      bestRegion = r;
    }
  }

  lastRegion = bestRegion;
  if (bestRegion == 0) moveCommand = CMD_LEFT;
  else if (bestRegion == 1) moveCommand = CMD_FORWARD;
  else if (bestRegion == 2) moveCommand = CMD_RIGHT;
}

static const char *commandName(MoveCommand command) {
  switch (command) {
    case CMD_LEFT: return "LEFT";
    case CMD_FORWARD: return "FORWARD";
    case CMD_RIGHT: return "RIGHT";
    default: return "STOP";
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32-CAM AI Thinker tracking + I2C slave");

  if (!psramFound()) {
    Serial.println("ERROR: khong phat hien PSRAM. Chon AI Thinker ESP32-CAM va bat PSRAM.");
    while (true) delay(1000);
  }
  Serial.printf("PSRAM: %u bytes\n", static_cast<unsigned>(ESP.getPsramSize()));

  if (!allocateBuffers()) {
    while (true) delay(1000);
  }
  if (!initCamera()) {
    while (true) delay(1000);
  }
  cameraReady = true;

  // Dang ky callback truoc Wire.begin theo cach dung I2C slave cua Arduino-ESP32.
  Wire.onRequest(onI2CRequest);
  if (!Wire.begin(I2C_ADDRESS, I2C_SDA_PIN, I2C_SCL_PIN, 100000)) {
    Serial.println("ERROR: khong khoi tao duoc I2C slave.");
    while (true) delay(1000);
  }
  failSafeStop();
  Serial.printf("Ready: QVGA grayscale; I2C slave 0x%02X; SDA=%d SCL=%d\n",
                I2C_ADDRESS, I2C_SDA_PIN, I2C_SCL_PIN);
}

void loop() {
  if (!cameraReady) {
    failSafeStop();
    delay(1000);
    return;
  }

  uint32_t tStart = micros();
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERROR: camera frame = nullptr");
    failSafeStop();
    delay(100);
    return;
  }

  if (fb->width != FRAME_W || fb->height != FRAME_H ||
      fb->format != PIXFORMAT_GRAYSCALE || fb->len < IMAGE_SIZE) {
    Serial.printf("ERROR: frame invalid (%ux%u format=%u len=%u)\n",
                  static_cast<unsigned>(fb->width), static_cast<unsigned>(fb->height),
                  static_cast<unsigned>(fb->format), static_cast<unsigned>(fb->len));
    esp_camera_fb_return(fb);
    failSafeStop();
    delay(100);
    return;
  }

  memcpy(gray, fb->buf, IMAGE_SIZE);
  esp_camera_fb_return(fb);
  uint32_t tCapture = micros();

  if (!hasPreviousFrame) {
    memcpy(prevGray, gray, IMAGE_SIZE);
    hasPreviousFrame = true;
  }

  detectEdges();
  uint32_t tEdges = micros();
  cleanNoise();
  uint32_t tMorph = micros();
  calculateEdgeDensities();
  findObjects();
  uint32_t tObjects = micros();
  selectTarget();
  chooseMovement();
  publishPacket();
  memcpy(prevGray, gray, IMAGE_SIZE);
  uint32_t tEnd = micros();

  // Giam so lan in Serial de khong lam cham moi frame.
  static uint32_t frameNumber = 0;
  ++frameNumber;
  if (DEBUG_EVERY_N_FRAMES > 0 && frameNumber % DEBUG_EVERY_N_FRAMES == 0) {
    Serial.printf("Frame=%lu Objects=%d Target=(%d,%d) Region=%d Command=%s\n",
                  static_cast<unsigned long>(frameNumber), objectCount,
                  targetX, targetY, bestRegion, commandName(moveCommand));
    Serial.printf("Time us: capture=%lu edge=%lu open+close=%lu components=%lu "
                  "motion+I2C=%lu total=%lu\n",
                  static_cast<unsigned long>(tCapture - tStart),
                  static_cast<unsigned long>(tEdges - tCapture),
                  static_cast<unsigned long>(tMorph - tEdges),
                  static_cast<unsigned long>(tObjects - tMorph),
                  static_cast<unsigned long>(tEnd - tObjects),
                  static_cast<unsigned long>(tEnd - tStart));
  }
  delay(1);
}
