#include<Arduino.h>
#include"esp_camera.h"
#include<Wire.h>
#include<math.h>

#define CAMERA_MODEL_AI_THINKER
#define FRAME_W 320
#define FRAME_H 240
#define maxObject 5
#define myAdress 0x42
#define ImageSize (FRAME_H*FRAME_W)
#define minArea 40
#define maxArea 30000
#define Edge_detected 35
#define MotionThredHold 15
#define Maxcomponent 32

HardwareSerial MotorSerial(2);

enum MoveCommand : uint8_t {
    CMD_STOP = 0,
    CMD_LEFT = 1,
    CMD_FORWARD = 2,
    CMD_RIGHT = 3
};

volatile int16_t targetX = -1;
volatile int16_t targetY = -1;
volatile uint8_t data[5] = {0xFF, 0xFF, 0xFF, 0xFF, CMD_STOP};
uint8_t moveCommand = CMD_STOP;

int lastRegion = 1;

float bestScore = -1.0f;
int bestRegion = 1;

struct Segment{
    int y;

    int x_start;
    int x_end;

    int label;
};

struct RegionInfo
{
    float edgeDensity;
    float motionDensity;
    float probability;
};


struct Component{
    bool used;
    int area;
    long sumX;
    long sumY;
    int minX;
    int maxX;
    int minY;
    int maxY;
};

struct Object{
  int center_x;
  int center_y;
  int area;
  int min_x;
  int max_x;
  int min_y;
  int max_y;
  int width;
  int height;
};

Component component[Maxcomponent];
int parent[Maxcomponent];
int nextLabel = 0;

Object object[maxObject];
RegionInfo region[3];
int trackID = 1;
int objectCount = 0;
uint8_t *gray = nullptr;
uint8_t *mask = nullptr;
uint8_t *morphBuffer = nullptr;
uint8_t *prev_gray = nullptr;

void setup() {
  Serial.begin(115200);

  Wire.begin((uint8_t)0x42, 13, 14, 100000);

  Wire.onRequest(requestEvent);

  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

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
  config.pixel_format = PIXFORMAT_RGB565;

  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  mask = (uint8_t *)malloc(ImageSize);
  gray = (uint8_t *)malloc(ImageSize);
  morphBuffer = (uint8_t *) malloc(ImageSize);
  prev_gray = (uint8_t *) malloc(ImageSize);
  
  esp_camera_init(&config);
}

inline uint8_t rgb565ToGray(uint16_t pixel){
  int red = ((pixel >> 11) & 0x1F) << 3;
  int green = ((pixel >> 5) & 0x3F) << 2;
  int blue = (pixel & 0x1F) << 3;

  return (red * 30 + green * 59 + blue * 11)/100;
}

void grayFilterd(camera_fb_t *fb){
  uint8_t *img = fb->buf;
  for (int y = 0; y < FRAME_H; y++)
  {
    for (int x = 0; x < FRAME_W; x++)
    {
      int index = (y * FRAME_W + x) * 2;
      uint16_t pixel = ((uint16_t)img[index] << 8)|img[index + 1];
      gray[y * FRAME_W + x] = rgb565ToGray(pixel);
    }
  }
}


void resetComponents() {
  nextLabel = 0;

  for (int i = 0; i < Maxcomponent; i++) {
      parent[i] = i;

      component[i].used = false;
      component[i].area = 0;
      component[i].sumX = 0;
      component[i].sumY = 0;

      component[i].minX = FRAME_W;
      component[i].maxX = 0;
      component[i].minY = FRAME_H;
    component[i].maxY = 0;
  }
}

int createComponent(int xStart, int xEnd, int y) {
  if (nextLabel >= Maxcomponent)
    return -1;

  int label = nextLabel++;
  parent[label] = label;

  int length = xEnd - xStart + 1;

  component[label].used = true;
  component[label].area = length;
  component[label].sumX = ((long)xStart + xEnd) * length / 2;
  component[label].sumY = (long)y * length;

  component[label].minX = xStart;
  component[label].maxX = xEnd;
  component[label].minY = y;
  component[label].maxY = y;

  return label;
}


void addSegmentToComponent(int label, int xStart, int xEnd, int y)
{
  if (label < 0 || label >= Maxcomponent) return;

  if (!component[label].used) return;
  int length = xEnd - xStart + 1;
  component[label].area += length;
  component[label].sumX += ((long)xStart + xEnd) * length / 2;
  component[label].sumY += (long)y * length;

  if (xStart < component[label].minX)
  {
    component[label].minX = xStart;
  }

  if (xEnd > component[label].maxX)
  {
    component[label].maxX = xEnd;
  }

  if (y < component[label].minY)
  {
    component[label].minY = y;
  }

  if (y > component[label].maxY)
  {
    component[label].maxY = y;
  }
}


int findRoot(int label) {
  if (label < 0 || label >= nextLabel)
    return -1;

  if (parent[label] != label) {
    parent[label] = findRoot(parent[label]);
  }

  return parent[label];
}

int unionLabels(int a, int b) {
  a = findRoot(a);
  b = findRoot(b);

  if (a < 0) return b;
  if (b < 0) return a;
  if (a == b) return a;
  
  if (component[a].area < component[b].area) {
    int temp = a;
    a = b;
    b = temp;
  }

  parent[b] = a;

  component[a].area += component[b].area;
  component[a].sumX += component[b].sumX;
  component[a].sumY += component[b].sumY;

  component[a].minX = min(component[a].minX, component[b].minX);
  component[a].maxX = max(component[a].maxX, component[b].maxX);
  component[a].minY = min(component[a].minY, component[b].minY);
  component[a].maxY = max(component[a].maxY, component[b].maxY);
  component[b].used = false;

  return a;
}


void edgeMask(){
  memset(mask, 0, ImageSize);
  for (int y = 1; y < FRAME_H - 1; y++)
  {
    for (int x = 1; x < FRAME_W - 1; x++)
    {
      int index = y * FRAME_W + x;
      uint8_t left = gray[index - 1];
      uint8_t right = gray[index + 1];
      uint8_t up = gray[index - FRAME_W];
      uint8_t down = gray[index + FRAME_W];

      int gx = abs(int(left) - int(right));
      int gy = abs(int(up) - int(down));

      int gradient = gx + gy;

      if(gradient >= Edge_detected){
        mask[y*FRAME_W + x] = 1;
      }
    }
  }
}

bool segmentConnected(int x1, int x2, int x3, int x4)
{
  if (x3 > x2 + 5) return false;
  if (x1 > x4 + 5) return false;
  return true;
}

void dilation3x3(){
  memset(morphBuffer, 0, ImageSize);
  for (int y = 1; y < FRAME_H - 1; y++)
  {
    for (int x = 1; x < FRAME_W - 1; x++)
    {
      bool valid = false;
      for (int dy = -1; dy <= 1 && !valid; dy++)
      {
        for (int dx = -1; dx <= 1; dx++)
        {
          int index = (y + dy) * FRAME_W + (dx + x);
          if(mask[index] != 0){
            valid = true;
            break;
          }
        }
      }
      if(valid){
        morphBuffer[y * FRAME_W + x] = 1;
      }
    }
    
  }
  memcpy(mask, morphBuffer, ImageSize);
}

void erosion3x3(){
  memset(morphBuffer, 0, ImageSize);
  for (int y = 1; y < FRAME_H - 1; y++)
  {
    for (int x = 1; x < FRAME_W - 1; x++)
    {
      bool valid = true;
      for (int dy = -1; dy <= 1; dy++)
      {
        for (int dx = -1; dx <= 1; dx++)
        {
          int index = (y + dy) * FRAME_W + (dx + x);
          if(mask[index] == 0){
            valid = false;
            break;
          }
        }
        if(!valid) break;
      }
      if(valid){
        morphBuffer[y * FRAME_W + x] = 1;
      }
    }
    
  }
  memcpy(mask, morphBuffer, ImageSize);
}

void Openning3x3(){
  erosion3x3();
  dilation3x3();
}

void Closing3x3(){
  dilation3x3();
  erosion3x3();
}

void NoiseRemoved(){
  Openning3x3();
  Closing3x3();
}

float CaculateMotion(int beginX, int endX, int maxY, int minY){
  beginX = constrain(beginX, 0, FRAME_W);
  endX = constrain(endX, 0, FRAME_W);

  minY = constrain(minY, 0, FRAME_H - 1);
  maxY = constrain(maxY, 0, FRAME_H - 1);

  if (endX <= beginX || maxY < minY) {
    return 0.0f;
  }
  float motionPixel = 0;
  int width = beginX - beginX + 1;
  int height = maxY - minY + 1;
  long totalDiffer = 0;
  for (int y = minY; y <= maxY; y++)
  {
    for (int x = beginX; x < endX; x++)
    {
      int index = y * FRAME_W + x;
      int differ = abs(gray[index] - prev_gray[index]);
      if(differ > MotionThredHold){
        motionPixel++;
      }
      totalDiffer += differ;
    }
    
  }
  float totalPixel = (maxY - minY + 1) * (endX - beginX);
  float motionScore = motionPixel/ totalPixel;
  return motionScore;
}


void findObject() {
  objectCount = 0;
  resetComponents();

  static Segment previousSegments[100];
  static Segment currentSegments[100];

  int previousCount = 0;

  for (int y = 0; y < FRAME_H; y++) {

    int currentCount = 0;
    int x = 0;

    while (x < FRAME_W) {
      while (x < FRAME_W && mask[y * FRAME_W + x] == 0) {
        x++;
      }

      if (x >= FRAME_W)
        break;

      int startX = x;

      while (x < FRAME_W && mask[y * FRAME_W + x] != 0) {
        x++;
      }

      int endX = x - 1;

      if (currentCount >= 100)
        break;

      Segment &seg = currentSegments[currentCount];

      seg.y = y;
      seg.x_start = startX;
      seg.x_end = endX;
      seg.label = -1;

      currentCount++;
    }

    for (int c = 0; c < currentCount; c++) {
      Segment &seg = currentSegments[c];

      int label = -1;

      for (int p = 0; p < previousCount; p++) {

        Segment &prev = previousSegments[p];

        if (!segmentConnected(prev.x_start, prev.x_end, seg.x_start, seg.x_end)) {
          continue;
        }

        int prevRoot = findRoot(prev.label);

        if (prevRoot < 0) continue;
        if (label == -1) {
          label = prevRoot;
        }
        else {
          label = unionLabels(label, prevRoot);
        }
      }

      if (label == -1) {
        label = createComponent(seg.x_start, seg.x_end, y);
      }
      else {
        label = findRoot(label);

        addSegmentToComponent(label, seg.x_start, seg.x_end, y);
      }

      seg.label = label;
    }
    previousCount = currentCount;

    for (int i = 0; i < currentCount; i++) {
      previousSegments[i] = currentSegments[i];
    }
  }

  for (int i = 0; i < nextLabel; i++) {

    if (findRoot(i) != i)
      continue;

    if (!component[i].used)
      continue;

    int area = component[i].area;

    if (area < minArea || area > maxArea)
      continue;

    if (objectCount >= maxObject)
      break;

    Object &obj = object[objectCount];

    obj.area = area;

    obj.center_x = component[i].sumX / area;
    obj.center_y = component[i].sumY / area;

    obj.min_x = component[i].minX;
    obj.max_x = component[i].maxX;
    obj.min_y = component[i].minY;
    obj.max_y = component[i].maxY;

    obj.width = obj.max_x - obj.min_x + 1;
    obj.height = obj.max_y - obj.min_y + 1;

    objectCount++;
  }
}

void printObjInfo(){
  Serial.println();
  Serial.println("------------------------------");
  Serial.print("Object count: ");
  Serial.println(objectCount);
  for (int i = 0; i < objectCount; i++)
  {
    Serial.print("Object ");
    Serial.println(i + 1);
    Serial.print("  Center: ");
    Serial.print(object[i].center_x);
    Serial.print(", ");
    Serial.println(object[i].center_y);
    Serial.print("  Area: ");
    Serial.println(object[i].area);
    Serial.print("  Width: ");
    Serial.println(object[i].width);
    Serial.print("  Height: ");
    Serial.println(object[i].height);
    Serial.print("  Ratio: ");
    Serial.println((float)object[i].width/object[i].height);
  }
}

void targetFollow(){
  if(objectCount <= 0){
    targetX = -1;
    targetY = -1;
    return;
  }
  int follow = 0;
  for (int i = 0; i < objectCount; i++)
  {
    if(object[i].area > object[follow].area){
      follow = i;
    }
  }
  targetX = object[follow].center_x;
  targetY = object[follow].center_y;
}

void prepareI2CData(){

  noInterrupts();

  data[0] = (uint8_t)(targetX >> 8);
  data[1] = (uint8_t)(targetX & 0xFF);

  data[2] = (uint8_t)(targetY >> 8);
  data[3] = (uint8_t)(targetY & 0xFF);

  data[4] = moveCommand;

  interrupts();
}

void requestEvent(){
  uint8_t packet[5];

  for (int i = 0; i < 5; i++) {
      packet[i] = data[i];
  }

  Wire.write(packet, 5);
}

int regionStartX(int r){
  return r * FRAME_W / 3;
}
int regionEndX(int r){
  return (r + 1) * FRAME_W / 3;
}

void CaculateRegion(){
  if (targetX < 0 || targetX >= FRAME_W){
    moveCommand = CMD_STOP;
    bestRegion = -1;
    return;
  }
  float bestScore = -1000.0f;
  int candidateRegion = lastRegion;
  for (int i = 0; i < 3; i++)
  {
    int x0 = regionStartX(i);
    int x1 = regionEndX(i);

    int edgeCount = 0;
    float total = (x1-x0)*FRAME_H;
    for (int j = 0; j < FRAME_H; j++)
    {
      for (int k = x0; k < x1; k++)
      {
        int idx = j * FRAME_W + k;
        if(mask[idx] == 1){
          edgeCount++;
        }
      }
    }
    region[i].edgeDensity = float(edgeCount)/total;
    region[i].motionDensity = 0;
    region[i].motionDensity = CaculateMotion(x0, x1,FRAME_H - 1, 0);
    if(region[i].motionDensity >= 1.0f) region[i].motionDensity = 1.0f;
    region[i].probability = 0.5 * (1 - fabs(targetX - (x0 + x1 - 1)/2)/FRAME_W) + 0.25 * region[i].edgeDensity + region[i].motionDensity * 0.25;
    float adjustedScore = region[i].probability;
    if(i != lastRegion) adjustedScore -= 0.05f;
    if(adjustedScore > bestScore){
      bestScore = adjustedScore;
      candidateRegion = i;
    }
  }

  bestRegion = candidateRegion;
  lastRegion = bestRegion;

  switch (bestRegion)
  {
  case 0:
    moveCommand = CMD_LEFT;
    break;
  case 1:
    moveCommand = CMD_FORWARD;
    break;
  case 2:
    moveCommand = CMD_RIGHT;
    break;

  default:
    break;
  }
}


void loop() {

  static bool hasPreviousFrame = false;

  camera_fb_t *fb = esp_camera_fb_get();

  if (fb == nullptr) {
      targetX = -1;
      targetY = -1;
      moveCommand = CMD_STOP;
      prepareI2CData();
      hasPreviousFrame = false;
      delay(20);
      return;
  }

  if (fb->width != FRAME_W || fb->height != FRAME_H || fb->format != PIXFORMAT_RGB565 || fb->len < ImageSize * 2) {
    esp_camera_fb_return(fb);

    targetX = -1;
    targetY = -1;
    moveCommand = CMD_STOP;
    prepareI2CData();
    hasPreviousFrame = false;
    delay(20);
    return;
  }

  grayFilterd(fb);
  esp_camera_fb_return(fb);

  if (!hasPreviousFrame) {
    memcpy(prev_gray, gray, ImageSize);
    hasPreviousFrame = true;
  }
  edgeMask();
  NoiseRemoved();
  findObject();
  targetFollow();
  CaculateRegion();
  prepareI2CData();
  memcpy(prev_gray, gray, ImageSize);
  printObjInfo();
  delay(10);
}
