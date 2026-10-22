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

volatile int16_t targetX = -1;
volatile int16_t targetY = -1;

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

  // MotorSerial.begin(115200, SERIAL_8N1, 14, 15);

  Wire.begin(myAdress);

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

void resetComponents()
{
  for (int i = 0; i < Maxcomponent; i++)
  {
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

int createComponent(int xStart, int xEnd, int y)
{
  for (int i = 0; i < Maxcomponent; i++)
  {
    if (!component[i].used)
    {
      component[i].used = true;
      int length = xEnd - xStart + 1;
      component[i].area = length;
      component[i].sumX = ((long)xStart + xEnd) * length / 2;
      component[i].sumY = (long)y * length;
      component[i].minX = xStart;
      component[i].maxX = xEnd;
      component[i].minY = y;
      component[i].maxY = y;

      return i;
    }
  }
  return -1;
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
  for (int y = 1; y <= FRAME_H - 1; y++)
  {
    for (int x = 1; x <= FRAME_W - 1; x++)
    {
      bool valid = false;
      for (int dy = -1; dy <= 1; dy++)
      {
        for (int dx = -1; dx <= 1; dx++)
        {
          int index = (y + dy) * FRAME_W + (dx + x);
          if(mask[index] != 0){
            valid = true;
            break;
          }
          if(!valid) break;
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

int CaculateMotion(int beginX, int endX, int maxY, int minY){
  int motionPixel = 0;
  int width = beginX - beginX + 1;
  int height = maxY - minY + 1;
  long totalDiffer = 0;
  for (int y = minY; y <= maxY; y++)
  {
    for (int x = beginX; x <= endX; x++)
    {
      int index = y * FRAME_W + x;
      int differ = abs(gray[index] - prev_gray[index]);
      if(differ > MotionThredHold){
        motionPixel++;
      }
      totalDiffer += differ;
    }
    
  }
  int totalPixel = (maxY - minY + 1) * (endX - beginX);
  int motionScore = (motionPixel * 100) / totalPixel;
  return motionScore;
}

void findObject(){

  objectCount = 0;

  resetComponents();
  static Segment previousSegments[100];
  static Segment currentSegments[100];

  int previousCount = 0;

  for (int y = 0; y < FRAME_H - 1; y++)
  {
    int x = 0;
    int currentCount = 0;
       
    while (x < FRAME_W)
    {
      while (x < FRAME_W && mask[y * FRAME_W + x] == 0)
      {
        x++;
      }
      if(x >= FRAME_W) break;
      
      int startX = x;
      while (x < FRAME_W && mask[y * FRAME_W + x] != 0)
      {
        x++;
      }
      int endX = x - 1;
      if(currentCount >= 100) break;
      currentSegments[currentCount].y = y;
      currentSegments[currentCount].x_start = startX;
      currentSegments[currentCount].x_end = endX;
      currentSegments[currentCount].label = -1;
      currentCount++;
    }

    for (int c = 0; c < currentCount; c++)
    {
      int label = -1;
      for (int p = 0; p < previousCount; p++)
      {
        if(segmentConnected(previousSegments[p].x_start, previousSegments[p].x_end, currentSegments[c].x_start, currentSegments[c].x_end)){
          label = previousSegments[p].label;
          break;
        }
      }
      if(label == -1){
        label = createComponent(currentSegments[c].x_start, currentSegments[c].x_end, y);
      }
      else{
        addSegmentToComponent(label, currentSegments[c].x_start, currentSegments[c].x_end, y);
      }

      currentSegments[c].label = label;
    }

    previousCount = currentCount;
    for (int i = 0; i < currentCount; i++){
      previousSegments[i] =
      currentSegments[i];
    }
  }
  for (int i = 0; i < Maxcomponent; i++){
    if(!component[i].used) continue;
    int area = component[i].area;
    if(area < minArea) continue;
    if(area > maxArea) continue;
    if(objectCount >= maxObject) break;

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
  Serial1.println(objectCount);
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

void requestEvent(){
  uint8_t data[4];

  data[0] = targetX >> 8;
  data[1] = targetX & 0xFF;

  data[2] = targetY >> 8;
  data[3] = targetY & 0xFF;

  Wire.write(data, 4);
}

int regionStartX(int r){
  return r * (FRAME_W/3);
}
int regionEndX(int r){
  return (r + 1) * (FRAME_W/3);
}

void CaculateRegion(){
  for (int i = 0; i < 3; i++)
  {
    int x0 = regionStartX(i);
    int x1 = regionEndX(i);

    int edgeCount = 0;
    int total = (x1-x0)*FRAME_H;
    for (int j = 0; j < FRAME_H; j++)
    {
      for (int k = x0; x0 < x1; k++)
      {
        int idx = j * FRAME_W + k;
        if(mask[idx] == 1){
          edgeCount++;
        }
      }
    }
    region[i].edgeDensity = float(edgeCount)/total;
    region[i].motionDensity = 0;
    for (int objectNum = 0; objectNum < objectCount; objectNum++)
    {
      region[i].motionDensity += CaculateMotion(x0, x1, object[objectNum].max_y, object[objectNum].min_y); 
    }
    region[i].probability = 0.5 * (1 - (targetX - (x0 + x1 - 1))/2)/FRAME_W + 0.25 * region[i].edgeDensity + region[i].motionDensity * 0.25;
  }
  memcpy(prev_gray, gray, ImageSize);
}

void loop(){
  unsigned long startTime = millis();
  camera_fb_t *fb = esp_camera_fb_get();
  grayFilterd(fb);
  edgeMask();
  NoiseRemoved();
  findObject();
  printObjInfo();
  targetFollow();
  CaculateRegion();
}