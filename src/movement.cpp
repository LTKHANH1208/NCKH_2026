#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "imu.h"
#include "ekf.h"

#define GPS_RX 16
#define CAMERA_ADDR 0x42
#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define DEAD_ZONE 15
#define MOVE_THRESHOLD 4

IMU imu;
EKF ekf(0.01);
HardwareSerial gpsPort(2);
char nmea[128]; size_t nmeaLen=0;
bool originSet=false, gpsFix=false;
double lat0=0,lon0=0,gpsX=0,gpsY=0;
uint32_t lastFixMs=0,lastImuUs=0,lastCameraMs=0,lastPrintMs=0;
int16_t prevX=-1,prevY=-1;

enum MoveCommand : uint8_t {
    CMD_STOP = 0,
    CMD_LEFT = 1,
    CMD_FORWARD = 2,
    CMD_RIGHT = 3
};

static double toDegrees(const char* str,char hemi){
  double raw=atof(str);int deg=(int)(raw/100.0);
  double val=deg+(raw-deg*100.0)/60.0;
  return (hemi=='S'||hemi=='W')?-val:val;
}
void parseNMEA(char* s){
  if(strncmp(s,"$GPRMC,",7)!=0 && strncmp(s,"$GNRMC,",7)!=0) return;
  char* star=strchr(s,'*'); if(!star||strlen(star+1)<2)return;
  uint8_t checksum=0;
  for(char* p=s+1;p<star;p++)checksum^=(uint8_t)*p;
  if(checksum!=(uint8_t)strtoul(star+1,nullptr,16))return;
  *star='\0';
  char* fields[13]={};int n=0;fields[n++]=s;
  for(char* p=s;*p && n<13;p++)if(*p==','){*p='\0';fields[n++]=p+1;}
  if(n<9||fields[2][0]!='A'||!fields[3][0]||!fields[5][0]||
     !(fields[4][0]=='N'||fields[4][0]=='S')||
     !(fields[6][0]=='E'||fields[6][0]=='W')){
    gpsFix=false;return;
  }
  const double lat=toDegrees(fields[3],fields[4][0]);
  const double lon=toDegrees(fields[5],fields[6][0]);
  if(fabs(lat)>90||fabs(lon)>180)return;
  gpsFix=true;lastFixMs=millis();
  if(!originSet){lat0=lat;lon0=lon;originSet=true;Serial.println("GPS ORIGIN SET (0,0)");}
  const double k=3.14159265358979323846/180.0;
  gpsX=(lon-lon0)*k*6371000.0*cos(lat0*k);
  gpsY=(lat-lat0)*k*6371000.0;
  ekf.update(gpsX,gpsY,2.5); // one update per valid RMC sentence
}
void pollGPS(){
  while(gpsPort.available()){
    char c=(char)gpsPort.read();
    if(c=='\n'){nmea[nmeaLen]='\0';parseNMEA(nmea);nmeaLen=0;}
    else if(c!='\r'){
      if(nmeaLen<sizeof(nmea)-1)nmea[nmeaLen++]=c;
      else nmeaLen=0;
    }
  }
}
void pollCamera(){
  uint8_t received = Wire.requestFrom((uint8_t)CAMERA_ADDR,(uint8_t)5);

  if (received != 5) {
      while (Wire.available()) Wire.read();
      // stopRobot(); Hàm điều khiển dừng
      delay(50);
      return;
  }
  uint8_t data[5];

  for (int i = 0; i < 5; i++) {
        data[i] = Wire.read();
  }

  int16_t x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  int16_t y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);

  uint8_t command = data[4];

  if (x < 0 || y < 0) {
    // stopRobot();
  }
  else {
    switch (command) {
      case CMD_LEFT:
        // turnLeft();
        break;

      case CMD_FORWARD:
        // moveForward();
        break;

      case CMD_RIGHT:
        // turnRight();
        break;

      default:
        // stopRobot();
        break;
    }
  }
}
void setup(){
  Serial.begin(115200);delay(500);
  if(!imu.begin(21,22)){Serial.println("ERROR MPU6050 not found");while(true)delay(1000);}
  gpsPort.setRxBufferSize(2048);
  gpsPort.begin(9600,SERIAL_8N1,GPS_RX,-1);
  Serial.println("Keep IMU stationary for calibration...");
  if(!imu.calibrate()){Serial.println("ERROR IMU calibration failed");while(true)delay(1000);}
  ekf.initializeAttitude(imu.getStationaryAccel());
  lastImuUs=micros();lastCameraMs=lastPrintMs=millis();
  Serial.println("IMU + GPS + EKF + CAMERA READY");
}
void loop(){
  pollGPS();
  uint32_t nowUs=micros();
  if((uint32_t)(nowUs-lastImuUs)>=10000){
    double dt=(uint32_t)(nowUs-lastImuUs)*1e-6;
    lastImuUs=nowUs;
    if(imu.update())ekf.predict(imu.getAccel(),imu.getGyro(),dt);
  }
  uint32_t ms=millis();
  if((uint32_t)(ms-lastCameraMs)>=100){lastCameraMs=ms;pollCamera();}
  if((uint32_t)(ms-lastPrintMs)>=1000){
    lastPrintMs=ms;
    State s=ekf.getState();
    bool fresh=gpsFix&&((uint32_t)(ms-lastFixMs)<3000);
    Serial.printf("GPS %s | raw local X=%.2f Y=%.2f m\n",fresh?"FIX":"NO FIX",gpsX,gpsY);
    Serial.printf("EKF position X=%.2f Y=%.2f Z=%.2f m\n",s.position.x(),s.position.y(),s.position.z());
    Serial.printf("EKF velocity Vx=%.2f Vy=%.2f Vz=%.2f m/s\n",s.velocity.x(),s.velocity.y(),s.velocity.z());
  }
}
