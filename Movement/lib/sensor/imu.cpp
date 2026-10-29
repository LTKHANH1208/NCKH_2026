#include "IMU.h"
static constexpr uint8_t MPU_ADDR=0x68;
static bool writeReg(uint8_t reg,uint8_t value){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission()==0;
}
bool IMU::begin(int sda,int scl){
  Wire.begin(sda,scl); Wire.setClock(100000);
  Wire.beginTransmission(MPU_ADDR);
  if(Wire.endTransmission()!=0) return false;
  return writeReg(0x6B,0x00) && writeReg(0x1B,0x00) && writeReg(0x1C,0x00);
}
bool IMU::readRaw(Eigen::Vector3d& a,Eigen::Vector3d& w){
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if(Wire.endTransmission(false)!=0) return false;
  if(Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)14,(uint8_t)true)!=14) return false;
  int16_t raw[7];
  for(int i=0;i<7;i++) raw[i]=(int16_t)((Wire.read()<<8)|Wire.read());
  a=Eigen::Vector3d(raw[0],raw[1],raw[2])*(9.80665/16384.0);
  w=Eigen::Vector3d(raw[4],raw[5],raw[6])*(3.14159265358979323846/(180.0*131.0));
  return true;
}
bool IMU::calibrate(int samples){
  Eigen::Vector3d sumA=Eigen::Vector3d::Zero(),sumW=Eigen::Vector3d::Zero(),a,w;
  int valid=0;
  for(int i=0;i<samples;i++){
    if(readRaw(a,w)){sumA+=a;sumW+=w;valid++;}
    delay(5);
  }
  if(valid<samples*0.8) return false;
  initialAccel=sumA/valid;gyroOffset=sumW/valid;
  return initialAccel.norm()>7.0 && initialAccel.norm()<12.5;
}
bool IMU::update(){
  Eigen::Vector3d a,w;
  if(!readRaw(a,w))return false;
  accel=a;gyro=w-gyroOffset;
  return true;
}
