#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoEigenDense.h>
class IMU {
public:
  bool begin(int sda=21,int scl=22);
  bool calibrate(int samples=300);
  bool update();
  Eigen::Vector3d getAccel() const {return accel;}
  Eigen::Vector3d getGyro() const {return gyro;}
  Eigen::Vector3d getStationaryAccel() const {return initialAccel;}
private:
  Eigen::Vector3d accel=Eigen::Vector3d::Zero(),gyro=Eigen::Vector3d::Zero();
  Eigen::Vector3d gyroOffset=Eigen::Vector3d::Zero(),initialAccel=Eigen::Vector3d::Zero();
  bool readRaw(Eigen::Vector3d&,Eigen::Vector3d&);
};
