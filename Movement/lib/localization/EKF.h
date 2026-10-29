#pragma once
#include <ArduinoEigenDense.h>

struct State {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_corrected = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_corrected = Eigen::Vector3d::Zero();
};

class EKF {
public:
  explicit EKF(double dt = 0.01);
  void predict(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt);
  void update(double gps_x, double gps_y, double variance = 6.25);
  void initializeAttitude(const Eigen::Vector3d& stationaryAccel);
  State getState() const { return state; }
private:
  using Mat15 = Eigen::Matrix<double,15,15>;
  using Mat15x12 = Eigen::Matrix<double,15,12>;
  State state;
  Mat15 P;
  double defaultDt;
  static Eigen::Matrix3d skew(const Eigen::Vector3d& v);
};
