#include "EKF.h"
#include <cmath>

Eigen::Matrix3d EKF::skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0,-v.z(),v.y(), v.z(),0,-v.x(), -v.y(),v.x(),0;
  return m;
}

EKF::EKF(double dt): defaultDt(dt) {
  P.setZero();
  P.block<3,3>(0,0).diagonal().setConstant(10.0);
  P.block<3,3>(3,3).diagonal().setConstant(1.0);
  P.block<3,3>(6,6).diagonal().setConstant(0.1);
  P.block<3,3>(9,9).diagonal().setConstant(0.01);
  P.block<3,3>(12,12).diagonal().setConstant(0.01);
}

void EKF::initializeAttitude(const Eigen::Vector3d& a) {
  if (a.norm() < 5.0 || a.norm() > 15.0) return;
  // Body acceleration at rest points up; yaw remains arbitrary (zero).
  const Eigen::Vector3d up = a.normalized();
  state.attitude = Eigen::Quaterniond::FromTwoVectors(up, Eigen::Vector3d::UnitZ());
  state.attitude.normalize();
}

void EKF::predict(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt) {
  if (!(dt > 0.0001 && dt < 0.2) || !std::isfinite(dt)) return;
  state.accel_corrected = acc - state.accel_bias;
  state.gyro_corrected = gyro - state.gyro_bias;
  const Eigen::Matrix3d R0 = state.attitude.toRotationMatrix();
  const Eigen::Vector3d aWorld = R0 * state.accel_corrected + Eigen::Vector3d(0,0,-9.80665);
  const Eigen::Vector3d w = state.gyro_corrected;
  const double theta = w.norm()*dt;
  Eigen::Quaterniond dq;
  if(theta > 1e-9) dq = Eigen::Quaterniond(Eigen::AngleAxisd(theta, w.normalized()));
  else dq = Eigen::Quaterniond(1, 0.5*w.x()*dt, 0.5*w.y()*dt, 0.5*w.z()*dt).normalized();
  state.position += state.velocity*dt + 0.5*aWorld*dt*dt;
  state.velocity += aWorld*dt;
  state.attitude = (state.attitude*dq).normalized();

  Mat15 F = Mat15::Identity();
  F.block<3,3>(0,3) = Eigen::Matrix3d::Identity()*dt;
  F.block<3,3>(0,6) = -0.5*R0*skew(state.accel_corrected)*dt*dt;
  F.block<3,3>(0,9) = -0.5*R0*dt*dt;
  F.block<3,3>(3,6) = -R0*skew(state.accel_corrected)*dt;
  F.block<3,3>(3,9) = -R0*dt;
  F.block<3,3>(6,6) = Eigen::Matrix3d::Identity()-skew(w)*dt;
  F.block<3,3>(6,12) = -Eigen::Matrix3d::Identity()*dt;

  // Continuous white-noise density: accel, gyro, accel-bias RW, gyro-bias RW.
  Mat15x12 G = Mat15x12::Zero();
  G.block<3,3>(0,0) = 0.5*R0*dt*dt;
  G.block<3,3>(3,0) = R0*dt;
  G.block<3,3>(6,3) = Eigen::Matrix3d::Identity()*dt;
  G.block<3,3>(9,6) = Eigen::Matrix3d::Identity()*sqrt(dt);
  G.block<3,3>(12,9) = Eigen::Matrix3d::Identity()*sqrt(dt);
  Eigen::Matrix<double,12,12> N = Eigen::Matrix<double,12,12>::Zero();
  N.block<3,3>(0,0).diagonal().setConstant(0.35*0.35/dt);
  N.block<3,3>(3,3).diagonal().setConstant(0.03*0.03/dt);
  N.block<3,3>(6,6).diagonal().setConstant(0.003*0.003);
  N.block<3,3>(9,9).diagonal().setConstant(0.0003*0.0003);
  P = F*P*F.transpose() + G*N*G.transpose();
  P = 0.5*(P+P.transpose()).eval();
}

void EKF::update(double gps_x, double gps_y, double variance) {
  if(!std::isfinite(gps_x) || !std::isfinite(gps_y) || !(variance>0)) return;
  Eigen::Matrix<double,2,15> H = Eigen::Matrix<double,2,15>::Zero();
  H(0,0)=1; H(1,1)=1;
  Eigen::Vector2d residual(gps_x-state.position.x(),gps_y-state.position.y());
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity()*variance;
  Eigen::Matrix2d S = H*P*H.transpose()+R;
  if((residual.transpose()*S.ldlt().solve(residual))(0,0) > 25.0) return; // outlier gate
  Eigen::Matrix<double,15,2> K = P*H.transpose()*S.ldlt().solve(Eigen::Matrix2d::Identity());
  Eigen::Matrix<double,15,1> dx = K*residual;
  state.position += dx.segment<3>(0);
  state.velocity += dx.segment<3>(3);
  Eigen::Vector3d dtheta = dx.segment<3>(6);
  Eigen::Quaterniond dq(1,0.5*dtheta.x(),0.5*dtheta.y(),0.5*dtheta.z());
  state.attitude = (state.attitude*dq.normalized()).normalized();
  state.accel_bias += dx.segment<3>(9);
  state.gyro_bias += dx.segment<3>(12);
  Mat15 I = Mat15::Identity();
  Mat15 A = I-K*H;
  P = A*P*A.transpose()+K*R*K.transpose();
  // Right-multiplicative error-state reset Jacobian (first-order).
  Mat15 J = Mat15::Identity();
  J.block<3,3>(6,6) -= 0.5*skew(dtheta);
  P = J*P*J.transpose();
  P = 0.5*(P+P.transpose()).eval();
}
