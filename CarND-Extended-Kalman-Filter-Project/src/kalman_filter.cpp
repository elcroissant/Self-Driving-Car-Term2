#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // The predicted measurement vector x' is a vector containing values 
  // in the form [px,py,vx,vy]. The radar sensor will output values in polar coordinates.
  // In order to calculate y for the radar sensor, we need to convert x' to polar coordinates. 
  // In other words, the function h(x) maps values from Cartesian coordinates to polar coordinates. 
  // So the equation for radar becomes y=z_radar-h(x').
  VectorXd z_pred(3);
  
  // Unpack the state vector
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  

  z_pred[0] = sqrt(px*px + py*py);
  
  // px correction if necessary
  if(fabs(px) < 0.0001){
    px = 0.0001;
  }
  
  z_pred[1] = atan(py/px);
  
  if (fabs(z_pred[0]) < 0.0001) {
    z_pred[0] = 0.0001;
  }

  z_pred[2] = (px*vx + py*vy) / z_pred[0];
  
  VectorXd y = z - z_pred;
//////
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
