#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.2;
  
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.0175;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.1;

  // 
  is_initialized_ = false;

  // set state dimention
  n_x_ = 5;

  // set spreading parameter
  lambda_ = 3 - n_x_;

  // initialize sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_x_ + 1);
  
  // set augumented dimention
  n_aug_ = 7;

  //create vector for weights
  weights_ = VectorXd(2*n_aug_+1);
    
  //create vector for predicted state
  x_ = VectorXd(n_x_);
  
  //create covariance matrix for prediction
  P_ = MatrixXd(n_x_, n_x_);


  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0,std_radrd_*std_radrd_;

  R_laser_ = MatrixXd(2,2);
  R_laser_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

  NIS_radar_ = 0.0;
  NIS_laser_ = 0.0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  
  if (!is_initialized_) 
  {
  
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) { 
      // Convert radar from polar to cartesian coordinates and initialize state. 
      float ro     = meas_package.raw_measurements_(0); 
      float phi    = meas_package.raw_measurements_(1); 
      float ro_dot = meas_package.raw_measurements_(2); 
      x_ << ro * cos(phi), ro * sin(phi), ro_dot * cos(phi), ro_dot * sin(phi); 
    } 
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) { 
      // Initialize state the initial location and zero velocity 
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0; 
    } 
    //remember previous timestamp 
    time_us_ = meas_package.timestamp_; 

    // done initializing, no need to predict or update 
    is_initialized_ = true; 
    return; 
  }

  //Compute the time elapsed between the current and previous measurements 
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds 
  time_us_ = meas_package.timestamp_; 

  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) { 
    UpdateRadar(meas_package); 
  } 
  else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) { 
    UpdateLidar(meas_package); 
  } 
  

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
 
    //create augmented mean vector
    VectorXd x_aug = VectorXd(7);      //create augmented mean state
   
    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);
  
    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    x_aug.head(n_x_) = x_;
    x_aug(n_aug_-2) = 0;
    x_aug(n_aug_-1) = 0;
  
    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_,n_x_) = P_;
    P_aug(n_aug_-2,n_aug_-2) = std_a_*std_a_;
    P_aug(n_aug_-1,n_aug_-1) = std_yawdd_*std_yawdd_;
  
    //create square root matrix
    MatrixXd A_aug = P_aug.llt().matrixL();
    
    //create augmented sigma points
    Xsig_aug.col(0) = x_aug;
   
    for (int i = 0; i < n_aug_; i++)
    {
      Xsig_aug.col(i+1) = x_aug + A_aug.col(i)*sqrt(lambda_+n_aug_);
      Xsig_aug.col(n_aug_+1+i) = x_aug - A_aug.col(i)*sqrt(lambda_+n_aug_);
    }

    // Sigma point prediction
    for (int i = 0; i < 2* n_aug_ + 1; i++)
    {
      VectorXd sig_pred = VectorXd(5);
      sig_pred = Xsig_aug.col(i);
      
      double v = sig_pred(2);
      double yaw = sig_pred(3);
      double yaw_d = sig_pred(4);
      double nu_a = sig_pred(5);
      double nu_psidd = sig_pred(6);
      
      //predict sigma points
      if (yaw_d != 0)
      {
          sig_pred(0) += v/yaw_d * (sin(yaw + yaw_d*delta_t)-sin(yaw));
          sig_pred(1) += v/yaw_d * (cos(yaw)-cos(yaw + yaw_d*delta_t));
      }
      else
      {
          sig_pred(0) += v*cos(yaw)*delta_t;
          sig_pred(1) += v*sin(yaw)*delta_t;
      }
      sig_pred(2) += 0;
      sig_pred(3) += yaw_d*delta_t;
      sig_pred(4) += 0;
     
      // adding noise
      sig_pred(0) += 0.5*delta_t*delta_t*nu_a*cos(yaw);
      sig_pred(1) += 0.5*delta_t*delta_t*nu_a*sin(yaw);
      
      
      sig_pred(2) += delta_t*nu_a;
      sig_pred(3) += 0.5*delta_t*delta_t*nu_psidd;
      sig_pred(4) += delta_t*nu_psidd;
  
     //write predicted sigma points into right column
     Xsig_pred_.col(i) = sig_pred.head(5);
    }

  // 3. Predicted mean and covariance
  //set weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i=1; i < 2*n_aug_+1; i++)
  {
      weights_(i) = 0.5/(lambda_ + n_aug_);
  }

  //predict state mean
  x_.fill(0);
  for (int i=0; i < 2*n_aug_+1; i++)
  {
      x_ += weights_(i) * Xsig_pred_.col(i);
  }
  
  //predict state covariance matrix
  P_.fill(0);
  for (int i=0; i < 2*n_aug_+1; i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
      
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
  

}

void UKF::UpdateUKF(MeasurementPackage meas_package, MatrixXd Zsig, int n_z)
{
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
 
  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++)
  {
      z_pred += weights_(i) * Zsig.col(i);
  }
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++)
  {
    // residual
    VectorXd z_diff = VectorXd(n_z);
    z_diff = Zsig.col(i) - z_pred;
      
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S += weights_(i) * z_diff * z_diff.transpose();
  }
  
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  { 
      R = R_radar_;
  }
  else if(meas_package.sensor_type_ = MeasurementPackage::LASER)
  {
      R = R_laser_;
  }


  S = S + R;

  // STEP 2 UPDATE STATE
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i=0; i< 2*n_aug_+1; i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i)-x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    VectorXd z_diff = Zsig.col(i)-z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    
    Tc += weights_(i)*x_diff*z_diff.transpose();
  }
  //calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();
  //update state mean and covariance matrix
  
  // Measurements
  VectorXd z = meas_package.raw_measurements_;
  // REsidula 
 VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  
  x_ = x_ + K*z_diff;
  P_ = P_ - K*S*K.transpose();

  //calculate NIS
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  { 
    NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  }
  else if(meas_package.sensor_type_ = MeasurementPackage::LASER)
  {
    NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  
  */
  //set measurement dimension
  int n_z = 2;
   //create matrix for sigma points in measurement space
   MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
   for (int i = 0; i < 2* n_aug_ + 1; i++)
   {
      Zsig(0, i) = Xsig_pred_(0,i);
      Zsig(1, i) = Xsig_pred_(1,i);
   }
   UpdateUKF(meas_package, Zsig, n_z);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
   //transform sigma points into measurement space
   for (int i = 0; i < 2* n_aug_ + 1; i++)
   {
     VectorXd sig_pred = VectorXd(5);
     sig_pred = Xsig_pred_.col(i);
     double px = sig_pred(0);
     double py = sig_pred(1);
     double v = sig_pred(2);
     double yaw = sig_pred(3);
     double yaw_d = sig_pred(4);
   
     double sum_sqrt = sqrt(px*px + py*py);
     Zsig(0,i) = sum_sqrt;
     Zsig(1,i) = atan2(py,px);
     Zsig(2,i) = (px*cos(yaw)*v + py*sin(yaw)*v) / sum_sqrt;
   }
  UpdateUKF(meas_package, Zsig, n_z);   
}

