#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
  const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if (estimations.size() == 0)
  {
    cout << "CalculateRMSE () - Error - estimations vector is empty" << endl;
    return rmse;
  }

  if (estimations.size() != ground_truth.size())
  {
    cout << "CalculateRMSE () - Error - estimations and ground truth vectors are different size" << endl;
    return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    VectorXd residual = estimations[i]-ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // helper variables for Jacobian matrix
  float px2 = px * px;
  float py2 = py * py;

  float sum_px2_py2 = px2 + py2;
  float sqrt_sum2 = sqrt(sum_px2_py2);
  float sqrt3_sum2 = pow(sqrt_sum2,3);

  //check division by zero
  if(fabs(sum_px2_py2) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  float px_by_sqrt_sum2 = px/sqrt_sum2;
  float py_by_sqrt_sum2 = py/sqrt_sum2;
  float py_by_sum_px2_py2 = py/sum_px2_py2;
  float px_by_sum_px2_py2 = px/sum_px2_py2;

  //compute the Jacobian matrix
  Hj << px_by_sqrt_sum2, py_by_sqrt_sum2, 0, 0,
        - py_by_sum_px2_py2, px_by_sum_px2_py2, 0, 0,
        py * (vx * py - vy * px)/sqrt3_sum2, px * (vy * px - vx * py)/sqrt3_sum2, px/sqrt_sum2, py/sqrt_sum2;
  return Hj;

}
