#include "kalman_filter.h"
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

using namespace std;

#define pi 3.14159

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

  x_ = F_*x_; // + u;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;  
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  Tools tools;
  MatrixXd Hj = tools.CalculateJacobian(x_);
  VectorXd h;  
  h = VectorXd(3);
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  h << sqrt(px*px + py*py), atan(py/px), (px*vx + py*vy)/sqrt(px*px+py*py);
  
  while (h(1) > pi){
    h(1) = h(1) - 2*pi;
  }
  while (h(1) < -1*pi){
    h(1) = h(1) + 2*pi;
  }

  VectorXd y = z - h;
  MatrixXd Ht = Hj.transpose();
  cout << Hj << endl;
  cout << P_ << endl;
  cout << Ht << endl;  
  cout << R_ << endl;
  MatrixXd S = Hj * P_ * Ht; // + R_;
  // MatrixXd Si = S.inverse();
  // MatrixXd K =  P_ * Ht * Si;  
  // x_ = x_ + (K * y);
  // long x_size = x_.size();
  // MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // P_ = (I - K * Hj) * P_;
}
