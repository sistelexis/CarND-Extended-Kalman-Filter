#include "kalman_filter.h"
#include <iostream>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  /**
  TODO:
    * predict the state - START
  */
  x_ = F_ * x_ ; 
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  /**
  TODO:
    * predict the state - END
  */
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations - START
  */
  VectorXd y = z - H_ * x_; 

  KF(y);
  /**
  TODO:
    * update the state by using Kalman Filter equations - END
  */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations - START
  */
  double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  //double phi = atan(x_(1) / x_(0));
  double phi = atan2(x_(1) , x_(0));
  double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
  VectorXd h = VectorXd(3); // h(x_)
  h << rho, phi, rho_dot;
  
  VectorXd y = z - h;
  
  while (y(1)<-M_PI) {
    y(1) += 2 * M_PI;
  }
  while (y(1)>M_PI) {
    y(1) -= 2 * M_PI;
  }
  
  KF(y);	
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations - END
  */
}

// Universal update Kalman Filter step. Equations from the lectures
void KalmanFilter::KF(const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  // New state
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}