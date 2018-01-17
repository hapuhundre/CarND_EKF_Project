#include "kalman_filter.h"

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
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * 基于卡尔曼方程更新状态
  */
	VectorXd y = z - H_*x_;
	MatrixXd Ht = H_.transpose();
  // ---------------
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd K = P_* Ht * S.inverse();

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

VectorXd Cartesian2Polar(const VectorXd &x_state){
  /*
    笛卡尔坐标转换为极坐标
    Radar: h(x')
    LADAR 直接乘以H即可
  */
  VectorXd polar = VectorXd(3);
  float p_x = x_state(0);
  float p_y = x_state(1);
  float v_x = x_state(2);
  float v_y = x_state(3);

  float ro = sqrt(p_x*p_x + p_y*p_y);
  // if(fabs(ro) < 0.0001)
  //   ro = 0.0001;
  polar << ro, atan2(p_y, p_x),
           (p_x*v_x + p_y*v_y) / ro;
  
  return polar;
}


void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * 扩展卡尔曼滤波
  */
  VectorXd polar = Cartesian2Polar(x_);
  VectorXd y = z - polar;


  // ------------------------------------------
  // 角度的正则化 -pi to pi
  // 必须要有，否则update会出错
  // a essential step to avoid big angle (if not, it would raise a big issue of result)
  while(y(1) > M_PI){
    y(1) -= (2*M_PI);
  }

  while(y(1) < -M_PI){
    y(1) += (2*M_PI);
  }
  // ---------------------------------------------

  // *************the same as Kalman filter*************************
  MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_; 
  // *****************************************
}
