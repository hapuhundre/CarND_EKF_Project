#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * 复制构造函数
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // 系数矩阵的初始化
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // LASER的测量协方差矩阵
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // RADAR的测量协方差矩阵
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
    * FusionEKF各个系数矩阵的初始化.  
   **/
  // 参考 L14 Find out the H Matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd(4, 4);
}

/**
* 析构函数
*/

FusionEKF::~FusionEKF() {}


void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  初始化
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * 将初始状态设置为1
      * 创建协方差矩阵
      * 注意雷达数据是极坐标（需要转换）
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      将极坐标转换为笛卡尔坐标并对状态初始化
      */
      cout << "Radar Init." << endl;
      float range = measurement_pack.raw_measurements_[0];     // 距原点的距离r
      float angle = measurement_pack.raw_measurements_[1];     // 与x轴的夹角
      float range_dot = measurement_pack.raw_measurements_[2]; // r'

      ekf_.x_ << range*cos(angle), range*sin(angle), range_dot*cos(angle), range_dot*sin(angle); // 位置与初始速度
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      笛卡尔坐标Initialize state.
      */
      cout << "Laser Init." << endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }
    // 卡尔曼滤波参数设置
    // 状态协方差矩阵 P 初始化
	  
	  ekf_.P_ << 1, 0, 0, 0,
			       0, 1, 0, 0,
			       0, 0, 1000, 0,
			       0, 0, 0, 1000;
  
    // 转移矩阵F_初始化
	  ekf_.F_ << 1, 0, 1, 0,
	      	   0, 1, 0, 1,
			       0, 0, 1, 0,
			       0, 0, 0, 1;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /*****************************************************************************
   *  预测
   ****************************************************************************/

  /**
   TODO:
     * 将时间步长带入状态转移矩阵中
      - 时间的单位为秒
     * 将噪声系数带入协方差矩阵Q
     * Q中的噪声系数均为9.0
   */
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	// 时间步长
	previous_timestamp_ = measurement_pack.timestamp_;
	
  // 1. 将F矩阵加入时间步长
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // 2. 设置过程协方差矩阵 Q
  float dt_2 = dt*dt;
  float dt_3 = dt_2*dt;
  float dt_4 = dt_3*dt;
  // 测量噪声设置
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4*noise_ax/4, 0, dt_3*noise_ax/2, 0,
	           0, dt_4*noise_ay/4, 0, dt_3*noise_ay/2,
		    	   dt_3*noise_ax/2, 0, dt_2*noise_ax, 0,
	           0, dt_3*noise_ay/2, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  测量更新
   ****************************************************************************/

  /**
     * 基于传感器类型跟新状态
     * 更新状态与协方差矩阵
   */
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    
    /*      Radar更新      */
    // H. R需要调整
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    /*     Laser更新   */
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
}
