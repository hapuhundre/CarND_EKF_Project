#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // 输入有效性验证
  if(estimations.size() != ground_truth.size() || estimations.size()==0 ){
    cout << "Invalid input data" << endl;
    return rmse;
  }
	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd res = estimations[i] - ground_truth[i];

		// coefficient-wise 乘法
    // more to see: https://eigen.tuxfamily.org/dox/group__TutorialArrayClass.html
		res = res.array()*res.array();
		rmse += res;
	}
	// 平均值
	rmse = rmse/estimations.size();
	// 计算平方根
	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
    计算雅可比矩阵
  */
  MatrixXd H_j(3,4);
  H_j << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;
  
  float p_x = x_state(0);
  float p_y = x_state(1);
  float v_x = x_state(2);
  float v_y = x_state(3);
  
  float h1 = p_x*p_x + p_y*p_y;
  float h2 = sqrt(h1);
  float h3 = h1*h2;

  // 判断是否除以0
  if(fabs(h1) < 0.0001){
    cout << "Jacobian Matrix Error! Division by Zero" << endl;
    return H_j;
  }

  H_j << p_x/h2, p_y/h2, 0, 0,
         -p_y/h1, p_x/h1, 0, 0,
         p_y*(v_x*p_y-v_y*p_x)/h3, p_x*(p_x*v_y-v_x*p_y)/h3, p_x/h2, p_y/h2;
  return H_j;
}
