#include <iostream>
#include "tools.h"
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != 0 && estimations.size() == ground_truth.size()) {


        //accumulate squared residuals
        for (int i = 0; i < estimations.size(); ++i) {

            VectorXd error = estimations[i] - ground_truth[i];

            //coefficient-wise multiplication
            error = error.array() * error.array();
            rmse += error;
        }

        //calculate the mean
        rmse = rmse / estimations.size();

        //calculate the squared root
        rmse = rmse.array().sqrt();
    } else
    {
        std::cout << "Error in estimation or ground_thruth vector dimesions" << std::endl;
    }
    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */


    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //TODO: YOUR CODE HERE


    float pxy2 = px*px + py*py;
    float sqrt_pxy2 = sqrt(pxy2);

    //check division by zero
    if(std::fabs(pxy2) <= 0.0001f){
        std::cout << "Division by 0" << std::endl;
        return Hj;
    }


    Hj(0,0) = px / sqrt_pxy2;
    Hj(0,1) = py / sqrt_pxy2;
    Hj(1,0) = (-1.0 * py)/pxy2;
    Hj(1,1) = px/pxy2;
    Hj(2,0) = py * (vx*py - vy*px)/(sqrt_pxy2*sqrt_pxy2*sqrt_pxy2);
    Hj(2,1) = px * (vy*px - vx*py)/(sqrt_pxy2*sqrt_pxy2*sqrt_pxy2);
    Hj(2,2) = px / sqrt_pxy2;
    Hj(2,3) = py / sqrt_pxy2;

    //compute the Jacobian matrix

    return Hj;
}
