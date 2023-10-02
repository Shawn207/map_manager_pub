/*
	FILE: kalman_filter.cpp
	--------------------------------------
	function definition of kalman_filter velocity estimator
*/
#include <Eigen/Dense>
#include <map_manager/detector/kalman_filter.h>

using Eigen::MatrixXd;
using namespace std;

kalman_filter::kalman_filter()
{
    this->is_initialized = false;
}

void kalman_filter::setup(MatrixXd states, MatrixXd A, MatrixXd B, MatrixXd H, MatrixXd P, MatrixXd Q, MatrixXd R)
{
    this->states = states;
    this->A = A;
    this->B = B;
    this->H = H;
    this->P = P;
    this->Q = Q;
    this->R = R;
    this->is_initialized = true;
}

void kalman_filter::setA(MatrixXd A)
{
    this->A = A;
}

void kalman_filter::estimate(MatrixXd z, MatrixXd u)
{
    // predict
    this->states = this->A * this->states + this->B * u;
    this->P = this->A * this->P * this->A.transpose() + this->Q;

    // update
    MatrixXd S = this->R + this->H * this->P * this->H.transpose(); // innovation matrix
    MatrixXd K = this->P * this->H.transpose() * S.inverse(); // kalman gain
    this->states = this->states + K * (z - this->H * this->states);

    this->P = (MatrixXd::Identity(this->P.rows(),this->P.cols()) - K * this->H) * this->P;
}

double kalman_filter::output(int state_index)
{
    if(this->is_initialized)
    {
        return this->states(state_index, 0);
    }
    else
    {
        return 0;
    }
}