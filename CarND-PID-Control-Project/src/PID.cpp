#include "PID.h"
#include <iostream>
#include <numeric>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    p_error_ = numeric_limits<double>::max();
    i_error_.clear();
    d_error_ = 0;
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

void PID::UpdateError(double cte) {
    double prev_cte = p_error_;
    p_error_ = cte;

    if(i_error_.size() >= 10)
    {
        //cout << "Remove first: " << *(i_error_.begin()) << endl;
        i_error_.erase(i_error_.begin());
    }
    i_error_.push_back(cte);

    d_error_ = (cte - prev_cte);

    cout << "p_error: " << p_error_ << " d_error: " << d_error_ << " i_error: " <<  std::accumulate(i_error_.begin(), i_error_.end(), 0.0) << endl;    
}

double PID::TotalError() {
    return -Kp_ * p_error_ -Ki_ * std::accumulate(i_error_.begin(), i_error_.end(), 0.0) -Kd_ * d_error_;
}


