#include "PID.h"
#include <iostream>
#include <numeric>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double steer_Kp, double steer_Ki, double steer_Kd, double speed_Kp, double speed_Ki, double speed_Kd) {
    steer_p_error_ = 0;
    steer_i_error_.clear();
    steer_d_error_ = 0;
    steer_Kp_ = steer_Kp;
    steer_Ki_ = steer_Ki;
    steer_Kd_ = steer_Kd;
    speed_p_error_ = 0;
    speed_i_error_.clear();
    speed_d_error_ = 0;
    speed_Kp_ = steer_Kp;
    speed_Ki_ = steer_Ki;
    speed_Kd_ = steer_Kd;  
}

void PID::UpdateError(double steer_cte, double speed_cte) {
    double prev_steer_cte = steer_p_error_;
    steer_p_error_ = steer_cte;

    if(steer_i_error_.size() >= 10)
    {
        cout << "Remove first: " << *(steer_i_error_.begin()) << endl;
        steer_i_error_.erase(steer_i_error_.begin());
    }
    steer_i_error_.push_back(steer_cte);

    steer_d_error_ = steer_cte - prev_steer_cte;

    cout << "p_error: " << steer_p_error_ << " d_error: " << steer_d_error_ << " i_error: " <<  std::accumulate(steer_i_error_.begin(), steer_i_error_.end(), 0.0) << endl;

    double prev_speed_cte = speed_p_error_;
    speed_p_error_ = speed_cte;

    if(speed_i_error_.size() >= 10)
    {
        cout << "Removed first: " << *(speed_i_error_.begin()) << endl;
        speed_i_error_.erase(speed_i_error_.begin());
    }
    speed_i_error_.push_back(speed_cte);

    speed_d_error_ = speed_cte - prev_speed_cte;
}

double PID::TotalSteerError() {
    return steer_Kp_ * steer_p_error_ + steer_Ki_ * std::accumulate(steer_i_error_.begin(), steer_i_error_.end(), 0.0) + steer_Kd_ * steer_d_error_;
}


double PID::TotalSpeedError() {
    return speed_Kp_ * speed_p_error_ + speed_Ki_ * std::accumulate(speed_i_error_.begin(), speed_i_error_.end(), 0.0) + speed_Kd_ * speed_d_error_;
}

