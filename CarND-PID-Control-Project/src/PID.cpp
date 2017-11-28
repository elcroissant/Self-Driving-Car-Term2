#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double steer_Kp, double steer_Ki, double steer_Kd, double speed_Kp, double speed_Ki, double speed_Kd) {
    steer_p_error_ = 0;
    steer_i_error_ = 0;
    steer_d_error_ = 0;
    steer_Kp_ = steer_Kp;
    steer_Ki_ = steer_Ki;
    steer_Kd_ = steer_Kd;
    speed_p_error_ = 0;
    speed_i_error_ = 0;
    speed_d_error_ = 0;
    speed_Kp_ = steer_Kp;
    speed_Ki_ = steer_Ki;
    speed_Kd_ = steer_Kd;  
}

void PID::UpdateError(double steer_cte, double speed_cte) {
    double prev_steer_cte = steer_p_error_;
    steer_p_error_ = steer_cte;
    steer_i_error_ += steer_cte;
    steer_d_error_ = steer_cte - prev_steer_cte;

    cout << "p_error: " << steer_p_error_ << " d_error: " << steer_d_error_ << " i_error: " << steer_i_error_ << endl;

    double prev_speed_cte = speed_p_error_;
    speed_p_error_ = speed_cte;
    speed_i_error_ += speed_cte;
    speed_d_error_ = speed_cte - prev_speed_cte;
}

double PID::TotalSteerError() {
    return steer_Kp_ * steer_p_error_ + steer_Ki_ * steer_i_error_ + steer_Kd_ * steer_d_error_;
}


double PID::TotalSpeedError() {
    return speed_Kp_ * speed_p_error_ + speed_Ki_ * speed_i_error_ + speed_Kd_ * speed_d_error_;
}

