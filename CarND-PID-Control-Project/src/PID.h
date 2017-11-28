#ifndef PID_H
#define PID_H

class PID {
private:
  /*
  * Errors for steering
  */
  double steer_p_error_;
  double steer_i_error_;
  double steer_d_error_;

  /*
  * Coefficients for steering
  */ 
  double steer_Kp_;
  double steer_Ki_;
  double steer_Kd_;

 /*
  * Errors for speed
  */
  double speed_p_error_;
  double speed_i_error_;
  double speed_d_error_;

  /*
  * Coefficients for speed
  */ 
  double speed_Kp_;
  double speed_Ki_;
  double speed_Kd_;

public:
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double steer_Kp, double steer_Ki, double steer_Kd, double speed_Kp, double speed_Ki, double speed_Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double steer_cte, double speed_cte);

  /*
  * Calculate the total steering PID error.
  */
  double TotalSteerError();
  
  /*
  * Calculate the total speed PID error.
  */
  double TotalSpeedError();
};

#endif /* PID_H */
