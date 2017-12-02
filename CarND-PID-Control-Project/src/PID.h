#ifndef PID_H
#define PID_H


#include <vector>

class PID {
private:
  /*
  * Errors for steering
  */
  double p_error_;
  std::vector<double> i_error_;
  double d_error_;

  /*
  * Coefficients for steering
  */ 
  double Kp_;
  double Ki_;
  double Kd_;



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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total steering PID error.
  */
  double TotalError();
  
  };

#endif /* PID_H */
