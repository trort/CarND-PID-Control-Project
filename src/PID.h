#ifndef PID_H
#define PID_H

#include <deque>
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Maximum abs Errors
  */
  double max_abs_p_error;
  double max_abs_i_error;
  double max_abs_d_error;
  double max_abs_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * previous cte value and flag, used to calculate derivative
   */
  std::deque<double> prev_ctes;
  std::vector<double> deriv_filter = {-0.2, -0.1, 0, 0.1, 0.2};

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
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
