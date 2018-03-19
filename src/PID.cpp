#include "PID.h"
#include <algorithm>

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = i_error = d_error = 0.0;
  prev_ctes.clear();
  max_abs_error = 0.5;    // avoid extreme steer angle
  max_abs_d_error = 1;    // in case of sudden change in cte
  max_abs_i_error = 1;    // put an upper limit of accumulated error
  max_abs_p_error = 1;
}

void PID::UpdateError(double cte) {
  p_error = cte * Kp;
  p_error = std::max(std::min(p_error, max_abs_p_error), -max_abs_p_error);

  i_error += cte * Ki;
  i_error = std::max(std::min(i_error, max_abs_i_error), -max_abs_i_error);

  // use a digital filter for derivative calculation to reduce noise
  prev_ctes.push_back(cte);
  if (prev_ctes.size() < 5) {d_error = 0.0;}
  else {
    d_error = 0.0;
    for(int i = 0; i < 5; ++i){
      d_error += (prev_ctes[i] * deriv_filter[i] * Kd);
    }
    prev_ctes.pop_front();
    d_error = std::max(std::min(d_error, max_abs_d_error), -max_abs_d_error);
  }
}

double PID::TotalError() {
  double total_error = p_error + i_error + d_error ;
  return std::max(std::min(total_error, max_abs_error), -max_abs_error);
}

