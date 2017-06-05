#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd) {
  
  Kp = kp;
  Ki = ki;
  Kd = kd;
  
  dp = 1;
  di = 1;
  dd = 1;
  
  p_error = 0;
  i_error = 0;
  d_error = 0;
  sq_error = 0;
  best_sq_error = 99999999;
  
  pre_cte = 0;
  sum_cte = 0;
  
}

void PID::UpdateError(double cte) {
  
  sum_cte += cte;
  sq_error += cte * cte;
  
  p_error = Kp * cte;
  i_error = Ki * sum_cte;
  d_error = Kd * (cte - pre_cte);
  
  pre_cte = cte;
  
}

double PID::TotalError() {
  
  return p_error + i_error + d_error;
  
}

