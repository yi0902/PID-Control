#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  
  double sq_error;
  double best_sq_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  /*
   * Delta of Coefficients
   */
  double dp;
  double di;
  double dd;
    
  /*
  * Cross-tracking error
  */
  double pre_cte;
  double sum_cte;

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
  void Init(double kp, double ki, double kd);

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
