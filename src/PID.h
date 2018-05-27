#ifndef PID_H
#define PID_H
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
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle variables
  */
  std::vector<double> dp;
  int step;
  int param_index;
  int per_lap_steps;

  double lap_error;
  double best_error;
  
  bool try_positive;
  bool try_negative;
  bool enable_twiddle;

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

  /*
  * Update PID controller parameter by adding dp value which is based on index
  */
  void UpdateParameter(int index, double value);
};

#endif /* PID_H */