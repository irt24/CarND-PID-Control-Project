#ifndef PID_H
#define PID_H

#include <fstream>

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

  /** Used to name the debug output file. */
  std::string run_id;

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
  void Init(double Kp, double Ki, double Kd, std::string run_id);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Resets internal state (excluding coefficients).
  */
  void Reset();

 private:
  std::ofstream debug_file_;
  double cte_prev_ = 0.0;
  double cte_sum_ = 0.0;
};

#endif /* PID_H */
