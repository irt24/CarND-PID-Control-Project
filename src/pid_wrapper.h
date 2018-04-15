#ifndef PID_WRAPPER_H
#define PID_WRAPPER_H 

#include <fstream>
#include <vector>

/**
 * Wraps the PID controller with the Twiddle algorithm.
 *
 * The Twiddle algorithm proceeds in "cycles". One cycle corresponds to a single
 * state of the PID coefficients and consists of two phases: a "non-eval" phase
 * in which CTE accummulates, and a "eval" phase in which we measure the
 * performance of the system given the fixed set of coefficients. The algorithm
 * stops whenever the sum of the coefficient deltas is small.
 */
class PIDWrapper {
 public:
  /**
   * Builds a new PIDWrapper from the following arguments:
   *    Kp: Coefficient for "proportional" component.
   *    Ki: Coefficient for "integral" component.
   *    Kd: Coefficient for "differential" component.
   *    num_noneval_steps: The number of steps (error updates) in a cycle before
   *        evaluation starts.
   *    num_eval_steps: The number of steps in a cycle to evaluate for.  
   *    stop_criterium: When the sum of the twiddle deltas is below this value
   *        (i.e. the "dp" vector in the course), the twiddler algorithm stops. 
   *    run_id: A string describing the run; Will be used for debug filenames.
   */
  PIDWrapper(
      double Kp, double Ki, double Kd,
      int num_noneval_steps, int num_eval_steps, double stop_criterium,
      std::string run_id);

  /** Destructor. */
  ~PIDWrapper();

  /** Calls UpdateError() on the underlying PID controller and runs twiddle. */
  void UpdateError(double cte);

  /** Calls TotalError() on the underlying PID controller. */
  double TotalError();

  PID pid;
  int num_noneval_steps;
  int num_eval_steps;
  double stop_criterium;
  std::string run_id;

 private:
  bool ShouldTwiddle();
  bool IsEvalStep();
  bool IsEndOfCycle();
  // In practice, dp_factor will be either 1 or -2.
  void UpdateCurrentParamByDPFactor(int dp_factor);
  bool UpdateWasBeneficial();
  void MoveToNextParam();
  void AttemptIncrement();
  void AttemptDecrement();
  void UndoDecrement();
  void ApplyNextUpdate();

  std::ofstream debug_file_;
  std::vector<double> dp_;
  int global_step_ = 0;
  int cycle_step_ = 0;
  double total_error_ = 0.0;
  double best_error_ = 0.0;
  int current_param_ = 0;  // 0 => Kp, 1 => Ki, 2 => Kd.
  bool attempted_increment_ = false;
  bool attempted_decrement_ = false;
};

#endif /* PID_WRAPPER_H */
