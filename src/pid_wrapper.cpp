#include "assert.h"
#include "PID.h"
#include "pid_wrapper.h"
#include <vector>

using namespace std;

// TODO: Move this to a common place so that it can be reused by PID.
namespace {
  vector<string> kHeaderItems =
      {"GLOBAL_STEP", "CYCLE_STEP", "KP", "KI", "KD", "PARAM_BEING_UPDATED",
       "BEST_ERROR_IN_CYCLE", "TOTAL_ERROR_IN_CYCLE"};

  /** Joins the string elements of a vector into a pretty-print string. */
  string JoinItems(const vector<string>& items) {
    string join = "";
    for (int i = 0; i < items.size(); i++) {
      join += items[i];
      join += (i == items.size() - 1) ? "\n" : "\t";
    }
    return join;
  }

  string JoinItems(const vector<double>& items) {
    vector<string> items_str;
    for (const double item : items) {
      items_str.push_back(to_string(item));
    }
    return JoinItems(items_str);
  }
}  // end anonymous namespace

PIDWrapper::PIDWrapper(
    double Kp, double Ki, double Kd,
    int num_noneval_steps, int num_eval_steps, double stop_criterium,
    string run_id) {
  pid.Init(Kp, Ki, Kd, run_id); 

  this->num_noneval_steps = num_noneval_steps;
  this->num_eval_steps = num_eval_steps;
  this->stop_criterium = stop_criterium;
  this->run_id = run_id;

  // These are the same order of magnitude as the initial parameters
  // (0.2, 0.0004, 3.0).
  dp_ = {0.1, 0.0005, 1.0};
  debug_file_.open("/tmp/pid_wrapper_debug_" + run_id + ".txt");
  debug_file_ << JoinItems(kHeaderItems);
  debug_file_.flush();
}

PIDWrapper::~PIDWrapper() {
  debug_file_.close();
}

void PIDWrapper::UpdateError(double cte) {
  global_step_ += 1;
  cycle_step_ += 1;

  pid.UpdateError(cte);
  if (!ShouldTwiddle()) {
    return;
  }

  if (IsEvalStep()) {
    total_error_ += cte * cte;
    if (best_error_ > 0.0 && best_error_ < total_error_) {
      ApplyNextUpdate();  // Early stopping. The error is already too high.
    }
  }

  if (IsEndOfCycle()) {
    if (best_error_ == 0.0) {
      best_error_ = total_error_;
    }
    ApplyNextUpdate();
  }

  vector<double> debug_items =
      {static_cast<double>(global_step_),
       static_cast<double>(cycle_step_),
       pid.Kp, pid.Ki, pid.Kd,
       static_cast<double>(current_param_),
       best_error_, total_error_};
  assert(debug_items.size() == kHeaderItems.size());
  debug_file_ << JoinItems(debug_items);
  debug_file_.flush();
}

double PIDWrapper::TotalError() {
  return pid.TotalError();
}

bool PIDWrapper::ShouldTwiddle() {
  return num_noneval_steps > 0 &&
         num_eval_steps > 0 &&
         dp_[0] + dp_[1] + dp_[2] > stop_criterium;
}

bool PIDWrapper::IsEvalStep() {
  return cycle_step_ > num_noneval_steps;
}

bool PIDWrapper::IsEndOfCycle() {
  return cycle_step_ == num_noneval_steps + num_eval_steps;
}

void PIDWrapper::UpdateCurrentParamByDPFactor(int dp_factor) {
  double delta = dp_factor * dp_[current_param_];
  switch (current_param_) {
    case 0: { pid.Kp += delta; break; }
    case 1: { pid.Ki += delta; break; }
    case 2: { pid.Kd += delta; break; }
    default: assert(false);
  }
}

bool PIDWrapper::UpdateWasBeneficial() {
  bool was_beneficial = (total_error_ < best_error_);
  if (was_beneficial) {
    best_error_ = total_error_;
    dp_[current_param_] *= 1.1;
  }
  return was_beneficial;
}

void PIDWrapper::MoveToNextParam() {
  assert(attempted_increment_ || attempted_decrement_);
  current_param_ = (current_param_ + 1) % 3;
  attempted_increment_ = attempted_decrement_ = false;
}

void PIDWrapper::AttemptIncrement() {
  assert(!attempted_increment_ && !attempted_decrement_);
  UpdateCurrentParamByDPFactor(1);
  attempted_increment_ = true;
}

void PIDWrapper::AttemptDecrement() {
  assert(attempted_increment_ && !attempted_decrement_);
  UpdateCurrentParamByDPFactor(-2);
  attempted_decrement_ = true;
}

void PIDWrapper::UndoDecrement() {
  assert(attempted_increment_ && attempted_decrement_);
  UpdateCurrentParamByDPFactor(1);
  dp_[current_param_] *= 0.9; 
}

void PIDWrapper::ApplyNextUpdate() {
  if (!attempted_increment_ && !attempted_decrement_) {
    AttemptIncrement();
  } else if (UpdateWasBeneficial()) {
    MoveToNextParam();
    AttemptIncrement();
  } else if (attempted_decrement_) {
    UndoDecrement();
    MoveToNextParam();
    AttemptIncrement();
  } else {
    AttemptDecrement();
  }

  cycle_step_ = 0;
  total_error_ = 0; 
  pid.Reset();
}
