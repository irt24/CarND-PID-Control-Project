#include "PID.h"
#include "assert.h"
#include <iostream>
#include <vector>

using namespace std;

// TODO: Move this to a common place so that it can be reused by pid_wrapper.
namespace {
  std::vector<string> kHeaderItems =
      {"CTE", "CTE_PREV", "CTE_SUM", "KP", "KI", "KD", "P_ERROR", "I_ERROR", "D_ERROR"};

  /** Joins the string elements of a vector into a pretty-print string. */
  string JoinItems(const std::vector<string>& items) {
    string join = "";
    for (int i = 0; i < items.size(); i++) {
      join += items[i];
      join += (i == items.size() - 1) ? "\n" : "\t";
    }
    return join;
  }

  string JoinItems(const std::vector<double>& items) {
    std::vector<string> items_str;
    for (const double item : items) {
      items_str.push_back(std::to_string(item));
    }
    return JoinItems(items_str);
  }
}  // end anonymous namespace

PID::PID() {}

PID::~PID() {
  debug_file_.close();
}

void PID::Init(double Kp, double Ki, double Kd, string run_id) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->run_id = run_id;
  Reset();

  // Print the header to the debug file.
  debug_file_.open("/tmp/pid_debug_" + run_id + ".txt");
  debug_file_ << JoinItems(kHeaderItems);
  debug_file_.flush();
}

void PID::UpdateError(double cte) {
  // Update the PROPORTIONAL error.
  p_error = Kp * cte;

  // Update the INTEGRAL error.
  cte_sum_ += cte;
  i_error = Ki * cte_sum_;

  // Update the DIFFERENTIAL error.
  d_error = cte_prev_ == 0.0 ? 0.0 : Kd * (cte - cte_prev_);
  cte_prev_ = cte;

  // Print debug info.
  std::vector<double> debug_items =
      {cte, cte_prev_, cte_sum_, Kp, Ki, Kd, p_error, i_error, d_error};
  assert(debug_items.size() == kHeaderItems.size());
  debug_file_ << JoinItems(debug_items);
  debug_file_.flush();
}

double PID::TotalError() {
  return p_error + i_error + d_error;
}

void PID::Reset() {
  p_error = i_error = d_error = 0.0;
  cte_prev_ = cte_sum_ = 0.0;
}

