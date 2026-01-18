#include "auv_control/pid.hpp"

#include <algorithm>
#include <cmath>

namespace auv_control {

PID::PID(double kp, double ki, double kd, double max_output, double max_integral)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      max_output_(max_output),
      max_integral_(max_integral),
      integral_(0.0),
      prev_error_(0.0),
      first_run_(true) {}

double PID::compute(double error, double dt) {
  // Handle zero or negative dt
  if (dt <= 0.0) {
    return 0.0;
  }

  // Proportional term
  double p_term = kp_ * error;

  // Integral term with anti-windup
  integral_ += ki_ * error * dt;
  integral_ = std::clamp(integral_, -max_integral_, max_integral_);
  double i_term = integral_;

  // Derivative term (skip on first run to avoid spike)
  double d_term = 0.0;
  if (!first_run_) {
    d_term = kd_ * (error - prev_error_) / dt;
  }

  // Update state for next iteration
  prev_error_ = error;
  first_run_ = false;

  // Compute output and clamp
  double output = p_term + i_term + d_term;
  output = std::clamp(output, -max_output_, max_output_);

  return output;
}

void PID::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
  first_run_ = true;
}

void PID::setGains(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

}  // namespace auv_control
