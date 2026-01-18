#ifndef AUV_CONTROL__PID_HPP_
#define AUV_CONTROL__PID_HPP_

namespace auv_control {

class PID {
public:
  /**
   * @brief Construct a new PID controller
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   * @param max_output Maximum absolute value of output (output clamped to [-max_output, max_output])
   * @param max_integral Maximum absolute value of integral term (anti-windup)
   */
  PID(double kp, double ki, double kd, double max_output, double max_integral);

  /**
   * @brief Compute control output given error and time step
   * @param error Current error (setpoint - measurement)
   * @param dt Time step in seconds
   * @return Control output clamped to [-max_output, max_output]
   */
  double compute(double error, double dt);

  /**
   * @brief Reset integral term and derivative state
   * Call on setpoint change or after large error to prevent windup
   */
  void reset();

  /**
   * @brief Update PID gains at runtime
   * @param kp New proportional gain
   * @param ki New integral gain
   * @param kd New derivative gain
   */
  void setGains(double kp, double ki, double kd);

private:
  double kp_;
  double ki_;
  double kd_;
  double max_output_;
  double max_integral_;
  double integral_;
  double prev_error_;
  bool first_run_;
};

}  // namespace auv_control

#endif  // AUV_CONTROL__PID_HPP_
