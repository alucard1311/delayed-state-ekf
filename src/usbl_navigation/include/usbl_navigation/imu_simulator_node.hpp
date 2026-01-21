#ifndef USBL_NAVIGATION__IMU_SIMULATOR_NODE_HPP_
#define USBL_NAVIGATION__IMU_SIMULATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <random>

namespace usbl_navigation
{

/**
 * @brief IMU simulator node that adds realistic noise and bias drift
 *
 * Subscribes to truth odometry and publishes noisy IMU data at 100Hz.
 * Simulates gyroscope and accelerometer with:
 * - White noise (noise density)
 * - Slowly drifting bias (random walk)
 * - Gravity in body frame
 */
class ImuSimulatorNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new IMU Simulator Node
   */
  explicit ImuSimulatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Callback for truth odometry subscription
   */
  void truthCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Timer callback to publish IMU data at fixed rate
   */
  void publishCallback();

  /**
   * @brief Update bias values with random walk
   * @param dt Time step in seconds
   */
  void updateBias(double dt);

  /**
   * @brief Compute linear acceleration from velocity change
   * @param dt Time step in seconds
   * @return True acceleration in body frame
   */
  Eigen::Vector3d computeAcceleration(double dt);

  /**
   * @brief Add gravity to acceleration measurement in body frame
   * @param accel_body Body-frame acceleration (without gravity)
   * @return Specific force (what accelerometer measures)
   */
  Eigen::Vector3d addGravity(const Eigen::Vector3d & accel_body) const;

  /**
   * @brief Generate white noise sample
   * @param std_dev Standard deviation of noise
   * @return 3D noise vector
   */
  Eigen::Vector3d generateNoise(double std_dev);

  // Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // IMU noise parameters
  double gyro_noise_density_;       // rad/s/sqrt(Hz)
  double gyro_bias_instability_;    // rad/s
  double accel_noise_density_;      // m/s^2/sqrt(Hz)
  double accel_bias_instability_;   // m/s^2
  double publish_rate_;             // Hz

  // Bias state (slowly drifting)
  Eigen::Vector3d gyro_bias_;       // rad/s
  Eigen::Vector3d accel_bias_;      // m/s^2

  // Latest truth state
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d angular_velocity_body_;    // From truth twist
  Eigen::Vector3d linear_velocity_world_;    // World-frame velocity
  Eigen::Vector3d prev_linear_velocity_world_;  // For acceleration computation
  bool have_truth_;
  rclcpp::Time last_truth_time_;

  // Random number generation
  std::mt19937 rng_;
  std::normal_distribution<double> normal_dist_;

  // Timing for bias logging
  rclcpp::Time last_bias_log_time_;
  double bias_log_interval_;  // seconds

  // Gravity constant (NED: +Z is down, so gravity is +9.81 in Z)
  static constexpr double GRAVITY = 9.81;
};

}  // namespace usbl_navigation

#endif  // USBL_NAVIGATION__IMU_SIMULATOR_NODE_HPP_
