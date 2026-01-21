#ifndef USBL_NAVIGATION__DVL_SIMULATOR_NODE_HPP_
#define USBL_NAVIGATION__DVL_SIMULATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <random>

namespace usbl_navigation
{

/**
 * @brief DVL simulator node with body-frame conversion and dropout simulation
 *
 * Subscribes to truth odometry and publishes DVL velocity measurements at 5Hz.
 * Simulates:
 * - World-to-body frame velocity transformation
 * - Velocity noise and scale factor error
 * - Canyon dropout scenario (t=120s-150s)
 * - Random dropouts based on probability
 */
class DvlSimulatorNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new DVL Simulator Node
   */
  explicit DvlSimulatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Callback for truth odometry subscription
   */
  void truthCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Timer callback to publish DVL data at fixed rate
   */
  void publishCallback();

  /**
   * @brief Check if DVL has bottom lock
   * @return true if bottom lock is acquired
   */
  bool checkBottomLock();

  /**
   * @brief Transform velocity from world to body frame
   * @param v_world Velocity in world (NED) frame
   * @return Velocity in body frame
   */
  Eigen::Vector3d transformToBodyFrame(const Eigen::Vector3d & v_world) const;

  /**
   * @brief Add noise and scale factor error to velocity
   * @param v_body Clean body-frame velocity
   * @return Noisy velocity measurement
   */
  Eigen::Vector3d addNoise(const Eigen::Vector3d & v_body);

  // Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bottom_lock_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // DVL noise parameters
  double velocity_noise_std_;      // m/s
  double scale_factor_error_;      // fraction (e.g., 0.01 = 1%)
  double dropout_probability_;     // probability of random dropout
  double canyon_dropout_start_;    // seconds
  double canyon_dropout_end_;      // seconds
  double publish_rate_;            // Hz
  bool enable_canyon_dropout_;     // Enable/disable canyon scenario

  // Latest truth state
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d linear_velocity_world_;  // World-frame velocity from truth
  bool have_truth_;

  // Mission timing
  rclcpp::Time mission_start_time_;
  bool mission_started_;

  // Random number generation
  std::mt19937 rng_;
  std::normal_distribution<double> normal_dist_;
  std::uniform_real_distribution<double> uniform_dist_;
};

}  // namespace usbl_navigation

#endif  // USBL_NAVIGATION__DVL_SIMULATOR_NODE_HPP_
