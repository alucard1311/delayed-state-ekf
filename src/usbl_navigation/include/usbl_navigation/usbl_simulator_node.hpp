#ifndef USBL_NAVIGATION__USBL_SIMULATOR_NODE_HPP_
#define USBL_NAVIGATION__USBL_SIMULATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Dense>

#include <deque>
#include <random>
#include <utility>

namespace usbl_navigation
{

/**
 * @brief USBL simulator node with delayed timestamps, noise, dropouts, and outliers
 *
 * Simulates realistic USBL acoustic positioning measurements with:
 * - Processing delay (measurement timestamp is in the past)
 * - Range-dependent Gaussian noise
 * - Random dropouts (10% probability)
 * - Random outlier injection (5% probability with 5m offset)
 *
 * Publishes:
 * - /usbl/position (geometry_msgs/PointStamped) at 0.2Hz with DELAYED timestamps
 * - /usbl/valid (std_msgs/Bool) indicating measurement validity
 */
class UsblSimulatorNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new USBL Simulator Node
   */
  explicit UsblSimulatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Callback for truth odometry subscriber
   * @param msg Odometry message from truth generator
   */
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Timer callback for publishing USBL measurements
   */
  void publishTimer();

  /**
   * @brief Get interpolated position at specified time from buffer
   * @param time Requested timestamp
   * @param position_out Output position vector
   * @return true if position found, false if time is outside buffer range
   */
  bool getPositionAtTime(const rclcpp::Time & time, Eigen::Vector3d & position_out);

  /**
   * @brief Compute range from ship to AUV
   * @param auv_position AUV position in world frame
   * @return Range in meters
   */
  double computeRange(const Eigen::Vector3d & auv_position) const;

  /**
   * @brief Add range-dependent noise to position
   * @param position Position to add noise to
   * @param range Range from ship to AUV
   * @return Noisy position
   */
  Eigen::Vector3d addNoise(const Eigen::Vector3d & position, double range);

  /**
   * @brief Inject random outlier offset
   * @param position Position to offset
   * @return Offset position
   */
  Eigen::Vector3d injectOutlier(const Eigen::Vector3d & position);

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Position buffer: (timestamp, position) pairs for delay lookup
  // Using deque as a ring buffer for efficient front/back operations
  std::deque<std::pair<rclcpp::Time, Eigen::Vector3d>> position_buffer_;
  static constexpr size_t MAX_BUFFER_SIZE = 100;

  // Ship position (fixed at surface)
  Eigen::Vector3d ship_position_;

  // Configuration parameters
  double range_noise_percent_;   // Noise as percentage of range (0.02 = 2%)
  double min_noise_;             // Minimum noise floor in meters
  double processing_delay_;      // USBL processing delay in seconds
  double dropout_probability_;   // Probability of measurement dropout
  double outlier_probability_;   // Probability of outlier injection
  double outlier_offset_;        // Outlier offset magnitude in meters
  double publish_rate_;          // Publishing rate in Hz

  // Random number generators
  std::mt19937 rng_;
  std::normal_distribution<double> noise_dist_;
  std::uniform_real_distribution<double> uniform_dist_;

  // Statistics for logging
  uint64_t total_measurements_;
  uint64_t dropout_count_;
  uint64_t outlier_count_;
};

}  // namespace usbl_navigation

#endif  // USBL_NAVIGATION__USBL_SIMULATOR_NODE_HPP_
