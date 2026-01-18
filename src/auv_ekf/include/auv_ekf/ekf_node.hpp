// Copyright 2026 Vinay
// SPDX-License-Identifier: MIT

#ifndef AUV_EKF__EKF_NODE_HPP_
#define AUV_EKF__EKF_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>

namespace auv_ekf {

/**
 * @brief Extended Kalman Filter node for AUV state estimation
 *
 * State vector (12 states):
 * - Position: [x, y, z] in NED frame (meters)
 * - Velocity: [vx, vy, vz] body frame (m/s)
 * - Orientation: [roll, pitch, yaw] Euler angles (radians)
 * - Angular velocity: [wx, wy, wz] body frame (rad/s)
 *
 * Prediction model: Constant velocity with rotation
 * Prediction rate: 50Hz (faster than slowest sensor)
 */
class EkfNode : public rclcpp::Node {
public:
  EkfNode();

private:
  // State vector dimension: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
  static constexpr int STATE_SIZE = 12;

  // State indices for readability
  enum StateIdx {
    X = 0, Y = 1, Z = 2,           // Position in NED frame
    VX = 3, VY = 4, VZ = 5,        // Velocity in body frame
    ROLL = 6, PITCH = 7, YAW = 8,  // Orientation (Euler angles)
    WX = 9, WY = 10, WZ = 11       // Angular velocity in body frame
  };

  // EKF state variables
  Eigen::VectorXd state_;           // 12x1 state vector
  Eigen::MatrixXd covariance_;      // 12x12 covariance matrix P
  Eigen::MatrixXd process_noise_;   // 12x12 process noise matrix Q

  // Timing
  rclcpp::Time last_predict_time_;
  bool initialized_;

  // Timer for prediction step
  rclcpp::TimerBase::SharedPtr predict_timer_;

  // Publisher for state estimate
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;

  // Parameters
  double initial_covariance_position_;
  double initial_covariance_velocity_;
  double initial_covariance_orientation_;
  double initial_covariance_angular_vel_;
  double process_noise_position_;
  double process_noise_velocity_;
  double process_noise_orientation_;
  double process_noise_angular_vel_;
  double prediction_rate_;

  // Methods
  void initializeState();
  void predict();
  void publishState();

  /**
   * @brief Convert Euler angles to rotation matrix (ZYX convention)
   * @param roll Roll angle in radians
   * @param pitch Pitch angle in radians
   * @param yaw Yaw angle in radians
   * @return 3x3 rotation matrix from body to NED frame
   */
  Eigen::Matrix3d eulerToRotationMatrix(double roll, double pitch, double yaw);

  /**
   * @brief Normalize angle to [-pi, pi]
   * @param angle Angle in radians
   * @return Normalized angle
   */
  double normalizeAngle(double angle);
};

}  // namespace auv_ekf

#endif  // AUV_EKF__EKF_NODE_HPP_
