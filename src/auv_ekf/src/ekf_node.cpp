// Copyright 2026 Vinay
// SPDX-License-Identifier: MIT

#include "auv_ekf/ekf_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace auv_ekf {

// Constants for pressure-to-depth conversion
constexpr double ATMOSPHERIC_PRESSURE = 101325.0;  // Pa
constexpr double WATER_DENSITY = 1025.0;           // kg/m³ (seawater)
constexpr double GRAVITY = 9.81;                   // m/s²

// Sensor timeout for dead reckoning detection
constexpr double SENSOR_TIMEOUT = 1.0;  // seconds

EkfNode::EkfNode()
: Node("ekf_node"),
  state_(Eigen::VectorXd::Zero(STATE_SIZE)),
  covariance_(Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE)),
  process_noise_(Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE)),
  initialized_(false),
  R_imu_(Eigen::MatrixXd::Zero(6, 6)),
  R_pressure_(Eigen::MatrixXd::Zero(1, 1)),
  R_dvl_(Eigen::MatrixXd::Zero(3, 3)),
  dead_reckoning_mode_(false)
{
  // Declare parameters with defaults
  this->declare_parameter("initial_covariance.position", 1.0);
  this->declare_parameter("initial_covariance.velocity", 0.1);
  this->declare_parameter("initial_covariance.orientation", 0.1);
  this->declare_parameter("initial_covariance.angular_velocity", 0.01);
  this->declare_parameter("process_noise.position", 0.01);
  this->declare_parameter("process_noise.velocity", 0.1);
  this->declare_parameter("process_noise.orientation", 0.01);
  this->declare_parameter("process_noise.angular_velocity", 0.1);
  this->declare_parameter("prediction_rate", 50.0);

  // Get parameters
  initial_covariance_position_ = this->get_parameter("initial_covariance.position").as_double();
  initial_covariance_velocity_ = this->get_parameter("initial_covariance.velocity").as_double();
  initial_covariance_orientation_ = this->get_parameter("initial_covariance.orientation").as_double();
  initial_covariance_angular_vel_ = this->get_parameter("initial_covariance.angular_velocity").as_double();
  process_noise_position_ = this->get_parameter("process_noise.position").as_double();
  process_noise_velocity_ = this->get_parameter("process_noise.velocity").as_double();
  process_noise_orientation_ = this->get_parameter("process_noise.orientation").as_double();
  process_noise_angular_vel_ = this->get_parameter("process_noise.angular_velocity").as_double();
  prediction_rate_ = this->get_parameter("prediction_rate").as_double();

  // Initialize process noise matrix Q (diagonal)
  process_noise_.diagonal() <<
    process_noise_position_, process_noise_position_, process_noise_position_,
    process_noise_velocity_, process_noise_velocity_, process_noise_velocity_,
    process_noise_orientation_, process_noise_orientation_, process_noise_orientation_,
    process_noise_angular_vel_, process_noise_angular_vel_, process_noise_angular_vel_;

  // Initialize IMU measurement noise R_imu (6x6 diagonal)
  // States: [roll, pitch, yaw, wx, wy, wz]
  // Orientation noise ~0.01 rad, angular velocity noise ~0.01 rad/s
  R_imu_.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;

  // Initialize pressure measurement noise R_pressure (1x1)
  // Depth noise ~0.1m uncertainty
  R_pressure_(0, 0) = 0.1 * 0.1;  // Variance = (0.1m)^2

  // Initialize DVL measurement noise R_dvl (3x3 diagonal)
  // Velocity noise ~0.01 m/s per axis (typical for DVL)
  R_dvl_.diagonal() << 0.01, 0.01, 0.01;

  // Initialize sensor timing (will be updated on first callback)
  last_imu_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_pressure_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_dvl_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Create publisher for pose estimate
  pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/auv/ekf/pose", 10);

  // Create subscribers for sensor data
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/auv/imu", 10,
    std::bind(&EkfNode::imuCallback, this, std::placeholders::_1));

  pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
    "/auv/pressure", 10,
    std::bind(&EkfNode::pressureCallback, this, std::placeholders::_1));

  dvl_sub_ = this->create_subscription<stonefish_ros2::msg::DVL>(
    "/auv/dvl", 10,
    std::bind(&EkfNode::dvlCallback, this, std::placeholders::_1));

  // Create prediction timer
  auto period = std::chrono::duration<double>(1.0 / prediction_rate_);
  predict_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&EkfNode::predict, this));

  RCLCPP_INFO(this->get_logger(), "EKF node initialized with prediction rate: %.1f Hz", prediction_rate_);
}

void EkfNode::initializeState()
{
  // Initialize state to zeros (will be updated by first sensor readings)
  state_.setZero();

  // Initialize covariance matrix P (diagonal with initial uncertainties)
  covariance_.setZero();

  // Position uncertainty (x, y, z)
  covariance_(X, X) = initial_covariance_position_;
  covariance_(Y, Y) = initial_covariance_position_;
  covariance_(Z, Z) = initial_covariance_position_;

  // Velocity uncertainty (vx, vy, vz)
  covariance_(VX, VX) = initial_covariance_velocity_;
  covariance_(VY, VY) = initial_covariance_velocity_;
  covariance_(VZ, VZ) = initial_covariance_velocity_;

  // Orientation uncertainty (roll, pitch, yaw)
  covariance_(ROLL, ROLL) = initial_covariance_orientation_;
  covariance_(PITCH, PITCH) = initial_covariance_orientation_;
  covariance_(YAW, YAW) = initial_covariance_orientation_;

  // Angular velocity uncertainty (wx, wy, wz)
  covariance_(WX, WX) = initial_covariance_angular_vel_;
  covariance_(WY, WY) = initial_covariance_angular_vel_;
  covariance_(WZ, WZ) = initial_covariance_angular_vel_;

  last_predict_time_ = this->now();
  initialized_ = true;

  RCLCPP_INFO(this->get_logger(), "EKF state initialized");
}

void EkfNode::predict()
{
  // Initialize on first call
  if (!initialized_) {
    initializeState();
    return;
  }

  // Check sensor health and update dead reckoning mode
  checkSensorHealth();

  // Calculate time delta
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_predict_time_).seconds();

  // Guard against invalid dt (first call after init, clock jumps, etc.)
  if (dt <= 0.0 || dt > 1.0) {
    last_predict_time_ = current_time;
    return;
  }

  // Extract current state
  double x = state_(X);
  double y = state_(Y);
  double z = state_(Z);
  double vx = state_(VX);
  double vy = state_(VY);
  double vz = state_(VZ);
  double roll = state_(ROLL);
  double pitch = state_(PITCH);
  double yaw = state_(YAW);
  double wx = state_(WX);
  double wy = state_(WY);
  double wz = state_(WZ);

  // Get rotation matrix from body to NED frame
  Eigen::Matrix3d R = eulerToRotationMatrix(roll, pitch, yaw);

  // Body velocity vector
  Eigen::Vector3d v_body(vx, vy, vz);

  // Transform velocity to NED frame
  Eigen::Vector3d v_ned = R * v_body;

  // State prediction (constant velocity model)
  // Position updates (integrate velocity in NED frame)
  state_(X) = x + v_ned(0) * dt;
  state_(Y) = y + v_ned(1) * dt;
  state_(Z) = z + v_ned(2) * dt;

  // Velocity unchanged in constant velocity model
  // state_(VX), state_(VY), state_(VZ) remain the same

  // Orientation updates (integrate angular velocity)
  state_(ROLL) = normalizeAngle(roll + wx * dt);
  state_(PITCH) = normalizeAngle(pitch + wy * dt);
  state_(YAW) = normalizeAngle(yaw + wz * dt);

  // Angular velocity unchanged in constant velocity model
  // state_(WX), state_(WY), state_(WZ) remain the same

  // Build state transition Jacobian F (12x12)
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

  // Position depends on velocity (through rotation)
  // dPos/dVel = R * dt (simplified - full Jacobian would include rotation derivatives)
  F.block<3, 3>(X, VX) = R * dt;

  // Position depends on orientation (through rotation of velocity)
  // Simplified: we use numerical approximation or ignore for now
  // Full implementation would compute dR/d(roll,pitch,yaw)

  // Orientation depends on angular velocity
  F(ROLL, WX) = dt;
  F(PITCH, WY) = dt;
  F(YAW, WZ) = dt;

  // Covariance prediction: P = F * P * F^T + Q
  // Increase process noise during dead reckoning to reflect increased uncertainty
  double noise_multiplier = dead_reckoning_mode_ ? 2.0 : 1.0;
  covariance_ = F * covariance_ * F.transpose() + process_noise_ * dt * noise_multiplier;

  // Update time
  last_predict_time_ = current_time;

  // Publish state estimate
  publishState();
}

void EkfNode::publishState()
{
  auto msg = nav_msgs::msg::Odometry();

  // Header
  msg.header.stamp = this->now();
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  // Position
  msg.pose.pose.position.x = state_(X);
  msg.pose.pose.position.y = state_(Y);
  msg.pose.pose.position.z = state_(Z);

  // Orientation (convert Euler to quaternion)
  tf2::Quaternion q;
  q.setRPY(state_(ROLL), state_(PITCH), state_(YAW));
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  // Pose covariance (6x6 subset: x, y, z, roll, pitch, yaw)
  // Row-major order for ROS message
  msg.pose.covariance[0] = covariance_(X, X);      // x-x
  msg.pose.covariance[7] = covariance_(Y, Y);      // y-y
  msg.pose.covariance[14] = covariance_(Z, Z);     // z-z
  msg.pose.covariance[21] = covariance_(ROLL, ROLL);   // roll-roll
  msg.pose.covariance[28] = covariance_(PITCH, PITCH); // pitch-pitch
  msg.pose.covariance[35] = covariance_(YAW, YAW);     // yaw-yaw

  // Linear velocity (body frame)
  msg.twist.twist.linear.x = state_(VX);
  msg.twist.twist.linear.y = state_(VY);
  msg.twist.twist.linear.z = state_(VZ);

  // Angular velocity (body frame)
  msg.twist.twist.angular.x = state_(WX);
  msg.twist.twist.angular.y = state_(WY);
  msg.twist.twist.angular.z = state_(WZ);

  // Twist covariance
  msg.twist.covariance[0] = covariance_(VX, VX);
  msg.twist.covariance[7] = covariance_(VY, VY);
  msg.twist.covariance[14] = covariance_(VZ, VZ);
  msg.twist.covariance[21] = covariance_(WX, WX);
  msg.twist.covariance[28] = covariance_(WY, WY);
  msg.twist.covariance[35] = covariance_(WZ, WZ);

  pose_pub_->publish(msg);
}

Eigen::Matrix3d EkfNode::eulerToRotationMatrix(double roll, double pitch, double yaw)
{
  // ZYX (yaw-pitch-roll) rotation matrix
  // This transforms from body frame to NED frame
  double cr = std::cos(roll);
  double sr = std::sin(roll);
  double cp = std::cos(pitch);
  double sp = std::sin(pitch);
  double cy = std::cos(yaw);
  double sy = std::sin(yaw);

  Eigen::Matrix3d R;
  R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
       sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
       -sp,     cp * sr,                cp * cr;

  return R;
}

double EkfNode::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

void EkfNode::quaternionToEuler(double qx, double qy, double qz, double qw,
                                 double& roll, double& pitch, double& yaw)
{
  // Convert quaternion to Euler angles (ZYX convention)
  // Roll (x-axis rotation)
  double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2.0 * (qw * qy - qz * qx);
  if (std::abs(sinp) >= 1.0) {
    // Use 90 degrees if out of range
    pitch = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  // Yaw (z-axis rotation)
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

void EkfNode::checkSensorHealth()
{
  auto now = this->now();

  // Check if each sensor has timed out
  // Note: A time of 0 means no measurement received yet, treat as not timed out
  // to avoid spurious warnings on startup
  bool imu_ok = (last_imu_time_.nanoseconds() == 0) ||
                (now - last_imu_time_).seconds() < SENSOR_TIMEOUT;
  bool pressure_ok = (last_pressure_time_.nanoseconds() == 0) ||
                     (now - last_pressure_time_).seconds() < SENSOR_TIMEOUT;
  bool dvl_ok = (last_dvl_time_.nanoseconds() == 0) ||
                (now - last_dvl_time_).seconds() < SENSOR_TIMEOUT;

  // Track previous state for logging transitions
  bool was_dead_reckoning = dead_reckoning_mode_;

  // Enter dead reckoning if any sensor has timed out (after receiving at least one message)
  bool sensors_received = last_imu_time_.nanoseconds() > 0 ||
                          last_pressure_time_.nanoseconds() > 0 ||
                          last_dvl_time_.nanoseconds() > 0;
  dead_reckoning_mode_ = sensors_received && (!imu_ok || !pressure_ok || !dvl_ok);

  // Log state transitions
  if (dead_reckoning_mode_ && !was_dead_reckoning) {
    RCLCPP_WARN(this->get_logger(),
                "Entering dead reckoning mode - sensors: IMU=%s, Pressure=%s, DVL=%s",
                imu_ok ? "OK" : "TIMEOUT",
                pressure_ok ? "OK" : "TIMEOUT",
                dvl_ok ? "OK" : "TIMEOUT");
  } else if (!dead_reckoning_mode_ && was_dead_reckoning) {
    RCLCPP_INFO(this->get_logger(), "Exiting dead reckoning mode - all sensors OK");
  }
}

void EkfNode::measurementUpdate(const Eigen::VectorXd& z, const Eigen::MatrixXd& H,
                                 const Eigen::MatrixXd& R)
{
  // Skip if not initialized
  if (!initialized_) {
    return;
  }

  // Innovation (measurement residual): y = z - H * x
  Eigen::VectorXd y = z - H * state_;

  // Wrap angle innovations for orientation states (indices 0-2 in measurement if present)
  // Check which states H is selecting by looking at which columns are non-zero
  for (int i = 0; i < y.size(); ++i) {
    // Find the state index this measurement corresponds to
    for (int j = 0; j < STATE_SIZE; ++j) {
      if (std::abs(H(i, j)) > 0.5) {  // Non-zero element indicates state mapping
        // If it's an orientation state (ROLL=6, PITCH=7, YAW=8), wrap the innovation
        if (j >= ROLL && j <= YAW) {
          y(i) = normalizeAngle(y(i));
        }
        break;
      }
    }
  }

  // Innovation covariance: S = H * P * H^T + R
  Eigen::MatrixXd S = H * covariance_ * H.transpose() + R;

  // Kalman gain: K = P * H^T * S^-1
  Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();

  // State update: x = x + K * y
  state_ = state_ + K * y;

  // Wrap orientation angles to [-pi, pi]
  state_(ROLL) = normalizeAngle(state_(ROLL));
  state_(PITCH) = normalizeAngle(state_(PITCH));
  state_(YAW) = normalizeAngle(state_(YAW));

  // Covariance update: P = (I - K * H) * P
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  covariance_ = (I - K * H) * covariance_;
}

void EkfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Skip if not initialized
  if (!initialized_) {
    return;
  }

  // Update IMU timing for dead reckoning detection
  last_imu_time_ = this->now();

  // Extract orientation quaternion and convert to Euler
  double roll, pitch, yaw;
  quaternionToEuler(msg->orientation.x, msg->orientation.y,
                    msg->orientation.z, msg->orientation.w,
                    roll, pitch, yaw);

  // Extract angular velocity
  double wx = msg->angular_velocity.x;
  double wy = msg->angular_velocity.y;
  double wz = msg->angular_velocity.z;

  // Create measurement vector z (6x1): [roll, pitch, yaw, wx, wy, wz]
  Eigen::VectorXd z(6);
  z << roll, pitch, yaw, wx, wy, wz;

  // Create measurement matrix H (6x12): maps state to measurement
  // H selects states [ROLL, PITCH, YAW, WX, WY, WZ]
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, STATE_SIZE);
  H(0, ROLL) = 1.0;   // Measurement 0 is roll
  H(1, PITCH) = 1.0;  // Measurement 1 is pitch
  H(2, YAW) = 1.0;    // Measurement 2 is yaw
  H(3, WX) = 1.0;     // Measurement 3 is angular velocity x
  H(4, WY) = 1.0;     // Measurement 4 is angular velocity y
  H(5, WZ) = 1.0;     // Measurement 5 is angular velocity z

  // Perform measurement update
  measurementUpdate(z, H, R_imu_);
}

void EkfNode::pressureCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
{
  // Skip if not initialized
  if (!initialized_) {
    return;
  }

  // Update pressure timing for dead reckoning detection
  last_pressure_time_ = this->now();

  // Calculate depth from pressure
  // depth = (pressure - atmospheric_pressure) / (water_density * gravity)
  // In NED frame, positive Z is DOWN, so depth is positive underwater
  double pressure = msg->fluid_pressure;
  double depth = (pressure - ATMOSPHERIC_PRESSURE) / (WATER_DENSITY * GRAVITY);

  // Create measurement vector z (1x1): [depth]
  Eigen::VectorXd z(1);
  z << depth;

  // Create measurement matrix H (1x12): H(0, Z) = 1.0, rest zeros
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_SIZE);
  H(0, Z) = 1.0;  // Measurement is depth (Z in NED frame)

  // Perform measurement update
  measurementUpdate(z, H, R_pressure_);
}

void EkfNode::dvlCallback(const stonefish_ros2::msg::DVL::SharedPtr msg)
{
  // Skip if not initialized
  if (!initialized_) {
    return;
  }

  // Update DVL timing for dead reckoning detection
  last_dvl_time_ = this->now();

  // Extract velocity from DVL message (body frame)
  double vx = msg->velocity.x;
  double vy = msg->velocity.y;
  double vz = msg->velocity.z;

  // Create measurement vector z (3x1): [vx, vy, vz]
  Eigen::VectorXd z(3);
  z << vx, vy, vz;

  // Create measurement matrix H (3x12): maps state velocity to measurement
  // H selects states [VX, VY, VZ]
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, STATE_SIZE);
  H(0, VX) = 1.0;  // Measurement 0 is velocity x
  H(1, VY) = 1.0;  // Measurement 1 is velocity y
  H(2, VZ) = 1.0;  // Measurement 2 is velocity z

  // Perform measurement update
  measurementUpdate(z, H, R_dvl_);
}

}  // namespace auv_ekf
