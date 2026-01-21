// Copyright 2026 Vinay
// SPDX-License-Identifier: MIT

#ifndef USBL_NAVIGATION__DELAYED_STATE_EKF_HPP_
#define USBL_NAVIGATION__DELAYED_STATE_EKF_HPP_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <deque>

namespace usbl_navigation {

/**
 * @brief State indices for 16-element state vector
 *
 * Note: Quaternion has 4 components but 3 DOF (unit constraint).
 * STATE_SIZE = 16 for Eigen storage, but conceptually 15 independent states.
 */
enum StateIdx {
  PX = 0, PY = 1, PZ = 2,           // Position (NED frame)
  VX = 3, VY = 4, VZ = 5,           // Velocity (NED frame)
  QW = 6, QX = 7, QY = 8, QZ = 9,   // Quaternion (w, x, y, z)
  BGX = 10, BGY = 11, BGZ = 12,     // Gyro bias (rad/s)
  BAX = 13, BAY = 14, BAZ = 15      // Accel bias (m/s^2)
};

/// State vector dimension (16 elements: 15 states + quaternion normalization)
constexpr int STATE_SIZE = 16;

/**
 * @brief IMU measurement with timestamp
 */
struct ImuMeasurement {
  rclcpp::Time timestamp;
  Eigen::Vector3d gyro;   ///< Angular velocity (rad/s, raw - not bias-corrected)
  Eigen::Vector3d accel;  ///< Linear acceleration (m/s^2, raw - includes gravity)
};

/**
 * @brief Buffered state for delayed measurement updates
 */
struct BufferedState {
  rclcpp::Time timestamp;
  Eigen::Matrix<double, STATE_SIZE, 1> state;
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> covariance;
};

/**
 * @brief Delayed-State Extended Kalman Filter for AUV navigation
 *
 * This EKF supports delayed measurement updates, enabling proper handling of
 * USBL position fixes that arrive with 0.2-0.5s latency. The filter maintains
 * a buffer of historical states that can be corrected and repropagated.
 *
 * State vector (16 elements, 15 DOF):
 * - Position [0-2]: x, y, z in NED frame (meters)
 * - Velocity [3-5]: vx, vy, vz in NED frame (m/s)
 * - Quaternion [6-9]: w, x, y, z (body-to-NED rotation)
 * - Gyro bias [10-12]: bias_gx, bias_gy, bias_gz (rad/s)
 * - Accel bias [13-15]: bias_ax, bias_ay, bias_az (m/s^2)
 *
 * Coordinate convention: NED (North-East-Down)
 * - +X points North
 * - +Y points East
 * - +Z points Down
 * - Gravity vector: [0, 0, +9.81] m/s^2
 */
class DelayedStateEKF {
public:
  /**
   * @brief Constructor with default parameters
   */
  DelayedStateEKF();

  /**
   * @brief Predict state forward using IMU measurement
   * @param imu IMU measurement (raw gyro and accel)
   * @param dt Time step in seconds
   */
  void predict(const ImuMeasurement& imu, double dt);

  /**
   * @brief Update state with DVL velocity measurement
   * @param velocity_body Body-frame velocity from DVL (m/s)
   * @param R Measurement noise covariance (3x3)
   */
  void updateDvl(const Eigen::Vector3d& velocity_body, const Eigen::Matrix3d& R);

  /**
   * @brief Update state with delayed USBL position measurement
   * @param position NED position from USBL (meters)
   * @param meas_time Timestamp when measurement was actually taken
   * @param R Measurement noise covariance (3x3)
   * @return true if update was applied, false if measurement was rejected
   */
  bool updateUsblDelayed(const Eigen::Vector3d& position,
                         const rclcpp::Time& meas_time,
                         const Eigen::Matrix3d& R);

  /**
   * @brief Get current state vector
   * @return 16x1 state vector
   */
  Eigen::Matrix<double, STATE_SIZE, 1> getState() const;

  /**
   * @brief Get current covariance matrix
   * @return 16x16 covariance matrix
   */
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> getCovariance() const;

  /**
   * @brief Get current position in NED frame
   * @return Position vector [x, y, z]
   */
  Eigen::Vector3d getPosition() const;

  /**
   * @brief Get current velocity in NED frame
   * @return Velocity vector [vx, vy, vz]
   */
  Eigen::Vector3d getVelocity() const;

  /**
   * @brief Get current orientation as quaternion
   * @return Quaternion (w, x, y, z)
   */
  Eigen::Quaterniond getQuaternion() const;

  /**
   * @brief Set process noise parameters
   * @param position_noise Position process noise (m/s^2)
   * @param velocity_noise Velocity process noise (m/s^3)
   * @param orientation_noise Orientation process noise (rad/s^2)
   * @param gyro_bias_noise Gyro bias random walk (rad/s^2)
   * @param accel_bias_noise Accel bias random walk (m/s^3)
   */
  void setProcessNoise(double position_noise, double velocity_noise,
                       double orientation_noise, double gyro_bias_noise,
                       double accel_bias_noise);

  /**
   * @brief Set Mahalanobis distance threshold for outlier rejection
   * @param threshold Chi-squared threshold (default: 9.21 for 3DOF, 99%)
   */
  void setMahalanobisThreshold(double threshold);

  /**
   * @brief Set maximum buffer size for historical states
   * @param size Maximum number of states to buffer
   */
  void setBufferSize(size_t size);

  /**
   * @brief Reset filter to initial state
   */
  void reset();

  /**
   * @brief Check if filter has been initialized
   * @return true if filter has received at least one IMU measurement
   */
  bool isInitialized() const;

private:
  // State and covariance
  Eigen::Matrix<double, STATE_SIZE, 1> state_;
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P_;
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q_;

  // Buffers for delayed updates
  std::deque<BufferedState> state_buffer_;
  std::deque<ImuMeasurement> imu_buffer_;

  // Configuration
  size_t buffer_size_;
  double mahalanobis_threshold_;
  bool initialized_;

  // Process noise components (stored for Q matrix construction)
  double process_noise_position_;
  double process_noise_velocity_;
  double process_noise_orientation_;
  double process_noise_gyro_bias_;
  double process_noise_accel_bias_;

  /**
   * @brief Normalize quaternion in state vector to unit length
   */
  void normalizeQuaternion();

  /**
   * @brief Store current state in buffer for delayed updates
   * @param time Timestamp of the state
   */
  void storeState(const rclcpp::Time& time);

  /**
   * @brief Find buffered state closest to requested time
   * @param time Requested timestamp
   * @param out Output buffered state
   * @return true if suitable state found, false otherwise
   */
  bool findBufferedState(const rclcpp::Time& time, BufferedState& out);

  /**
   * @brief Repropagate state from historical time to current time
   * @param from_time Starting time for repropagation
   * @param corrected_state Corrected state at measurement time
   * @param corrected_cov Corrected covariance at measurement time
   */
  void repropagateFrom(const rclcpp::Time& from_time,
                       const Eigen::Matrix<double, STATE_SIZE, 1>& corrected_state,
                       const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>& corrected_cov);

  /**
   * @brief Repropagate state from historical time to current time (uses current state)
   * @param from_time Starting time for repropagation
   */
  void repropagateFrom(const rclcpp::Time& from_time);

  /**
   * @brief Core prediction step without IMU buffer update (for repropagation)
   * @param imu IMU measurement
   * @param dt Time step in seconds
   */
  void predictInternal(const ImuMeasurement& imu, double dt);

  /**
   * @brief Trim IMU buffer to remove old measurements
   */
  void trimImuBuffer();

  /**
   * @brief Trim state buffer to maximum size
   */
  void trimStateBuffer();

  /**
   * @brief Get rotation matrix from current quaternion state
   * @return 3x3 rotation matrix (body to NED)
   */
  Eigen::Matrix3d quaternionToRotationMatrix() const;

  /**
   * @brief Rotate vector from body frame to NED frame
   * @param v_body Vector in body frame
   * @return Vector in NED frame
   */
  Eigen::Vector3d rotateBodyToNed(const Eigen::Vector3d& v_body) const;

  /**
   * @brief Rotate vector from NED frame to body frame
   * @param v_ned Vector in NED frame
   * @return Vector in body frame
   */
  Eigen::Vector3d rotateNedToBody(const Eigen::Vector3d& v_ned) const;

  /**
   * @brief Compute state transition Jacobian for IMU prediction
   * @param gyro Bias-corrected gyro measurement
   * @param accel Bias-corrected accel measurement
   * @param q Current orientation quaternion
   * @param dt Time step
   * @return 16x16 Jacobian matrix
   */
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> computeStateTransitionJacobian(
      const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel,
      const Eigen::Quaterniond& q, double dt);

  /**
   * @brief Build process noise matrix Q for given dt
   * @param dt Time step
   * @return 16x16 process noise matrix
   */
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> buildProcessNoiseMatrix(double dt);

  /**
   * @brief Compute Mahalanobis distance for outlier detection
   * @param innovation Measurement residual
   * @param S Innovation covariance
   * @return Mahalanobis distance squared
   */
  double computeMahalanobisDistance(const Eigen::VectorXd& innovation,
                                    const Eigen::MatrixXd& S);
};

}  // namespace usbl_navigation

#endif  // USBL_NAVIGATION__DELAYED_STATE_EKF_HPP_
