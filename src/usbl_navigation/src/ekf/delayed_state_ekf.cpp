// Copyright 2026 Vinay
// SPDX-License-Identifier: MIT

#include "usbl_navigation/delayed_state_ekf.hpp"
#include <cmath>
#include <algorithm>

namespace usbl_navigation {

// Constants
constexpr double GRAVITY = 9.81;  // m/s^2 (positive in NED +Z direction)

DelayedStateEKF::DelayedStateEKF()
    : state_(Eigen::Matrix<double, STATE_SIZE, 1>::Zero()),
      P_(Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Zero()),
      Q_(Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Zero()),
      buffer_size_(500),
      mahalanobis_threshold_(9.21),  // Chi-squared 3DOF, 99% confidence
      initialized_(false),
      process_noise_position_(0.01),
      process_noise_velocity_(0.1),
      process_noise_orientation_(0.001),
      process_noise_gyro_bias_(0.0001),
      process_noise_accel_bias_(0.001) {
  // Initialize quaternion to identity (w=1, x=y=z=0)
  state_(QW) = 1.0;

  // Initialize covariance with reasonable initial uncertainties
  // Position uncertainty: 1.0 m
  P_(PX, PX) = 1.0;
  P_(PY, PY) = 1.0;
  P_(PZ, PZ) = 1.0;

  // Velocity uncertainty: 0.5 m/s
  P_(VX, VX) = 0.25;
  P_(VY, VY) = 0.25;
  P_(VZ, VZ) = 0.25;

  // Quaternion uncertainty: small (0.01 rad equivalent)
  P_(QW, QW) = 0.0001;
  P_(QX, QX) = 0.0001;
  P_(QY, QY) = 0.0001;
  P_(QZ, QZ) = 0.0001;

  // Gyro bias uncertainty: 0.01 rad/s
  P_(BGX, BGX) = 0.0001;
  P_(BGY, BGY) = 0.0001;
  P_(BGZ, BGZ) = 0.0001;

  // Accel bias uncertainty: 0.1 m/s^2
  P_(BAX, BAX) = 0.01;
  P_(BAY, BAY) = 0.01;
  P_(BAZ, BAZ) = 0.01;
}

void DelayedStateEKF::predict(const ImuMeasurement& imu, double dt) {
  // Guard against invalid time steps
  if (dt <= 0.0 || dt > 1.0) {
    return;
  }

  // Store IMU measurement for potential repropagation
  imu_buffer_.push_back(imu);
  trimImuBuffer();

  // Mark as initialized after first prediction
  if (!initialized_) {
    initialized_ = true;
  }

  // Bias-corrected measurements
  Eigen::Vector3d gyro = imu.gyro - state_.segment<3>(BGX);
  Eigen::Vector3d accel = imu.accel - state_.segment<3>(BAX);

  // Extract current quaternion
  Eigen::Quaterniond q(state_(QW), state_(QX), state_(QY), state_(QZ));
  q.normalize();  // Ensure normalized before use

  // Orientation update (first-order quaternion integration)
  double angle = gyro.norm() * dt;
  Eigen::Quaterniond dq;
  if (angle > 1e-10) {
    Eigen::Vector3d axis = gyro.normalized();
    dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
  } else {
    dq = Eigen::Quaterniond::Identity();
  }

  // Quaternion update: q_new = q * dq (body frame rotation)
  q = q * dq;
  q.normalize();

  // Rotate acceleration to NED frame and remove gravity
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Vector3d accel_ned = R * accel;

  // Gravity vector in NED frame (points down, +Z)
  Eigen::Vector3d gravity(0.0, 0.0, GRAVITY);

  // Corrected acceleration (remove gravity)
  Eigen::Vector3d accel_corrected = accel_ned - gravity;

  // Velocity update (NED frame)
  state_.segment<3>(VX) += accel_corrected * dt;

  // Position update (NED frame)
  state_.segment<3>(PX) += state_.segment<3>(VX) * dt;

  // Update quaternion in state
  state_(QW) = q.w();
  state_(QX) = q.x();
  state_(QY) = q.y();
  state_(QZ) = q.z();

  // Bias states remain unchanged (random walk modeled in Q)

  // Covariance prediction: P = F * P * F^T + Q
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F =
      computeStateTransitionJacobian(gyro, accel, q, dt);
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q_dt = buildProcessNoiseMatrix(dt);

  P_ = F * P_ * F.transpose() + Q_dt;

  // Ensure quaternion normalization in state
  normalizeQuaternion();

  // Store state in buffer for delayed updates
  storeState(imu.timestamp);
}

void DelayedStateEKF::updateDvl(const Eigen::Vector3d& velocity_body,
                                const Eigen::Matrix3d& R) {
  if (!initialized_) {
    return;
  }

  // DVL measures body-frame velocity, state stores NED-frame velocity
  // Transform state velocity to body frame for comparison
  Eigen::Vector3d velocity_ned = state_.segment<3>(VX);
  Eigen::Vector3d predicted_body = rotateNedToBody(velocity_ned);

  // Innovation (measurement - predicted)
  Eigen::Vector3d innovation = velocity_body - predicted_body;

  // Measurement matrix H: maps state velocity (NED) to body velocity
  // H * x = R_ned_to_body * velocity_ned
  Eigen::Matrix3d R_body_to_ned = quaternionToRotationMatrix();
  Eigen::Matrix3d R_ned_to_body = R_body_to_ned.transpose();

  Eigen::Matrix<double, 3, STATE_SIZE> H = Eigen::Matrix<double, 3, STATE_SIZE>::Zero();
  H.block<3, 3>(0, VX) = R_ned_to_body;

  // Innovation covariance: S = H * P * H^T + R
  Eigen::Matrix3d S = H * P_ * H.transpose() + R;

  // Outlier rejection using Mahalanobis distance
  double mahal_dist = computeMahalanobisDistance(innovation, S);
  if (mahal_dist > mahalanobis_threshold_) {
    return;  // Reject outlier
  }

  // Kalman gain: K = P * H^T * S^-1
  Eigen::Matrix<double, STATE_SIZE, 3> K = P_ * H.transpose() * S.inverse();

  // State update: x = x + K * y
  state_ += K * innovation;

  // Covariance update (Joseph form for numerical stability)
  // P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> I =
      Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> IKH = I - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();

  // Ensure quaternion normalization
  normalizeQuaternion();
}

bool DelayedStateEKF::updateUsblDelayed(const Eigen::Vector3d& position,
                                        const rclcpp::Time& meas_time,
                                        const Eigen::Matrix3d& R) {
  if (!initialized_) {
    return false;
  }

  // Step 1: Find buffered state at measurement time
  BufferedState buffered;
  if (!findBufferedState(meas_time, buffered)) {
    // Measurement too old (not in buffer) - reject
    RCLCPP_WARN(rclcpp::get_logger("DelayedStateEKF"),
                "USBL measurement too old, rejecting");
    return false;
  }

  // Step 2: Compute innovation using buffered state
  Eigen::Vector3d z = position;
  Eigen::Vector3d z_pred = buffered.state.segment<3>(PX);
  Eigen::Vector3d y = z - z_pred;

  // Step 3: Measurement matrix (position observation)
  Eigen::Matrix<double, 3, STATE_SIZE> H = Eigen::Matrix<double, 3, STATE_SIZE>::Zero();
  H(0, PX) = 1.0;
  H(1, PY) = 1.0;
  H(2, PZ) = 1.0;

  // Step 4: Innovation covariance
  Eigen::Matrix3d S = H * buffered.covariance * H.transpose() + R;

  // Step 5: Mahalanobis distance for outlier rejection
  double mahal_dist_sq = computeMahalanobisDistance(y, S);
  if (mahal_dist_sq > mahalanobis_threshold_) {
    // Outlier detected - reject measurement
    RCLCPP_WARN(rclcpp::get_logger("DelayedStateEKF"),
                "USBL outlier rejected: Mahalanobis^2=%.1f > threshold=%.1f",
                mahal_dist_sq, mahalanobis_threshold_);
    return false;
  }

  // Step 6: Kalman gain using buffered covariance
  Eigen::Matrix<double, STATE_SIZE, 3> K =
      buffered.covariance * H.transpose() * S.inverse();

  // Step 7: Update buffered state (at measurement time)
  Eigen::Matrix<double, STATE_SIZE, 1> state_corrected = buffered.state + K * y;

  // Normalize quaternion in corrected state
  Eigen::Quaterniond q_corr(state_corrected(QW), state_corrected(QX),
                            state_corrected(QY), state_corrected(QZ));
  q_corr.normalize();
  state_corrected(QW) = q_corr.w();
  state_corrected(QX) = q_corr.x();
  state_corrected(QY) = q_corr.y();
  state_corrected(QZ) = q_corr.z();

  // Update covariance at measurement time (Joseph form for numerical stability)
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> I =
      Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> IKH = I - K * H;
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P_corrected =
      IKH * buffered.covariance * IKH.transpose() + K * R * K.transpose();

  // Step 8: Repropagate from corrected state to current time
  repropagateFrom(meas_time, state_corrected, P_corrected);

  RCLCPP_DEBUG(rclcpp::get_logger("DelayedStateEKF"),
               "USBL update applied: innovation=[%.2f, %.2f, %.2f]m",
               y(0), y(1), y(2));

  return true;
}

Eigen::Matrix<double, STATE_SIZE, 1> DelayedStateEKF::getState() const {
  return state_;
}

Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> DelayedStateEKF::getCovariance() const {
  return P_;
}

Eigen::Vector3d DelayedStateEKF::getPosition() const {
  return state_.segment<3>(PX);
}

Eigen::Vector3d DelayedStateEKF::getVelocity() const {
  return state_.segment<3>(VX);
}

Eigen::Quaterniond DelayedStateEKF::getQuaternion() const {
  return Eigen::Quaterniond(state_(QW), state_(QX), state_(QY), state_(QZ));
}

void DelayedStateEKF::setProcessNoise(double position_noise, double velocity_noise,
                                      double orientation_noise, double gyro_bias_noise,
                                      double accel_bias_noise) {
  process_noise_position_ = position_noise;
  process_noise_velocity_ = velocity_noise;
  process_noise_orientation_ = orientation_noise;
  process_noise_gyro_bias_ = gyro_bias_noise;
  process_noise_accel_bias_ = accel_bias_noise;
}

void DelayedStateEKF::setMahalanobisThreshold(double threshold) {
  mahalanobis_threshold_ = threshold;
}

void DelayedStateEKF::setBufferSize(size_t size) {
  buffer_size_ = size;
}

void DelayedStateEKF::reset() {
  state_.setZero();
  state_(QW) = 1.0;  // Identity quaternion

  P_.setZero();
  P_(PX, PX) = 1.0;
  P_(PY, PY) = 1.0;
  P_(PZ, PZ) = 1.0;
  P_(VX, VX) = 0.25;
  P_(VY, VY) = 0.25;
  P_(VZ, VZ) = 0.25;
  P_(QW, QW) = 0.0001;
  P_(QX, QX) = 0.0001;
  P_(QY, QY) = 0.0001;
  P_(QZ, QZ) = 0.0001;
  P_(BGX, BGX) = 0.0001;
  P_(BGY, BGY) = 0.0001;
  P_(BGZ, BGZ) = 0.0001;
  P_(BAX, BAX) = 0.01;
  P_(BAY, BAY) = 0.01;
  P_(BAZ, BAZ) = 0.01;

  state_buffer_.clear();
  imu_buffer_.clear();
  initialized_ = false;
}

bool DelayedStateEKF::isInitialized() const {
  return initialized_;
}

void DelayedStateEKF::normalizeQuaternion() {
  Eigen::Quaterniond q(state_(QW), state_(QX), state_(QY), state_(QZ));
  q.normalize();
  state_(QW) = q.w();
  state_(QX) = q.x();
  state_(QY) = q.y();
  state_(QZ) = q.z();
}

void DelayedStateEKF::storeState(const rclcpp::Time& time) {
  BufferedState buffered;
  buffered.timestamp = time;
  buffered.state = state_;
  buffered.covariance = P_;

  state_buffer_.push_back(buffered);
  trimStateBuffer();
}

bool DelayedStateEKF::findBufferedState(const rclcpp::Time& time, BufferedState& out) {
  if (state_buffer_.empty()) {
    return false;
  }

  // Find state with timestamp closest to requested time
  // States are stored in chronological order
  double min_diff = std::numeric_limits<double>::max();
  bool found = false;

  for (const auto& state : state_buffer_) {
    double diff = std::abs((state.timestamp - time).seconds());
    if (diff < min_diff) {
      min_diff = diff;
      out = state;
      found = true;
    }
  }

  // Reject if time difference is too large (> 1 second)
  if (found && min_diff > 1.0) {
    return false;
  }

  return found;
}

void DelayedStateEKF::repropagateFrom(
    const rclcpp::Time& from_time,
    const Eigen::Matrix<double, STATE_SIZE, 1>& corrected_state,
    const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>& corrected_cov) {
  // Set state to corrected values
  state_ = corrected_state;
  P_ = corrected_cov;

  // Clear state buffer entries after correction time (they're now invalid)
  while (!state_buffer_.empty() && state_buffer_.back().timestamp > from_time) {
    state_buffer_.pop_back();
  }

  // Replay IMU measurements from correction time to current time
  rclcpp::Time prev_time = from_time;
  for (const auto& imu : imu_buffer_) {
    if (imu.timestamp <= from_time) {
      continue;  // Skip IMU measurements before correction time
    }

    // Calculate dt from previous measurement time
    double dt = (imu.timestamp - prev_time).seconds();
    if (dt <= 0 || dt >= 1.0) {
      continue;  // Skip invalid time steps
    }

    // Run prediction and rebuild state buffer
    predictInternal(imu, dt);
    storeState(imu.timestamp);

    prev_time = imu.timestamp;
  }
}

void DelayedStateEKF::repropagateFrom(const rclcpp::Time& from_time) {
  // Overload for backward compatibility - uses current state/covariance
  // This version is used when state_ and P_ are already set to corrected values

  // Clear state buffer entries after correction time (they're now invalid)
  while (!state_buffer_.empty() && state_buffer_.back().timestamp > from_time) {
    state_buffer_.pop_back();
  }

  // Replay IMU measurements from correction time to current time
  rclcpp::Time prev_time = from_time;
  for (const auto& imu : imu_buffer_) {
    if (imu.timestamp <= from_time) {
      continue;  // Skip IMU measurements before correction time
    }

    // Calculate dt from previous measurement time
    double dt = (imu.timestamp - prev_time).seconds();
    if (dt <= 0 || dt >= 1.0) {
      continue;  // Skip invalid time steps
    }

    // Run prediction and rebuild state buffer
    predictInternal(imu, dt);
    storeState(imu.timestamp);

    prev_time = imu.timestamp;
  }
}

void DelayedStateEKF::predictInternal(const ImuMeasurement& imu, double dt) {
  // Core prediction logic without IMU buffer update (used by repropagation)

  // Bias-corrected measurements
  Eigen::Vector3d gyro = imu.gyro - state_.segment<3>(BGX);
  Eigen::Vector3d accel = imu.accel - state_.segment<3>(BAX);

  // Extract current quaternion
  Eigen::Quaterniond q(state_(QW), state_(QX), state_(QY), state_(QZ));
  q.normalize();

  // Orientation update (first-order quaternion integration)
  double angle = gyro.norm() * dt;
  Eigen::Quaterniond dq;
  if (angle > 1e-10) {
    Eigen::Vector3d axis = gyro.normalized();
    dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
  } else {
    dq = Eigen::Quaterniond::Identity();
  }
  q = q * dq;
  q.normalize();

  // Rotate acceleration to NED frame and remove gravity
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Vector3d accel_ned = R * accel;
  Eigen::Vector3d gravity(0.0, 0.0, GRAVITY);
  Eigen::Vector3d accel_corrected = accel_ned - gravity;

  // Velocity update (NED frame)
  state_.segment<3>(VX) += accel_corrected * dt;

  // Position update (NED frame)
  state_.segment<3>(PX) += state_.segment<3>(VX) * dt;

  // Update quaternion in state
  state_(QW) = q.w();
  state_(QX) = q.x();
  state_(QY) = q.y();
  state_(QZ) = q.z();

  // Covariance prediction: P = F * P * F^T + Q
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F =
      computeStateTransitionJacobian(gyro, accel, q, dt);
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q_dt = buildProcessNoiseMatrix(dt);
  P_ = F * P_ * F.transpose() + Q_dt;

  // Ensure quaternion normalization
  normalizeQuaternion();
}

void DelayedStateEKF::trimImuBuffer() {
  // Keep only last ~5 seconds of IMU data at 100Hz = 500 measurements
  while (imu_buffer_.size() > buffer_size_) {
    imu_buffer_.pop_front();
  }
}

void DelayedStateEKF::trimStateBuffer() {
  while (state_buffer_.size() > buffer_size_) {
    state_buffer_.pop_front();
  }
}

Eigen::Matrix3d DelayedStateEKF::quaternionToRotationMatrix() const {
  Eigen::Quaterniond q(state_(QW), state_(QX), state_(QY), state_(QZ));
  return q.toRotationMatrix();
}

Eigen::Vector3d DelayedStateEKF::rotateBodyToNed(const Eigen::Vector3d& v_body) const {
  return quaternionToRotationMatrix() * v_body;
}

Eigen::Vector3d DelayedStateEKF::rotateNedToBody(const Eigen::Vector3d& v_ned) const {
  return quaternionToRotationMatrix().transpose() * v_ned;
}

Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>
DelayedStateEKF::computeStateTransitionJacobian(const Eigen::Vector3d& gyro,
                                                const Eigen::Vector3d& /* accel */,
                                                const Eigen::Quaterniond& q,
                                                double dt) {
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F =
      Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();

  // Position depends on velocity
  // dPos/dVel = I_3 * dt
  F.block<3, 3>(PX, VX) = Eigen::Matrix3d::Identity() * dt;

  // Velocity depends on orientation (through rotation of acceleration)
  // dVel/dQuat - simplified approximation
  // Full computation would require d(R*accel)/dq
  Eigen::Matrix3d R = q.toRotationMatrix();

  // Velocity depends on accel bias: dVel/dAccelBias = -R * dt
  F.block<3, 3>(VX, BAX) = -R * dt;

  // Quaternion propagation Jacobian
  // For small angles: dq_new/dq = I + 0.5 * Omega * dt
  // where Omega is the skew-symmetric form of angular velocity
  double wx = gyro.x(), wy = gyro.y(), wz = gyro.z();
  Eigen::Matrix4d Omega;
  Omega << 0, -wx, -wy, -wz,
           wx, 0, wz, -wy,
           wy, -wz, 0, wx,
           wz, wy, -wx, 0;
  F.block<4, 4>(QW, QW) = Eigen::Matrix4d::Identity() + 0.5 * Omega * dt;

  // Quaternion depends on gyro bias
  // dQuat/dGyroBias = -0.5 * Xi * dt (simplified)
  // Xi maps angular velocity to quaternion derivative
  double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
  Eigen::Matrix<double, 4, 3> Xi;
  Xi << -qx, -qy, -qz,
         qw, -qz,  qy,
         qz,  qw, -qx,
        -qy,  qx,  qw;
  F.block<4, 3>(QW, BGX) = -0.5 * Xi * dt;

  // Bias states: random walk, so F_bias = I (already set)

  return F;
}

Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>
DelayedStateEKF::buildProcessNoiseMatrix(double dt) {
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q =
      Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Zero();

  // Position process noise (scales with dt^2 for integrated velocity noise)
  double pos_var = process_noise_position_ * process_noise_position_ * dt;
  Q(PX, PX) = pos_var;
  Q(PY, PY) = pos_var;
  Q(PZ, PZ) = pos_var;

  // Velocity process noise (scales with dt for acceleration noise)
  double vel_var = process_noise_velocity_ * process_noise_velocity_ * dt;
  Q(VX, VX) = vel_var;
  Q(VY, VY) = vel_var;
  Q(VZ, VZ) = vel_var;

  // Quaternion process noise (small, scales with dt)
  double quat_var = process_noise_orientation_ * process_noise_orientation_ * dt;
  Q(QW, QW) = quat_var;
  Q(QX, QX) = quat_var;
  Q(QY, QY) = quat_var;
  Q(QZ, QZ) = quat_var;

  // Gyro bias random walk (scales with dt for random walk)
  double gyro_bias_var = process_noise_gyro_bias_ * process_noise_gyro_bias_ * dt;
  Q(BGX, BGX) = gyro_bias_var;
  Q(BGY, BGY) = gyro_bias_var;
  Q(BGZ, BGZ) = gyro_bias_var;

  // Accel bias random walk (scales with dt for random walk)
  double accel_bias_var = process_noise_accel_bias_ * process_noise_accel_bias_ * dt;
  Q(BAX, BAX) = accel_bias_var;
  Q(BAY, BAY) = accel_bias_var;
  Q(BAZ, BAZ) = accel_bias_var;

  return Q;
}

double DelayedStateEKF::computeMahalanobisDistance(const Eigen::VectorXd& innovation,
                                                   const Eigen::MatrixXd& S) {
  return innovation.transpose() * S.inverse() * innovation;
}

}  // namespace usbl_navigation
