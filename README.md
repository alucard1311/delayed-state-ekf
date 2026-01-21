# Delayed-State EKF for Underwater Navigation

A production-grade implementation of a **delayed-state Extended Kalman Filter** for USBL-aided underwater navigation. Demonstrates sensor fusion techniques used in real AUV systems where acoustic positioning has multi-second latency.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![C++](https://img.shields.io/badge/C%2B%2B-17-blue)
![Python](https://img.shields.io/badge/Python-3.10+-green)
![Docker](https://img.shields.io/badge/Docker-Containerized-blue)
![License](https://img.shields.io/badge/License-MIT-green)

## The Problem

Underwater vehicles operate in GPS-denied environments and rely on acoustic positioning systems like USBL (Ultra-Short Baseline). The challenge: **acoustic signals travel at ~1500 m/s in water**, causing measurement delays of 0.2-0.5+ seconds depending on depth.

A naive EKF that treats delayed measurements as current will produce incorrect state estimates. The solution is a **delayed-state EKF** that:

1. Maintains a buffer of historical states
2. Finds the state at measurement time
3. Applies the correction to the historical state
4. Repropagates to current time

## Results

### Position Error (Sawtooth Pattern)
The characteristic sawtooth shows error growing during dead reckoning (IMU+DVL), then dropping sharply when a USBL fix arrives:

```
Error
(m)
 5 |    /\      /\      /\
 4 |   /  \    /  \    /  \
 3 |  /    \  /    \  /    \
 2 | /      \/      \/      \
 1 |/                        \____
   +------------------------------- Time
        USBL fixes (every ~2-5s)
```

### Trajectory Tracking
The filter successfully tracks a lawnmower survey pattern, with the estimate (orange) closely following ground truth (blue).

### Covariance Consistency
Position errors remain within the 3-sigma bounds predicted by the filter covariance, indicating a well-tuned estimator.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    USBL Navigation System                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │    Truth     │  │     IMU      │  │     DVL      │          │
│  │  Generator   │  │  Simulator   │  │  Simulator   │          │
│  │  (100 Hz)    │  │  (100 Hz)    │  │   (5 Hz)     │          │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘          │
│         │                 │                 │                   │
│         │    ┌────────────┴─────────────────┘                   │
│         │    │                                                  │
│         │    │         ┌──────────────┐                         │
│         │    │         │    USBL      │                         │
│         │    │         │  Simulator   │                         │
│         │    │         │ (0.5 Hz +    │                         │
│         │    │         │  delay)      │                         │
│         │    │         └──────┬───────┘                         │
│         │    │                │                                 │
│         ▼    ▼                ▼                                 │
│  ┌─────────────────────────────────────────┐                   │
│  │         Delayed-State EKF               │                   │
│  │  ┌─────────────────────────────────┐   │                   │
│  │  │  State Buffer (500 entries)     │   │                   │
│  │  │  - Position, Velocity, Attitude │   │                   │
│  │  │  - Gyro & Accel Bias            │   │                   │
│  │  │  - Covariance matrices          │   │                   │
│  │  └─────────────────────────────────┘   │                   │
│  │                                         │                   │
│  │  • IMU Prediction (100 Hz)             │                   │
│  │  • DVL Velocity Update (5 Hz)          │                   │
│  │  • USBL Delayed Update + Repropagate   │                   │
│  │  • Mahalanobis Outlier Rejection       │                   │
│  └─────────────────┬───────────────────────┘                   │
│                    │                                            │
│                    ▼                                            │
│  ┌─────────────────────────────────────────┐                   │
│  │     Navigation Odometry (50 Hz)         │                   │
│  │     TF: world -> base_link              │                   │
│  └─────────────────────────────────────────┘                   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Features

### Delayed-State EKF
- **15-state vector**: Position (3), Velocity (3), Quaternion (4), Gyro Bias (3), Accel Bias (3)
- **State buffer**: 500 historical states (~5 seconds at 100 Hz)
- **USBL delayed update**: Finds historical state, applies correction, repropagates
- **Mahalanobis gating**: Chi-squared outlier rejection (99% confidence)
- **Joseph form**: Numerically stable covariance update

### Sensor Simulators
- **Truth Generator**: Smooth lawnmower survey trajectory at 100 Hz
- **IMU Simulator**: Gaussian noise + random-walk bias drift
- **DVL Simulator**: Body-frame velocity, bottom-lock status, canyon dropout scenario
- **USBL Simulator**: Depth-dependent delay, range-dependent noise, outlier injection

### Visualization & Metrics
- **Metrics Logger**: Synchronized CSV logging of truth, estimate, error, covariance
- **Plotting Scripts**: Publication-quality sawtooth, trajectory, and 3-sigma plots
- **RViz Integration**: Real-time truth vs estimate path visualization

## Tech Stack

| Component | Technology |
|-----------|------------|
| Framework | ROS2 Humble |
| Core Algorithms | C++17 |
| Math Library | Eigen3 |
| Plotting | Python + Matplotlib |
| Containerization | Docker |
| Build System | colcon / CMake |

## Quick Start

### Prerequisites
- Docker with X11 forwarding support
- ~4GB disk space

### Run the Demo

```bash
# Clone the repository
git clone git@github.com:alucard1311/delayed-state-ekf.git
cd delayed-state-ekf

# Build and run with Docker
docker-compose up --build

# Inside the container:
colcon build --packages-select usbl_navigation
source install/setup.bash

# Run nominal scenario (60 seconds)
mkdir -p /tmp/usbl_demo
ros2 launch usbl_navigation simulation.launch.py \
  scenario:=nominal \
  output_dir:=/tmp/usbl_demo

# Generate plots
python3 install/usbl_navigation/lib/usbl_navigation/plot_results.py \
  /tmp/usbl_demo/navigation_metrics.csv \
  --output-dir /tmp/usbl_demo
```

### Scenarios

| Scenario | Description |
|----------|-------------|
| `nominal` | Standard operation with 5% USBL outliers |
| `canyon_dropout` | DVL loses bottom lock (t=120-150s), USBL-only navigation |
| `high_outlier` | 20% USBL outlier rate, tests Mahalanobis gating |

```bash
# Run specific scenario
ros2 launch usbl_navigation simulation.launch.py scenario:=canyon_dropout
```

## Project Structure

```
src/
├── usbl_navigation/           # USBL Navigation Demo (v2.0)
│   ├── include/
│   │   └── usbl_navigation/
│   │       ├── delayed_state_ekf.hpp    # Core EKF implementation
│   │       ├── state_buffer.hpp         # Historical state storage
│   │       ├── navigation_node.hpp      # ROS2 node wrapper
│   │       ├── truth_generator_node.hpp
│   │       ├── imu_simulator_node.hpp
│   │       ├── dvl_simulator_node.hpp
│   │       ├── usbl_simulator_node.hpp
│   │       ├── metrics_logger_node.hpp
│   │       └── path_publisher_node.hpp
│   ├── src/
│   │   ├── ekf/                         # EKF core algorithms
│   │   ├── simulators/                  # Sensor simulators
│   │   └── nodes/                       # ROS2 node implementations
│   ├── config/
│   │   ├── simulation_params.yaml
│   │   ├── sensor_noise.yaml
│   │   ├── ekf_params.yaml
│   │   └── rviz_config.rviz
│   ├── scripts/
│   │   └── plot_results.py
│   └── launch/
│       └── simulation.launch.py
│
├── auv_ekf/                   # Basic EKF (v1.0)
├── auv_control/               # PID Controllers (v1.0)
├── auv_planner/               # Mission Planner (v1.0)
└── auv_sim/                   # Stonefish Integration (v1.0)
```

## Key Implementation Details

### Delayed Measurement Update

```cpp
void DelayedStateEKF::processUSBL(const Eigen::Vector3d& position,
                                   double measurement_time) {
    // 1. Find historical state at measurement time
    auto historical = state_buffer_.getStateAt(measurement_time);

    // 2. Compute innovation (measurement - predicted)
    Eigen::Vector3d innovation = position - historical.position;

    // 3. Mahalanobis distance for outlier rejection
    Eigen::Matrix3d S = H * historical.covariance * H.T + R;
    double mahalanobis = innovation.T * S.inverse() * innovation;

    if (mahalanobis > CHI_SQUARED_THRESHOLD) {
        return;  // Reject outlier
    }

    // 4. Apply correction to historical state
    Eigen::MatrixXd K = historical.covariance * H.T * S.inverse();
    historical.state += K * innovation;

    // 5. Repropagate to current time using stored IMU data
    repropagate(historical, current_time_);
}
```

### State Vector

```
x = [p_x, p_y, p_z,           # Position (NED frame)
     v_x, v_y, v_z,           # Velocity (NED frame)
     q_w, q_x, q_y, q_z,      # Orientation (quaternion)
     b_gx, b_gy, b_gz,        # Gyroscope bias
     b_ax, b_ay, b_az]        # Accelerometer bias
```

## Performance

| Metric | Nominal Scenario |
|--------|------------------|
| Final Position Error | < 2m |
| Max Position Error | < 5m |
| USBL Update Rate | 0.5 Hz |
| Navigation Output | 50 Hz |

## Related Work

This implementation draws from:
- Delayed-state Kalman filtering techniques used in underwater navigation
- INS/USBL fusion architectures common in commercial AUV systems
- ROS2 best practices for real-time sensor fusion

## License

MIT License - See [LICENSE](LICENSE) for details.

## Author

Built as a demonstration of underwater navigation techniques for autonomous systems.
