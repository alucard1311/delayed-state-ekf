---
phase: 06-sensor-foundation
plan: 02
subsystem: navigation
tags: [ros2, eigen, imu, dvl, sensor-simulation, noise-model]

# Dependency graph
requires:
  - phase: 06-01
    provides: usbl_navigation package structure, truth generator node
provides:
  - IMU simulator with noise and bias drift at 100Hz
  - DVL simulator with body-frame conversion at 5Hz
  - Canyon dropout simulation (t=120-150s)
  - Random dropout simulation
affects: [06-03, 07-navigation-filter, 08-demo-visualization]

# Tech tracking
tech-stack:
  added: []
  patterns: [ROS2 timer-based sensor simulation, random walk bias model]

key-files:
  created:
    - src/usbl_navigation/include/usbl_navigation/imu_simulator_node.hpp
    - src/usbl_navigation/src/nodes/imu_simulator_node.cpp
    - src/usbl_navigation/src/nodes/imu_simulator_main.cpp
    - src/usbl_navigation/include/usbl_navigation/dvl_simulator_node.hpp
    - src/usbl_navigation/src/nodes/dvl_simulator_node.cpp
    - src/usbl_navigation/src/nodes/dvl_simulator_main.cpp
  modified:
    - src/usbl_navigation/CMakeLists.txt
    - src/usbl_navigation/launch/simulation.launch.py

key-decisions:
  - "IMU bias drift uses random walk model with configurable instability"
  - "DVL transforms world velocity to body frame to match real DVL behavior"
  - "Gravity added to body-frame acceleration (specific force measurement)"

patterns-established:
  - "Sensor simulators subscribe to /truth/odometry and publish noisy measurements"
  - "Bias logging every 10s for debugging"
  - "Bottom lock status published separately from velocity"

# Metrics
duration: 4min
completed: 2026-01-21
---

# Phase 6 Plan 02: IMU and DVL Simulators Summary

**IMU simulator with 100Hz noisy data including bias drift, DVL simulator with 5Hz body-frame velocity and canyon dropout scenario**

## Performance

- **Duration:** 4 min
- **Started:** 2026-01-21T01:48:00Z
- **Completed:** 2026-01-21T01:52:11Z
- **Tasks:** 3
- **Files modified:** 8

## Accomplishments

- IMU simulator with gyro/accel noise density and random walk bias drift
- DVL simulator transforming world velocity to body frame
- Canyon dropout scenario (t=120-150s) with configurable enable flag
- Random dropout based on probability parameter
- All sensor simulators launch together from single launch file

## Task Commits

Each task was committed atomically:

1. **Task 1: Create imu_simulator_node with noise and bias drift** - `2e71388` (feat)
2. **Task 2: Create dvl_simulator_node with body-frame conversion and dropouts** - `81e07f9` (feat)
3. **Task 3: Update CMakeLists.txt and launch file** - `348bddd` (feat)

## Files Created/Modified

- `src/usbl_navigation/include/usbl_navigation/imu_simulator_node.hpp` - IMU simulator class definition
- `src/usbl_navigation/src/nodes/imu_simulator_node.cpp` - IMU noise model implementation
- `src/usbl_navigation/src/nodes/imu_simulator_main.cpp` - IMU node entry point
- `src/usbl_navigation/include/usbl_navigation/dvl_simulator_node.hpp` - DVL simulator class definition
- `src/usbl_navigation/src/nodes/dvl_simulator_node.cpp` - DVL simulation with dropouts
- `src/usbl_navigation/src/nodes/dvl_simulator_main.cpp` - DVL node entry point
- `src/usbl_navigation/CMakeLists.txt` - Added IMU and DVL executables
- `src/usbl_navigation/launch/simulation.launch.py` - Added IMU and DVL to launch

## Decisions Made

- **Random walk bias model**: Bias evolves as b += N(0,1) * instability * sqrt(dt)
- **Gravity handling**: Added to body-frame acceleration as specific force measurement
- **World-to-body velocity transform**: DVL measures v_body = q.inverse() * v_world
- **Bias logging interval**: Every 10 seconds for debugging without flooding logs

## Deviations from Plan

None - plan executed exactly as written.

## Issues Encountered

None - all tasks completed successfully.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- IMU publishes at 100Hz with noise and drifting bias
- DVL publishes at 5Hz in body frame with dropout simulation
- Ready for Plan 06-03 (USBL simulator with delayed timestamps)
- All sensor noise parameters loaded from sensor_noise.yaml

---
*Phase: 06-sensor-foundation*
*Completed: 2026-01-21*
