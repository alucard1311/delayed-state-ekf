---
phase: 07-navigation-filter
plan: 01
subsystem: navigation
tags: [ekf, quaternion, imu, eigen, state-estimation, delayed-state]

# Dependency graph
requires:
  - phase: 06-sensor-foundation
    provides: sensor simulators (IMU, DVL, USBL) for testing
provides:
  - DelayedStateEKF class with 16-element state vector
  - State buffer for delayed measurement updates
  - IMU buffer for repropagation
  - Strapdown IMU integration with bias correction
  - DVL and USBL measurement updates with outlier rejection
affects: [07-02, 07-03, 08-demo-visualization]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Delayed-state EKF for handling measurement latency"
    - "Quaternion-based orientation with unit constraint"
    - "NED frame velocity storage for simplified position integration"
    - "Mahalanobis distance outlier rejection"

key-files:
  created:
    - src/usbl_navigation/include/usbl_navigation/delayed_state_ekf.hpp
    - src/usbl_navigation/src/ekf/delayed_state_ekf.cpp
  modified:
    - src/usbl_navigation/CMakeLists.txt

key-decisions:
  - "16-element state vector (15 DOF with quaternion unit constraint)"
  - "Velocity stored in NED frame to simplify position integration"
  - "State transition Jacobian includes quaternion propagation and bias coupling"
  - "Chi-squared 3DOF 99% threshold (9.21) for outlier rejection"

patterns-established:
  - "DelayedStateEKF class as reusable filter component"
  - "BufferedState struct for historical state storage"
  - "ImuMeasurement struct for raw sensor data"
  - "Repropagation pattern for delayed measurement fusion"

# Metrics
duration: 8min
completed: 2026-01-20
---

# Phase 07 Plan 01: Delayed-State EKF Core Summary

**15-state delayed EKF with quaternion orientation, strapdown IMU integration, and state buffer for USBL latency handling**

## Performance

- **Duration:** 8 min
- **Started:** 2026-01-20T12:00:00Z
- **Completed:** 2026-01-20T12:08:00Z
- **Tasks:** 2
- **Files modified:** 3

## Accomplishments

- Created DelayedStateEKF class with 16-element state vector (position, velocity, quaternion, gyro/accel biases)
- Implemented strapdown IMU integration with bias-corrected measurements
- State buffer stores 500 historical states for delayed USBL updates
- IMU buffer enables repropagation after delayed measurement corrections
- DVL update transforms body-frame velocity to NED-frame state
- USBL delayed update with Mahalanobis outlier rejection

## Task Commits

Each task was committed atomically:

1. **Task 1: Create delayed_state_ekf.hpp with 15-state vector and state buffer** - `f841289` (feat)
2. **Task 2: Implement EKF prediction step with IMU integration** - `2e75f53` (feat)

## Files Created/Modified

- `src/usbl_navigation/include/usbl_navigation/delayed_state_ekf.hpp` - Header with StateIdx enum, BufferedState/ImuMeasurement structs, DelayedStateEKF class declaration (273 lines)
- `src/usbl_navigation/src/ekf/delayed_state_ekf.cpp` - Full implementation with predict(), updateDvl(), updateUsblDelayed(), state buffer management, Jacobian computation (528 lines)
- `src/usbl_navigation/CMakeLists.txt` - Added delayed_state_ekf library target

## Decisions Made

| Decision | Rationale |
|----------|-----------|
| 16-element state (not 15) | Quaternion has 4 components; unit constraint handled via normalization |
| Velocity in NED frame | Simplifies position integration (no rotation needed) |
| Chi-squared 9.21 threshold | 3DOF, 99% confidence for USBL outlier rejection |
| State buffer size 500 | ~5 seconds at 100Hz IMU rate, covers max USBL delay |

## Deviations from Plan

None - plan executed exactly as written.

## Issues Encountered

None.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- DelayedStateEKF core ready for integration
- Ready for Plan 07-02: Navigation filter node wrapper
- DVL and USBL update methods implemented and awaiting node integration

---
*Phase: 07-navigation-filter*
*Completed: 2026-01-20*
