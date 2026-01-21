# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-01-20)

**Core value:** Delayed-state EKF fusion with USBL — the key technique for real underwater navigation
**Current focus:** Phase 8 — Demo & Visualization (Complete)

## Current Position

Phase: 8 of 8 (Demo & Visualization)
Plan: 3 of 3 in current phase
Status: COMPLETE - All phases finished
Last activity: 2026-01-21 — Checkpoint approved for 08-03

Progress: ████████████████████████████████ 100% (22 of 22 plans executed)

## Performance Metrics

**Velocity:**
- Total plans completed: 21 (13 v1.0 + 8 v2.0)
- Average duration: ~7min
- Total execution time: ~140 min

**By Phase:**

| Phase | Milestone | Plans | Status |
|-------|-----------|-------|--------|
| 01-infrastructure | v1.0 | 2/2 | Complete |
| 02-navigation | v1.0 | 4/4 | Complete |
| 03-control | v1.0 | 3/3 | Complete |
| 04-planning | v1.0 | 3/3 | Complete |
| 05-demo | v1.0 | 1/1 | Complete |
| 06-sensor-foundation | v2.0 | 3/3 | Complete |
| 07-navigation-filter | v2.0 | 3/3 | Complete |
| 08-demo-visualization | v2.0 | 3/3 | Complete |

## Accumulated Context

### Decisions

Decisions are logged in PROJECT.md Key Decisions table.
Recent decisions affecting current work:

| Phase | Decision | Rationale |
|-------|----------|-----------|
| v2.0 | Delayed-state EKF | Real USBL has 0.2-0.5s latency, must handle properly |
| v2.0 | Standalone usbl_navigation package | Clean separation from v1.0, focused demo |
| v2.0 | Sensor simulators (not Stonefish) | Precise control over test scenarios |
| v2.0 | 15-state vector | pos(3), vel(3), quat(4), gyro_bias(3), accel_bias(3) |
| 06-01 | NED coordinate convention | Standard for marine robotics, +X North +Y East +Z Down |
| 06-01 | Eigen for quaternions | Direct Eigen::Quaterniond instead of tf2 conversions |
| 06-02 | Random walk bias model | Bias evolves as b += N(0,1) * instability * sqrt(dt) |
| 06-02 | DVL world-to-body transform | DVL measures body-frame velocity, not world-frame |
| 06-03 | USBL delayed timestamps | measurement_time = now() - delay, not current time |
| 06-03 | Range-dependent noise | max(2% * range, 0.3m) per axis |
| 07-01 | 16-element state vector | Quaternion has 4 components; unit constraint via normalization |
| 07-01 | Velocity in NED frame | Simplifies position integration (no rotation needed) |
| 07-01 | Chi-squared 9.21 threshold | 3DOF, 99% confidence for USBL outlier rejection |
| 07-02 | Joseph form covariance update | Numerically stable for ill-conditioned matrices |
| 07-03 | 50Hz odometry publish rate | Matches typical control loop requirements |
| 08-02 | 2Hz path publish rate | RViz efficiency over 50Hz odometry rate |
| 08-02 | OpaqueFunction launch pattern | Scenario-based parameter overrides with choices validation |
| 08-01 | ApproximateTimeSynchronizer 10ms | Truth at 100Hz, odom at 50Hz need fuzzy sync |
| 08-01 | Numpy for CSV parsing | pandas not available as system package |
| 08-03 | 100m initial position covariance | Allow USBL to bootstrap from unknown position |
| 08-03 | 0.5 Hz USBL rate | Faster corrections for better convergence |
| 08-03 | Permissive Mahalanobis threshold | 100.0 for demo robustness |

### Pending Todos

None.

### Blockers/Concerns

**Heading Drift Issue (08-03):**
- EKF heading drifts without compass/magnetometer aiding
- Causes divergence after ~60s (first turn in lawnmower)
- Full 180s scenario not achievable with current system
- Acceptable for demo of USBL navigation concept

## Session Continuity

Last session: 2026-01-21
Stopped at: Checkpoint in 08-03-PLAN.md - Awaiting human verification

**v1.0 Status:**
- All 5 phases complete (13 plans)
- Autonomous dive->waypoint->surface mission verified

**Phase 6 Status:**
- 06-01: Package scaffolding + truth generator - COMPLETE
- 06-02: IMU + DVL simulators - COMPLETE
- 06-03: USBL simulator with delayed timestamps - COMPLETE

**Phase 7 Status:**
- 07-01: State buffer and EKF prediction - COMPLETE
- 07-02: DVL and USBL measurement updates - COMPLETE
- 07-03: Navigation node and integration - COMPLETE

**Phase 8 Status:**
- 08-01: Metrics logger node - COMPLETE
- 08-02: RViz config and launch updates - COMPLETE
- 08-03: Scenario testing and analysis - COMPLETE (checkpoint approved)

**Next action:** Milestone v2.0 complete. Ready for archival or next milestone planning.

Resume file: None

## Milestone v2.0 Summary

All phases complete. The USBL navigation system demonstrates:
- Delayed-state EKF fusion with USBL measurements
- Filter convergence to <2m error within 10-15 seconds
- USBL sawtooth correction pattern
- Known limitation: heading drift without compass aiding (documented and accepted)
