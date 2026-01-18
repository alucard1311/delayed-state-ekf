# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-01-17)

**Core value:** Full vertical slice — one complete mission (dive → waypoint → surface) with all layers working
**Current focus:** Phase 2 complete — Ready for Phase 3 (Control)

## Current Position

Phase: 2 of 5 (Navigation)
Plan: 4 of 4 in current phase
Status: Complete
Last activity: 2026-01-18 — Completed 02-04-PLAN.md (Launch file and EKF verification)

Progress: ████████████████████ 60% (6 of 10 plans)

## Performance Metrics

**Velocity:**
- Total plans completed: 6
- Average duration: ~9min
- Total execution time: ~53 min

**By Phase:**

| Phase | Plans | Total | Status |
|-------|-------|-------|--------|
| 01-infrastructure | 2/2 | ~30min | Complete |
| 02-navigation | 4/4 | ~23min | Complete |

## Accumulated Context

### Decisions

Decisions are logged in PROJECT.md Key Decisions table.
Recent decisions affecting current work:

| Phase | Decision | Rationale |
|-------|----------|-----------|
| 01-01 | osrf/ros:humble-desktop base image | Includes full desktop tools for Stonefish visualization |
| 01-01 | Build Stonefish from main branch | v1.5 tag incompatible with stonefish_ros2 |
| 01-02 | GCC-13 with C++20 | Required by latest Stonefish for `<format>` header |
| 01-02 | AUV_Composite material (950 kg/m³) | Provides positive buoyancy |
| 01-02 | Push-type actuators | Simple virtual force, no mesh needed |
| 01-02 | Sensors added early | IMU, DVL, Pressure, Odometry ready for Phase 2 EKF |
| 02-01 | 12-state EKF vector | Position, velocity, orientation, angular velocity for full 6-DOF |
| 02-01 | 50Hz prediction rate | Faster than slowest sensor (DVL at 10Hz) |
| 02-01 | ZYX Euler convention | Standard for aerospace/underwater navigation |
| 02-02 | Generic measurementUpdate() | Reusable for any sensor with H matrix pattern |
| 02-02 | Remove stonefish_ros2 dep | EKF only uses standard ROS message types |
| 02-03 | Re-add stonefish_ros2 dep | Required for DVL message type |
| 02-03 | SENSOR_TIMEOUT = 1.0 second | Reasonable timeout for dead reckoning detection |
| 02-03 | 2x process noise in dead reckoning | Reflects uncertainty without destabilizing filter |
| 02-04 | 50Hz default prediction rate | Faster than slowest sensor, matches IMU rate |
| 02-04 | Parameter-driven noise matrices | Allows tuning without recompilation |

### Pending Todos

None.

### Blockers/Concerns

None — EKF fuses all sensors with dead reckoning fallback.

## Session Continuity

Last session: 2026-01-18
Stopped at: Completed 02-04-PLAN.md (Launch file and EKF verification)
Resume file: None
Next action: Begin Phase 03-control
