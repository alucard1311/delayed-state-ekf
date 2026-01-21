# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-01-17)

**Core value:** Full vertical slice — one complete mission (dive → waypoint → surface) with all layers working
**Current focus:** Phase 4 complete — Python mission planner + state machine implemented

## Current Position

Phase: 4 of 5 (Planning)
Plan: 2 of 2 in current phase
Status: Phase 04 complete
Last activity: 2026-01-20 — Completed plan 04-02 (state machine and planner node)

Progress: ██████████████████████████████ 100% (12 of 12 plans estimated)

## Performance Metrics

**Velocity:**
- Total plans completed: 12
- Average duration: ~7min
- Total execution time: ~84 min

**By Phase:**

| Phase | Plans | Total | Status |
|-------|-------|-------|--------|
| 01-infrastructure | 2/2 | ~30min | Complete |
| 02-navigation | 4/4 | ~23min | Complete |
| 03-control | 3/3 | ~10min | Complete |
| 04-planning | 2/2 | ~14min | Complete |

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
| 03-01 | Generic PID class | Reusable for depth, heading, velocity controllers |
| 03-01 | 50Hz control loop | Matches EKF output rate |
| 03-01 | Depth = -z convention | Positive depth is below surface |
| 03-02 | Angle wrapping for heading | Prevents 270-degree spins on ±pi discontinuity |
| 03-02 | Same command to bow/stern yaw | Stern is inverted in auv.xml |
| 03-02 | dt validation in control loop | Skip if dt <= 0 or > 1.0 to avoid PID spikes |
| 04-01 | Pure functions in navigation_utils | No ROS deps, easy to unit test |
| 04-01 | Dataclass for Waypoint | Clean, immutable data structure |
| 04-01 | YAML for mission config | Human-readable, standard ROS pattern |
| 04-01 | Heading convention: 0=+X, pi/2=+Y | Matches atan2(dy, dx) standard |
| 04-02 | Auto-start on EKF data | Simpler operation, no manual trigger needed |
| 04-02 | ERROR triggers safe surfacing | Safety-first design |
| 04-02 | State published as String | Human-readable monitoring |
| 04-02 | Quaternion yaw extraction inline | Avoid external dependency |

### Pending Todos

None.

### Blockers/Concerns

None.

## Session Continuity

Last session: 2026-01-20
Stopped at: Plan 04-02 complete, Phase 04 complete

**Plan 04-02 artifacts:**
- src/auv_planner/auv_planner/state_machine.py - MissionStateMachine with all states
- src/auv_planner/auv_planner/planner_node.py - ROS2 planner node
- src/auv_planner/launch/planner.launch.py - Launch file with configurable parameters

**State machine states:**
- IDLE -> DIVING -> NAVIGATING -> SURFACING -> COMPLETE
- ERROR state triggers safe surfacing

**Planner node interfaces:**
- Subscribes: `/auv/ekf/pose` (nav_msgs/Odometry)
- Publishes: `/auv/cmd/depth`, `/auv/cmd/heading`, `/auv/cmd/velocity` (std_msgs/Float64)
- Publishes: `/auv/mission/state` (std_msgs/String)

**Files modified (uncommitted from Phase 3):**
- src/auv_sim/description/auv.xml (material density, world_transform position)
- src/auv_control/src/control_node.cpp (depth/heave sign fixes)
- src/auv_ekf/src/ekf_node.cpp (uncommitted changes)
- src/auv_sim/launch/full_simulation.launch.py (new - full stack launcher)

Resume file: None
Next action: Phase 05 (Integration) or full system testing
