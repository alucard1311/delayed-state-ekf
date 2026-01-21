# Roadmap: Stonefish AUV Sim-to-Real Pipeline

## Overview

Build a complete AUV simulation demonstrating autonomous underwater navigation. v1.0 delivered the vertical slice (dive → waypoint → surface). v2.0 adds USBL navigation with delayed-state EKF fusion — demonstrating understanding of real underwater navigation challenges.

## Milestones

- ✅ **v1.0 Vertical Slice** - Phases 1-5 (complete)
- 🚧 **v2.0 USBL Navigation Demo** - Phases 6-8 (in progress)

## Phases

**Phase Numbering:**
- Integer phases (1, 2, 3): Planned milestone work
- Decimal phases (2.1, 2.2): Urgent insertions (marked with INSERTED)

<details>
<summary>✅ v1.0 Vertical Slice (Phases 1-5) - COMPLETE</summary>

- [x] **Phase 1: Infrastructure** - Docker + Stonefish + ROS2 foundation
- [x] **Phase 2: Navigation** - C++ EKF state estimator with sensor fusion
- [x] **Phase 3: Control** - C++ PID controllers for depth/heading/velocity
- [x] **Phase 4: Planning** - Python mission planner + state machine
- [x] **Phase 5: Demo** - Full vertical slice mission integration

### Phase 1: Infrastructure
**Goal**: Containerized simulation environment with Stonefish and ROS2
**Depends on**: Nothing (first phase)
**Requirements**: INFRA-01, INFRA-02, INFRA-03, INFRA-04
**Success Criteria** (what must be TRUE):
  1. Docker container launches Stonefish simulator
  2. ROS2 nodes can publish and subscribe to topics
  3. X11 forwarding shows Stonefish visualization
  4. Single docker-compose command starts everything

Plans:
- [x] 01-01: Docker infrastructure (Dockerfile, docker-compose.yml, helper scripts)
- [x] 01-02: ROS2 workspace and Stonefish integration

### Phase 2: Navigation
**Goal**: C++ EKF state estimator fusing DVL, IMU, and depth sensors
**Depends on**: Phase 1
**Requirements**: NAV-01, NAV-02, NAV-03, NAV-04, NAV-05
**Success Criteria** (what must be TRUE):
  1. EKF publishes filtered pose estimate at consistent rate
  2. Position estimate updates when DVL data arrives
  3. Orientation estimate updates when IMU data arrives
  4. Depth estimate updates when depth sensor data arrives
  5. EKF continues with dead reckoning when sensors drop out

Plans:
- [x] 02-01: EKF core package (state vector, prediction model)
- [x] 02-02: IMU and pressure measurement updates
- [x] 02-03: DVL measurement update and dead reckoning
- [x] 02-04: Launch file and EKF verification

### Phase 3: Control
**Goal**: C++ PID controllers for depth, heading, and velocity
**Depends on**: Phase 2
**Requirements**: CTRL-01, CTRL-02, CTRL-03, CTRL-04, CTRL-05
**Success Criteria** (what must be TRUE):
  1. Depth controller holds commanded depth within ±0.5m
  2. Heading controller holds commanded heading within ±5°
  3. Velocity controller maintains commanded speed within ±0.1m/s
  4. Thruster commands sent to Stonefish actuators

Plans:
- [x] 03-01: Control package foundation (PID class, control node skeleton)
- [x] 03-02: Depth and heading controllers
- [x] 03-03: Launch file and controller verification

### Phase 4: Planning
**Goal**: Python mission planner with state machine for autonomous behavior
**Depends on**: Phase 3
**Requirements**: PLAN-01, PLAN-02, PLAN-03, PLAN-04, SM-01, SM-02, SM-03, SM-04
**Success Criteria** (what must be TRUE):
  1. Mission planner loads waypoint sequence from config
  2. State machine progresses through IDLE→DIVING→NAVIGATING→SURFACING→COMPLETE
  3. Waypoint arrival detected and next waypoint commanded
  4. ERROR state triggers safe surfacing behavior
  5. Current state published for external monitoring

Plans:
- [x] 04-01: Mission planner package foundation (waypoints, navigation utils)
- [x] 04-02: State machine and planner node
- [x] 04-03: Integration and verification

### Phase 5: Demo
**Goal**: Full vertical slice — autonomous dive→waypoint→surface mission
**Depends on**: Phase 4
**Requirements**: DEMO-01, DEMO-02, DEMO-03, DEMO-04
**Success Criteria** (what must be TRUE):
  1. AUV dives from surface to 5m depth
  2. AUV navigates to at least one waypoint at depth
  3. AUV surfaces after waypoint sequence completes
  4. Entire mission runs without operator intervention

Plans:
- [x] 05-01: Mission configuration and demo verification

</details>

### 🚧 v2.0 USBL Navigation Demo (In Progress)

**Milestone Goal:** Demonstrate delayed-state EKF fusion with USBL — the key technique for real underwater navigation where acoustic positioning has multi-second latency.

- [x] **Phase 6: Sensor Foundation** - Truth generator + IMU/DVL/USBL simulators
- [ ] **Phase 7: Navigation Filter** - 15-state delayed-state EKF
- [ ] **Phase 8: Demo & Visualization** - Metrics, plots, RViz, scenarios

### Phase 6: Sensor Foundation
**Goal**: Standalone sensor simulators for controlled USBL navigation testing
**Depends on**: Nothing (standalone from v1.0)
**Requirements**: SIM-01, SIM-02, SIM-03, SIM-04, SIM-05, SIM-06, SIM-07
**Success Criteria** (what must be TRUE):
  1. Truth generator publishes smooth lawnmower trajectory at 100Hz
  2. IMU publishes data with realistic noise and slowly drifting bias
  3. DVL publishes body-frame velocity at 5Hz with bottom_lock status
  4. DVL drops out during canyon scenario (t=120s-150s)
  5. USBL publishes DELAYED positions (timestamp = measurement time, not publish time)
  6. USBL delay visible in logs (~0.25s for 50m depth)
  7. USBL occasionally produces outliers and dropouts
**Research**: Unlikely (internal sensor models, standard ROS2 patterns)
**Plans**: TBD

Plans:
- [x] 06-01: Package scaffolding and truth generator
- [x] 06-02: IMU and DVL simulators
- [x] 06-03: USBL simulator with delayed timestamps

### Phase 7: Navigation Filter
**Goal**: 15-state delayed-state EKF with USBL delayed measurement updates
**Depends on**: Phase 6
**Requirements**: DEKF-01, DEKF-02, DEKF-03, DEKF-04, DEKF-05, DEKF-06, DEKF-07, DEKF-08
**Success Criteria** (what must be TRUE):
  1. State buffer stores ~5 seconds of historical states
  2. EKF prediction integrates IMU at 100Hz
  3. DVL update reduces velocity uncertainty
  4. USBL update finds correct historical state by timestamp
  5. Outliers rejected via Mahalanobis gating (visible in logs)
  6. State repropagated to current time after USBL correction
  7. Navigation odometry published at 50Hz
  8. TF tree broadcasts world → odom → base_link
**Research**: Unlikely (EKF patterns from v1.0, delayed-state is extension)
**Plans**: TBD

Plans:
- [ ] 07-01: State buffer and EKF prediction
- [ ] 07-02: DVL and USBL measurement updates
- [ ] 07-03: Navigation node and integration

### Phase 8: Demo & Visualization
**Goal**: Publication-quality demo with metrics and visualizations
**Depends on**: Phase 7
**Requirements**: VIZ-01, VIZ-02, VIZ-03, VIZ-04, VIZ-05, VIZ-06, SCEN-01, SCEN-02, SCEN-03
**Success Criteria** (what must be TRUE):
  1. Metrics CSV captures truth, estimate, error, covariance over time
  2. Sawtooth plot shows error dropping after each USBL fix
  3. Trajectory plot shows estimate tracking truth
  4. Covariance plot shows error within 3σ bounds
  5. RViz clearly displays truth vs estimate paths
  6. Nominal scenario: final error <2m, max error <5m
  7. DVL dropout scenario: error grows then recovers with USBL
  8. Outlier scenario: outliers rejected, no position jumps
**Research**: Unlikely (Python plotting, RViz configuration)
**Plans**: TBD

Plans:
- [ ] 08-01: Metrics logger and plotting scripts
- [ ] 08-02: RViz configuration and launch file
- [ ] 08-03: Run scenarios and generate results

## Progress

**Execution Order:**
Phases execute in numeric order: 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8

| Phase | Milestone | Plans Complete | Status | Completed |
|-------|-----------|----------------|--------|-----------|
| 1. Infrastructure | v1.0 | 2/2 | Complete | 2026-01-17 |
| 2. Navigation | v1.0 | 4/4 | Complete | 2026-01-17 |
| 3. Control | v1.0 | 3/3 | Complete | 2026-01-20 |
| 4. Planning | v1.0 | 3/3 | Complete | 2026-01-20 |
| 5. Demo | v1.0 | 1/1 | Complete | 2026-01-20 |
| 6. Sensor Foundation | v2.0 | 3/3 | Complete | 2026-01-21 |
| 7. Navigation Filter | v2.0 | 0/3 | Not started | - |
| 8. Demo & Visualization | v2.0 | 0/3 | Not started | - |
