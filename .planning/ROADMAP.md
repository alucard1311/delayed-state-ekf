# Roadmap: Stonefish AUV Sim-to-Real Pipeline

## Overview

Build a complete AUV simulation demonstrating autonomous underwater navigation. Start with Docker/ROS2 infrastructure, add C++ EKF for state estimation, implement C++ PID control, add Python planning with state machine, then integrate into a full dive→waypoint→surface demo mission.

## Phases

**Phase Numbering:**
- Integer phases (1, 2, 3): Planned milestone work
- Decimal phases (2.1, 2.2): Urgent insertions (marked with INSERTED)

- [x] **Phase 1: Infrastructure** - Docker + Stonefish + ROS2 foundation
- [x] **Phase 2: Navigation** - C++ EKF state estimator with sensor fusion
- [ ] **Phase 3: Control** - C++ PID controllers for depth/heading/velocity
- [ ] **Phase 4: Planning** - Python mission planner + state machine
- [ ] **Phase 5: Demo** - Full vertical slice mission integration

## Phase Details

### Phase 1: Infrastructure
**Goal**: Containerized simulation environment with Stonefish and ROS2
**Depends on**: Nothing (first phase)
**Requirements**: INFRA-01, INFRA-02, INFRA-03, INFRA-04
**Success Criteria** (what must be TRUE):
  1. Docker container launches Stonefish simulator
  2. ROS2 nodes can publish and subscribe to topics
  3. X11 forwarding shows Stonefish visualization
  4. Single docker-compose command starts everything
**Research**: Likely (Stonefish Docker setup, ROS2 Humble container patterns)
**Research topics**: Stonefish 1.5+ installation, ROS2 Humble Docker base images, X11 forwarding in containers, Stonefish-ROS2 bridge
**Plans**: TBD

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
**Research**: Likely (Stonefish sensor topics, EKF implementation patterns)
**Research topics**: Stonefish DVL/IMU/depth sensor message types, C++ EKF libraries vs custom, ROS2 C++ node patterns
**Plans**: TBD

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
**Research**: Unlikely (standard PID, consumes EKF output)
**Plans**: TBD

Plans:
- [x] 03-01: Control package foundation (PID class, control node skeleton)
- [ ] 03-02: Depth and heading controllers
- [ ] 03-03: Velocity controller and launch file

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
**Research**: Unlikely (Python state machine, internal logic)
**Plans**: TBD

Plans:
- [ ] 04-01: TBD

### Phase 5: Demo
**Goal**: Full vertical slice — autonomous dive→waypoint→surface mission
**Depends on**: Phase 4
**Requirements**: DEMO-01, DEMO-02, DEMO-03, DEMO-04
**Success Criteria** (what must be TRUE):
  1. AUV dives from surface to 5m depth
  2. AUV navigates to at least one waypoint at depth
  3. AUV surfaces after waypoint sequence completes
  4. Entire mission runs without operator intervention
**Research**: Unlikely (integration of existing phases)
**Plans**: TBD

Plans:
- [ ] 05-01: TBD

## Progress

**Execution Order:**
Phases execute in numeric order: 1 → 2 → 3 → 4 → 5

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Infrastructure | 2/2 | Complete | 2026-01-17 |
| 2. Navigation | 4/4 | Complete | 2026-01-17 |
| 3. Control | 1/3 | In progress | - |
| 4. Planning | 0/TBD | Not started | - |
| 5. Demo | 0/TBD | Not started | - |
