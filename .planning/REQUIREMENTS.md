# Requirements: Stonefish AUV Sim-to-Real Pipeline

**Defined:** 2026-01-17
**Core Value:** Full vertical slice — one complete mission (dive → waypoint → surface) with all layers working: EKF navigation, state machine, control, and Docker containerization.

## v1 Requirements

Requirements for initial release. Each maps to roadmap phases.

### Navigation

- [ ] **NAV-01**: EKF state estimator fuses DVL velocity measurements for position estimation
- [ ] **NAV-02**: EKF state estimator fuses IMU data for orientation and angular velocity
- [ ] **NAV-03**: EKF state estimator fuses depth sensor for vertical position
- [ ] **NAV-04**: EKF provides filtered state estimate (position, velocity, orientation) to control system
- [ ] **NAV-05**: EKF handles sensor dropouts gracefully (dead reckoning fallback)

### Control

- [ ] **CTRL-01**: PID controller maintains commanded depth within ±0.5m tolerance
- [ ] **CTRL-02**: PID controller maintains commanded heading within ±5° tolerance
- [ ] **CTRL-03**: PID controller maintains commanded velocity within ±0.1m/s tolerance
- [ ] **CTRL-04**: Control outputs map to Stonefish thruster commands
- [ ] **CTRL-05**: Controllers are implemented in C++ for performance

### Planning

- [ ] **PLAN-01**: Mission planner accepts list of waypoints (x, y, depth)
- [ ] **PLAN-02**: Mission planner computes heading and distance to next waypoint
- [ ] **PLAN-03**: Mission planner detects waypoint arrival within configurable radius
- [ ] **PLAN-04**: Mission planner advances through waypoint sequence automatically

### State Machine

- [ ] **SM-01**: State machine implements IDLE → DIVING → NAVIGATING → SURFACING → COMPLETE states
- [ ] **SM-02**: State machine transitions on mission events (waypoint reached, depth achieved)
- [ ] **SM-03**: State machine handles ERROR state with safe surfacing behavior
- [ ] **SM-04**: State machine publishes current state for monitoring

### Demo Mission

- [ ] **DEMO-01**: AUV dives from surface to commanded depth
- [ ] **DEMO-02**: AUV navigates to at least one waypoint underwater
- [ ] **DEMO-03**: AUV surfaces after completing waypoint sequence
- [ ] **DEMO-04**: Complete mission runs autonomously without operator intervention

### Infrastructure

- [ ] **INFRA-01**: Stonefish simulator runs in Docker container
- [ ] **INFRA-02**: ROS2 nodes communicate via defined topic architecture
- [ ] **INFRA-03**: Docker container supports X11 forwarding for visualization
- [ ] **INFRA-04**: Single docker-compose command launches full simulation

## v2 Requirements

Deferred to future release. Tracked but not in current roadmap.

### Testing

- **TEST-01**: Unit tests for EKF prediction and update steps
- **TEST-02**: Unit tests for PID controller behavior
- **TEST-03**: Integration tests for sensor fusion pipeline

### CI/CD

- **CICD-01**: GitHub Actions runs build on push
- **CICD-02**: GitHub Actions runs unit tests on push
- **CICD-03**: GitHub Actions builds Docker image

## Out of Scope

Explicitly excluded. Documented to prevent scope creep.

| Feature | Reason |
|---------|--------|
| USBL beacon integration | Complex hardware dependency, not needed for demo |
| Lawnmower survey patterns | Simple waypoint following sufficient for interview |
| C++ planning layer | Python pragmatic here, C++ reserved for core algorithms |
| Multiple docker-compose services | Single container sufficient for demo |
| Perception/computer vision | Listed as "desired" not "required" in JD |
| Media/video generation | Can record after implementation |

## Traceability

Which phases cover which requirements. Updated by create-roadmap.

| Requirement | Phase | Status |
|-------------|-------|--------|
| INFRA-01 | Phase 1 | Pending |
| INFRA-02 | Phase 1 | Pending |
| INFRA-03 | Phase 1 | Pending |
| INFRA-04 | Phase 1 | Pending |
| NAV-01 | Phase 2 | Pending |
| NAV-02 | Phase 2 | Pending |
| NAV-03 | Phase 2 | Pending |
| NAV-04 | Phase 2 | Pending |
| NAV-05 | Phase 2 | Pending |
| CTRL-01 | Phase 3 | Pending |
| CTRL-02 | Phase 3 | Pending |
| CTRL-03 | Phase 3 | Pending |
| CTRL-04 | Phase 3 | Pending |
| CTRL-05 | Phase 3 | Pending |
| PLAN-01 | Phase 4 | Pending |
| PLAN-02 | Phase 4 | Pending |
| PLAN-03 | Phase 4 | Pending |
| PLAN-04 | Phase 4 | Pending |
| SM-01 | Phase 4 | Pending |
| SM-02 | Phase 4 | Pending |
| SM-03 | Phase 4 | Pending |
| SM-04 | Phase 4 | Pending |
| DEMO-01 | Phase 5 | Pending |
| DEMO-02 | Phase 5 | Pending |
| DEMO-03 | Phase 5 | Pending |
| DEMO-04 | Phase 5 | Pending |

**Coverage:**
- v1 requirements: 26 total
- Mapped to phases: 26
- Unmapped: 0 ✓

---
*Requirements defined: 2026-01-17*
*Last updated: 2026-01-17 after initial definition*
