# Requirements: Stonefish AUV Sim-to-Real Pipeline

**Defined:** 2026-01-17
**Core Value:** Full vertical slice — one complete mission (dive → waypoint → surface) with all layers working: EKF navigation, state machine, control, and Docker containerization.

## v1 Requirements (Complete)

Requirements for initial release. All mapped and validated.

### Navigation

- [x] **NAV-01**: EKF state estimator fuses DVL velocity measurements for position estimation
- [x] **NAV-02**: EKF state estimator fuses IMU data for orientation and angular velocity
- [x] **NAV-03**: EKF state estimator fuses depth sensor for vertical position
- [x] **NAV-04**: EKF provides filtered state estimate (position, velocity, orientation) to control system
- [x] **NAV-05**: EKF handles sensor dropouts gracefully (dead reckoning fallback)

### Control

- [x] **CTRL-01**: PID controller maintains commanded depth within ±0.5m tolerance
- [x] **CTRL-02**: PID controller maintains commanded heading within ±5° tolerance
- [x] **CTRL-03**: PID controller maintains commanded velocity within ±0.1m/s tolerance
- [x] **CTRL-04**: Control outputs map to Stonefish thruster commands
- [x] **CTRL-05**: Controllers are implemented in C++ for performance

### Planning

- [x] **PLAN-01**: Mission planner accepts list of waypoints (x, y, depth)
- [x] **PLAN-02**: Mission planner computes heading and distance to next waypoint
- [x] **PLAN-03**: Mission planner detects waypoint arrival within configurable radius
- [x] **PLAN-04**: Mission planner advances through waypoint sequence automatically

### State Machine

- [x] **SM-01**: State machine implements IDLE → DIVING → NAVIGATING → SURFACING → COMPLETE states
- [x] **SM-02**: State machine transitions on mission events (waypoint reached, depth achieved)
- [x] **SM-03**: State machine handles ERROR state with safe surfacing behavior
- [x] **SM-04**: State machine publishes current state for monitoring

### Demo Mission

- [x] **DEMO-01**: AUV dives from surface to commanded depth
- [x] **DEMO-02**: AUV navigates to at least one waypoint underwater
- [x] **DEMO-03**: AUV surfaces after completing waypoint sequence
- [x] **DEMO-04**: Complete mission runs autonomously without operator intervention

### Infrastructure

- [x] **INFRA-01**: Stonefish simulator runs in Docker container
- [x] **INFRA-02**: ROS2 nodes communicate via defined topic architecture
- [x] **INFRA-03**: Docker container supports X11 forwarding for visualization
- [x] **INFRA-04**: Single docker-compose command launches full simulation

## v2 Requirements (Active — USBL Navigation Demo)

Requirements for v2.0 milestone. Demonstrates delayed-state EKF fusion with USBL.

### Sensor Simulation

- [x] **SIM-01**: Truth generator publishes ground truth trajectory (lawnmower survey pattern)
- [x] **SIM-02**: IMU simulator adds realistic noise and bias drift to truth data
- [x] **SIM-03**: DVL simulator converts NED velocity to body frame with noise and dropouts
- [x] **SIM-04**: DVL simulator implements canyon dropout scenario (t=120s to t=150s)
- [x] **SIM-05**: USBL simulator publishes DELAYED position measurements (timestamp = measurement time)
- [x] **SIM-06**: USBL simulator adds range-dependent noise (max(2% * range, 0.3m))
- [x] **SIM-07**: USBL simulator implements outlier injection (5% probability, 5m offset)

### Delayed-State EKF

- [ ] **DEKF-01**: State buffer stores historical states with timestamps (500 entries, ~5s)
- [ ] **DEKF-02**: EKF uses 15-state vector (pos, vel, quat, gyro_bias, accel_bias)
- [ ] **DEKF-03**: EKF prediction step integrates IMU with bias removal
- [ ] **DEKF-04**: DVL update reduces velocity uncertainty using body-frame measurement
- [ ] **DEKF-05**: USBL update finds historical state at measurement timestamp
- [ ] **DEKF-06**: USBL update applies Mahalanobis gating for outlier rejection
- [ ] **DEKF-07**: USBL update repropagates state to current time after correction
- [ ] **DEKF-08**: Navigation node publishes filtered odometry at 50Hz

### Visualization & Metrics

- [ ] **VIZ-01**: Metrics logger records truth, estimate, error, covariance to CSV
- [ ] **VIZ-02**: Plotting script generates position error sawtooth plot
- [ ] **VIZ-03**: Plotting script generates trajectory comparison plot (truth vs estimate)
- [ ] **VIZ-04**: Plotting script generates covariance consistency plot (error within 3σ bounds)
- [ ] **VIZ-05**: RViz configuration shows truth path, estimated path, and USBL fixes
- [ ] **VIZ-06**: Complete launch file starts all nodes with configurable parameters

### Demo Scenarios

- [ ] **SCEN-01**: Nominal scenario achieves <2m final error, <5m max error
- [ ] **SCEN-02**: DVL dropout scenario shows error growth then USBL recovery
- [ ] **SCEN-03**: High outlier scenario rejects outliers without position jumps

## v3 Requirements (Deferred)

Tracked but not in current roadmap.

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
| Real Stonefish physics for v2 | Sensor simulators give precise control for demo |
| Unit tests for v2 | Focus on working demo first |
| C++ planning layer | Python pragmatic here, C++ reserved for core algorithms |
| Multiple docker-compose services | Single container sufficient for demo |
| Perception/computer vision | Listed as "desired" not "required" in JD |

## Traceability

Which phases cover which requirements. Updated by create-roadmap.

### v1 (Phases 1-5)

| Requirement | Phase | Status |
|-------------|-------|--------|
| INFRA-01 | Phase 1 | Complete |
| INFRA-02 | Phase 1 | Complete |
| INFRA-03 | Phase 1 | Complete |
| INFRA-04 | Phase 1 | Complete |
| NAV-01 | Phase 2 | Complete |
| NAV-02 | Phase 2 | Complete |
| NAV-03 | Phase 2 | Complete |
| NAV-04 | Phase 2 | Complete |
| NAV-05 | Phase 2 | Complete |
| CTRL-01 | Phase 3 | Complete |
| CTRL-02 | Phase 3 | Complete |
| CTRL-03 | Phase 3 | Complete |
| CTRL-04 | Phase 3 | Complete |
| CTRL-05 | Phase 3 | Complete |
| PLAN-01 | Phase 4 | Complete |
| PLAN-02 | Phase 4 | Complete |
| PLAN-03 | Phase 4 | Complete |
| PLAN-04 | Phase 4 | Complete |
| SM-01 | Phase 4 | Complete |
| SM-02 | Phase 4 | Complete |
| SM-03 | Phase 4 | Complete |
| SM-04 | Phase 4 | Complete |
| DEMO-01 | Phase 5 | Complete |
| DEMO-02 | Phase 5 | Complete |
| DEMO-03 | Phase 5 | Complete |
| DEMO-04 | Phase 5 | Complete |

### v2 (Phases 6-8)

| Requirement | Phase | Status |
|-------------|-------|--------|
| SIM-01 | Phase 6 | Complete |
| SIM-02 | Phase 6 | Complete |
| SIM-03 | Phase 6 | Complete |
| SIM-04 | Phase 6 | Complete |
| SIM-05 | Phase 6 | Complete |
| SIM-06 | Phase 6 | Complete |
| SIM-07 | Phase 6 | Complete |
| DEKF-01 | Phase 7 | Complete |
| DEKF-02 | Phase 7 | Complete |
| DEKF-03 | Phase 7 | Complete |
| DEKF-04 | Phase 7 | Complete |
| DEKF-05 | Phase 7 | Complete |
| DEKF-06 | Phase 7 | Complete |
| DEKF-07 | Phase 7 | Complete |
| DEKF-08 | Phase 7 | Complete |
| VIZ-01 | Phase 8 | Pending |
| VIZ-02 | Phase 8 | Pending |
| VIZ-03 | Phase 8 | Pending |
| VIZ-04 | Phase 8 | Pending |
| VIZ-05 | Phase 8 | Pending |
| VIZ-06 | Phase 8 | Pending |
| SCEN-01 | Phase 8 | Pending |
| SCEN-02 | Phase 8 | Pending |
| SCEN-03 | Phase 8 | Pending |

**Coverage:**
- v1 requirements: 26 total, 26 complete
- v2 requirements: 24 total, 15 complete, 9 pending
- Unmapped: 0 ✓

---
*Requirements defined: 2026-01-17*
*Last updated: 2026-01-21 after Phase 7 complete*
