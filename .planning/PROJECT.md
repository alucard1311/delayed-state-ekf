# Stonefish AUV Sim-to-Real Pipeline

## What This Is

A containerized simulation pipeline demonstrating autonomous underwater vehicle capabilities using Stonefish simulator and ROS2. Built as a portfolio piece for an Orpheus Robotics Autonomy/GNC Engineer interview — designed to show production-grade code that maps directly to their tech stack.

## Core Value

**Full vertical slice:** One complete mission (dive → waypoint → surface) with all layers working — EKF navigation, state machine, control, and Docker containerization. Every piece must be defensible in a code walkthrough.

## Current Milestone: v2.0 USBL Navigation Demo

**Goal:** Demonstrate delayed-state EKF fusion with USBL — the key technique for real underwater navigation where acoustic positioning has multi-second latency.

**Target features:**
- Sensor simulators (truth, IMU, DVL, USBL with realistic delays/noise)
- 15-state delayed-state EKF with state buffer and repropagation
- USBL delayed measurement update with Mahalanobis outlier rejection
- DVL dropout handling (canyon scenario)
- Metrics logging and publication-quality plots
- RViz visualization of truth vs estimate

## Requirements

### Validated

- ✓ C++ EKF state estimator fusing DVL, IMU, depth sensors — v1.0
- ✓ C++ PID controllers for depth, heading, and velocity — v1.0
- ✓ Python mission planner with waypoint management — v1.0
- ✓ Autonomous behavior state machine with error handling — v1.0
- ✓ Docker containerized simulation environment with Stonefish — v1.0
- ✓ ROS2 Humble integration with proper topic architecture — v1.0

### Active

- [ ] Truth generator node with lawnmower survey pattern
- [ ] IMU simulator with realistic noise and bias drift
- [ ] DVL simulator with body-frame velocity, dropouts, canyon scenario
- [ ] USBL simulator with delayed timestamps, range-dependent noise, outliers
- [ ] State buffer for historical state/IMU storage
- [ ] 15-state delayed-state EKF (pos, vel, quat, gyro_bias, accel_bias)
- [ ] USBL delayed measurement update with repropagation
- [ ] Mahalanobis gating for outlier rejection
- [ ] Metrics logger node (CSV output)
- [ ] Python plotting script (sawtooth error, trajectory, covariance bounds)
- [ ] RViz configuration for navigation visualization

### Out of Scope

- C++ planning layer — Python is pragmatic here, C++ reserved for core algorithms
- Multiple docker-compose services — single container sufficient for demo
- Perception/computer vision — listed as "desired" not "required" in JD
- Real Stonefish physics integration — pure simulation for USBL demo
- Unit tests for this milestone — focus on working demo first

## Context

**Target:** Orpheus Robotics Autonomy/GNC Engineer role ($130-170k, Southern MA)
- Cofounder interview with code walkthrough
- Timeline: Within one week
- They build deep-ocean AUVs for offshore wind, seabed minerals, subsea infrastructure

**JD alignment (what matters to them):**
- Sensor fusion, EKF, dead reckoning in GPS-denied environments
- ROS/ROS2 in C++ and Python
- Containerized simulation before field testing
- State machine for autonomous decision-making and reliability
- Error handling, health monitoring, fail-safes
- Unit tests and CI/CD
- Modular, well-documented GitHub repos

**Differentiator from "school project":**
- State machine shows production thinking (not just happy-path)
- C++ core shows performance awareness
- Docker shows deployment maturity
- Tests show professional rigor

## Constraints

- **Timeline**: One week — ruthless scope control required
- **Tech stack**: Stonefish 1.5+, ROS2 Humble, C++17, Python 3.10+, Docker
- **Language split**: C++ for EKF and control (performance-critical), Python for planning (velocity)
- **Demo environment**: Must run on standard Linux with Docker, X11 forwarding for visualization

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| C++ for EKF/control, Python for planning | Matches real AUV stacks, shows C++ proficiency while maintaining velocity | ✓ Good |
| Add state machine layer | JD explicitly requires "autonomous state machine for decision-making" | ✓ Good |
| Single vertical slice over breadth | One working mission more impressive than many half-working features | ✓ Good |
| Skip USBL/lawnmower patterns (v1.0) | Not required for core demo, can discuss as "future work" | ✓ Good |
| Delayed-state EKF for USBL (v2.0) | Shows understanding of real underwater nav challenges — acoustic latency | — Pending |
| Standalone usbl_navigation package (v2.0) | Clean separation from v1.0 code, focused demo | — Pending |
| Sensor simulators instead of Stonefish (v2.0) | Faster iteration, precise control over test scenarios | — Pending |
| 15-state with bias estimation (v2.0) | Production-grade INS, shows depth of understanding | — Pending |

---
*Last updated: 2026-01-20 after v2.0 milestone start*
