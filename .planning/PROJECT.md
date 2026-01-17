# Stonefish AUV Sim-to-Real Pipeline

## What This Is

A containerized simulation pipeline demonstrating autonomous underwater vehicle capabilities using Stonefish simulator and ROS2. Built as a portfolio piece for an Orpheus Robotics Autonomy/GNC Engineer interview — designed to show production-grade code that maps directly to their tech stack.

## Core Value

**Full vertical slice:** One complete mission (dive → waypoint → surface) with all layers working — EKF navigation, state machine, control, and Docker containerization. Every piece must be defensible in a code walkthrough.

## Requirements

### Validated

(None yet — ship to validate)

### Active

- [ ] C++ EKF state estimator fusing DVL, IMU, depth sensors (GPS-denied capable)
- [ ] C++ PID controllers for depth, heading, and velocity
- [ ] Python mission planner with waypoint management
- [ ] Autonomous behavior state machine with error handling and fail-safes
- [ ] Docker containerized simulation environment with Stonefish
- [ ] ROS2 Humble integration with proper topic architecture
- [ ] Unit tests for EKF and control algorithms
- [ ] Basic CI/CD pipeline (GitHub Actions)
- [ ] Complete vertical slice demo: dive to depth, navigate to waypoint, surface

### Out of Scope

- USBL beacon integration — complex hardware dependency, not needed for demo
- Full lawnmower survey patterns — simple waypoint following sufficient for interview
- C++ planning layer — Python is pragmatic here, C++ reserved for core algorithms
- Multiple docker-compose services — single container sufficient for demo
- Perception/computer vision — listed as "desired" not "required" in JD
- Media/video generation — can record after implementation

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
| C++ for EKF/control, Python for planning | Matches real AUV stacks, shows C++ proficiency while maintaining velocity | — Pending |
| Add state machine layer | JD explicitly requires "autonomous state machine for decision-making" | — Pending |
| Single vertical slice over breadth | One working mission more impressive than many half-working features | — Pending |
| Skip USBL/lawnmower patterns | Not required for core demo, can discuss as "future work" | — Pending |

---
*Last updated: 2026-01-17 after initialization*
