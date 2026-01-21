# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-01-20)

**Core value:** Delayed-state EKF fusion with USBL — the key technique for real underwater navigation
**Current focus:** Phase 6 — Sensor Foundation

## Current Position

Phase: 6 of 8 (Sensor Foundation)
Plan: 1 of 3 in current phase
Status: In progress
Last activity: 2026-01-21 — Completed 06-01-PLAN.md

Progress: ██████████████████░░░░░░░░░░░░ 64% (14 of 22 plans)

## Performance Metrics

**Velocity:**
- Total plans completed: 14 (13 v1.0 + 1 v2.0)
- Average duration: ~7min
- Total execution time: ~96 min

**By Phase:**

| Phase | Milestone | Plans | Status |
|-------|-----------|-------|--------|
| 01-infrastructure | v1.0 | 2/2 | Complete |
| 02-navigation | v1.0 | 4/4 | Complete |
| 03-control | v1.0 | 3/3 | Complete |
| 04-planning | v1.0 | 3/3 | Complete |
| 05-demo | v1.0 | 1/1 | Complete |
| 06-sensor-foundation | v2.0 | 1/3 | In progress |
| 07-navigation-filter | v2.0 | 0/3 | Not started |
| 08-demo-visualization | v2.0 | 0/3 | Not started |

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

### Pending Todos

None.

### Blockers/Concerns

None.

## Session Continuity

Last session: 2026-01-21
Stopped at: Completed 06-01-PLAN.md

**v1.0 Status:**
- All 5 phases complete (13 plans)
- Autonomous dive→waypoint→surface mission verified

**Phase 6 Status:**
- 06-01: Package scaffolding + truth generator - COMPLETE
- 06-02: IMU + DVL simulators - Pending
- 06-03: USBL simulator with delayed timestamps - Pending

**Next action:** Execute 06-02-PLAN.md

Resume file: None
