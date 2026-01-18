# Summary: 01-02 ROS2 Workspace and Stonefish Integration

**Status:** Complete
**Completed:** 2026-01-17

## What Was Built

ROS2 package `auv_sim` with Stonefish simulation integration:

- **Package structure:** `src/auv_sim/` with launch/, config/, description/ directories
- **Stonefish scenario:** `auv.xml` defining AUV with sensors and actuators
- **Launch file:** `simulation.launch.py` to start Stonefish simulator
- **ROS2 integration:** Sensors publish to topics, actuators subscribe to commands

## Key Files

| File | Purpose |
|------|---------|
| `src/auv_sim/package.xml` | ROS2 package manifest |
| `src/auv_sim/CMakeLists.txt` | Build configuration |
| `src/auv_sim/description/auv.xml` | Stonefish scenario with AUV, sensors, actuators |
| `src/auv_sim/launch/simulation.launch.py` | Launch file for simulation |
| `src/auv_sim/config/simulation.yaml` | Simulation parameters |

## AUV Configuration

**Physical:**
- Cylindrical hull (1.5m length, 0.15m radius) with bow/stern caps
- Material: AUV_Composite (density 950 kg/m³) for positive buoyancy

**Sensors (with ROS2 topics):**
- IMU @ 100Hz → `/auv/imu`
- DVL @ 10Hz → `/auv/dvl`
- Pressure @ 10Hz → `/auv/pressure`
- Odometry @ 50Hz → `/auv/odometry` (ground truth)

**Actuators (with ROS2 topics):**
- Surge thruster → `/auv/thruster/surge`
- Sway thruster → `/auv/thruster/sway`
- Heave thruster → `/auv/thruster/heave`
- Yaw bow → `/auv/thruster/yaw_bow`
- Yaw stern → `/auv/thruster/yaw_stern`

## Decisions Made

| Decision | Rationale |
|----------|-----------|
| AUV_Composite material (950 kg/m³) | Provides positive buoyancy (lighter than water at 1025 kg/m³) |
| Push-type actuators | Simple virtual force actuators, no mesh needed |
| Sensors added in Phase 1 | User proactively added for Phase 2 EKF readiness |
| GCC-13 with C++20 | Required by latest Stonefish for `<format>` header |

## Issues Resolved

1. **Docker ARG scope:** ARG before FROM not available after - fixed by redeclaring
2. **Stonefish version:** v1.5 tag incompatible with stonefish_ros2 - switched to main branch
3. **C++20 requirement:** Stonefish main needs C++20 `<format>` - added GCC-13
4. **ROS2 Humble compatibility:** Time arithmetic API changed - added sed patches
5. **AUV sinking:** Aluminium too dense - created AUV_Composite material
6. **No ROS2 topics:** Missing `<ros_publisher>`/`<ros_subscriber>` elements - added to XML

## Verification

- [x] Docker container builds successfully
- [x] Stonefish visualization window appears
- [x] AUV floats (positive buoyancy)
- [x] ROS2 topics visible (`ros2 topic list`)
- [x] Human verified simulation runs correctly

## How to Run

```bash
# On host
xhost +local:docker
./scripts/docker-run.sh

# Inside container
colcon build --packages-select auv_sim
source install/setup.bash
ros2 launch auv_sim simulation.launch.py
```
