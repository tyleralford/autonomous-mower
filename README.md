# Autonomous Mower Project

This repository contains the ROS 2 software stack for an autonomous mower robot.

## Phase 1: Foundational Simulation and Manual Control ✅ **100% COMPLETE**

Phase 1 focuses on creating a high-fidelity digital twin of the mower within a simulated environment for development and testing.

### **Current Status (August 7, 2025)**
- ✅ **Complete Robot Description**: Modular URDF/XACRO with realistic physics
- ✅ **Gazebo Simulation**: Lawn environment with DART physics engine  
- ✅ **ros2_control Integration**: All controllers operational (drive, reel, joint states)
- ✅ **Clock Synchronization**: Gazebo-ROS2 timing fully synchronized
- ✅ **Transform Pipeline**: Complete TF tree with dynamic joint transforms
- ✅ **Movement Control**: Robot responds to velocity commands with accurate odometry

### **Quick Start**

#### Prerequisites
- ROS 2 Jazzy
- Gazebo Harmonic  
- ros2_control and related packages
- teleop_twist_keyboard package

#### Build and Launch
```bash
# Clone and build workspace
cd mower_ws
colcon build
source install/setup.bash

# Launch complete simulation
ros2 launch mower_bringup sim.launch.py

# In new terminal: Launch teleoperation (for keyboard control)
ros2 launch mower_bringup teleop.launch.py
```

#### Manual Control
```bash
# Use keyboard in teleop terminal:
# w - Forward    s - Backward     a - Turn left    d - Turn right
# q - Fwd+left   e - Fwd+right    z - Back+left    c - Back+right
# x - Stop       SPACE - Emergency stop           ESC - Exit
```

#### Test Robot Movement
```bash
# Send direct movement command (alternative to teleop)
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.5}, angular: {z: 0.5}}}'

# Test reel controller
ros2 topic pub --once /reel_controller/commands std_msgs/msg/Float64MultiArray \
  '{data: [2.0]}'  # Reel velocity in rad/s

# Monitor system status
ros2 topic echo /joint_states
ros2 control list_controllers
```

### **System Status: Phase 1 Complete ✅**

## Project Structure

- `mower_description/` - Robot URDF/XACRO models and visualization
- `mower_simulation/` - Gazebo worlds and simulation assets  
- `mower_control/` - ros2_control configuration
- `mower_bringup/` - Launch files and system integration

## Phase 2: Sensor Fusion and State Estimation

Phase 2 adds simulated sensors (IMU, GPS) and a dual-EKF pipeline using robot_localization, plus a simulation-only GPS heading republisher.

### Build and Launch
```bash
cd mower_ws
colcon build
source install/setup.bash

# Launch simulation + bridges + EKFs + navsat_transform + heading node
ros2 launch mower_bringup sim.launch.py
```

### Inspect Sensor Data
- IMU (orientation, angular velocity, linear acceleration)
  - `ros2 topic hz /imu`
  - `ros2 topic echo /imu`
- GPS fix (lat, lon, alt)
  - `ros2 topic hz /gps/fix`
  - `ros2 topic echo /gps/fix`
- Simulated GPS heading (orientation-only Imu)
  - `ros2 topic hz /gps/heading`
  - `ros2 topic echo /gps/heading`

### State Estimation Outputs
- NavSat transform odometry (map frame XY from GPS)
  - `ros2 topic echo /odometry/gps`
- Local EKF (odom frame): smooth but drifting
  - `ros2 topic echo /odometry/filtered/local`
- Global EKF (map frame): drift-corrected
  - `ros2 topic echo /odometry/filtered/global`

### Visualize in RViz
Option A (load saved config):
- Open RViz and load `src/mower_localization/rviz/ekf_test.rviz`.

Option B (manual):
- Fixed Frame: set to `odom`.
- Add Displays:
  - TF
  - Odometry: `/diff_drive_controller/odom` (Wheel Odom)
  - Odometry: `/odometry/gps` (GPS Odom)
  - Odometry: `/odometry/filtered/local` (Local EKF)
  - Odometry: `/odometry/filtered/global` (Global EKF)

### TF Frames Diagram (optional)
```bash
ros2 run tf2_tools view_frames.py
```
This will generate a `frames.pdf` showing `map -> odom -> base_link`.

## Phase 3: Mapping & Navigation (In Progress)

Phase 3 adds zone recording (boundary / keep-out / travel areas), static map generation, navigation guard supervision, and Nav2 path planning & execution in the `map -> odom -> base_link` frame chain (runtime georeferencing removed for simplicity).

### Key Components
- Recorder service: `/mower/manage_recording` (`ManageRecording.srv`) to capture polygon vertices from filtered odometry.
- Map generator: Builds `map.pgm` + `map.yaml` from recorded CSVs (auto-triggered on STOP).
- Navigation guard: `map_bounds_guard` validates map readiness, pose freshness, boundary safety, and sequences Nav2 lifecycle (STARTUP → ACTIVE; pauses on boundary violations).
- Nav2 stack: Planner (Smac), Controller (DWB), Behavior Tree Navigator, Lifecycle Manager.
- Velocity bridge: `cmd_vel_stamper` (ensures `TwistStamped` for `diff_drive_controller`).

### Recording Workflow
1. Launch simulation & localization (also provides recorder):
```bash
ros2 launch mower_bringup sim.launch.py
```
2. Start boundary recording (action=0 START, area_type=0 BOUNDARY):
```bash
ros2 service call /mower/manage_recording mower_msgs/srv/ManageRecording '{action: 0, area_type: 0, filename: "maps/boundary.csv"}'
```
3. Drive the robot around the full perimeter (closed loop). Stop recording:
```bash
ros2 service call /mower/manage_recording mower_msgs/srv/ManageRecording '{action: 1, area_type: 0, filename: ""}'
```
4. (Optional) Record keep-out zones (area_type=1) and travel zones (area_type=2) using distinct filenames (e.g., `maps/keepout1.csv`, `maps/travel1.csv`). Each: START → drive polygon → STOP.
5. After each STOP the system regenerates `maps/map.yaml` and `maps/map.pgm` using all CSVs present.

### Launch Navigation
If simulation bringup did not already start navigation, launch Nav2 + guard explicitly:
```bash
ros2 launch mower_navigation navigation.launch.py boundary_buffer_m:=0.2 auto_start:=true verbose:=false
```

### Guard Status Monitoring
The guard publishes `mower_msgs/NavStatus` on `/mower/nav_status`.
- `waiting_for_map` – Map files not yet readable.
- `waiting_for_pose` – No recent filtered odom pose.
- `robot_outside_map` – Robot outside strict boundary; Nav2 paused.
- `starting_nav` – Lifecycle STARTUP / RESUME sequencing in progress.
- `active` – Nav2 assumed active; goals may be sent.

Echo status:
```bash
ros2 topic echo /mower/nav_status
```

### Send a Navigation Goal
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}'
```

### Map Files
Located under `maps/`:
- `boundary*.csv`, `keepout*.csv`, `travel*.csv`
- `map.pgm`, `map.yaml`
- `datum.yaml` (first captured GPS fix; kept for reproducibility though runtime no longer uses a UTM frame).

### Exposed Navigation Launch Parameters
```text
boundary_buffer_m  Re-entry interior margin (meters) after out-of-bounds (default 0.2)
auto_start         If true, guard sequences Nav2 lifecycle automatically (default true)
verbose            Extra guard logging (default false)
```

### Troubleshooting
- No `active` state: Check map existence (`ls maps/`), ensure boundary recorded.
- Immediate `robot_outside_map`: Boundary polygon likely incomplete (needs closed loop) or robot spawned outside perimeter.
- No motion after goal: Confirm `cmd_vel_stamper` running and `/diff_drive_controller/cmd_vel` receiving `TwistStamped` messages.
- Map not updating: Ensure STOP action invoked for each zone; filenames unique; check logs for generation errors.

### Next Steps
- Add real-world sensor integration (future phases) and refine guard to query actual lifecycle states instead of assumption timers.

