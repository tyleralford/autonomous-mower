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
