# Autonomous Mower Project

This repository contains the ROS 2 software stack for an autonomous mower robot.

## Phase 1: Foundational Simulation and Manual Control ✅ **97% COMPLETE**

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
# i - Forward    j - Turn left    l - Turn right
# k - Stop       u - Forward+left o - Forward+right  
# , - Backward   m - Back+left    . - Back+right
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
