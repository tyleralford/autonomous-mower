#!/bin/bash

# Test script to validate teleoperation functionality
# This script checks the teleop system without requiring interactive input

echo "=== Autonomous Mower Teleoperation Validation ==="
echo

# Source the workspace
source install/setup.bash

echo "✅ Checking if simulation is running..."
if ros2 topic list | grep -q "/diff_drive_controller/cmd_vel"; then
    echo "✅ Differential drive controller topic found"
else
    echo "❌ Simulation not running. Please start simulation first:"
    echo "   ros2 launch mower_bringup sim.launch.py"
    exit 1
fi

echo
echo "✅ Testing teleop launch file configuration..."
if ros2 launch mower_bringup teleop.launch.py --print > /dev/null 2>&1; then
    echo "✅ Teleop launch file is valid"
else
    echo "❌ Teleop launch file has errors"
    exit 1
fi

echo
echo "✅ Testing topic publication capability..."
echo "Publishing test movement command..."
timeout 2 ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "
{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'base_link'
  },
  twist: {
    linear: {x: 0.1, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}
  }
}" > /dev/null 2>&1

if [ $? -eq 0 ]; then
    echo "✅ Successfully published test movement command"
else
    echo "❌ Failed to publish movement command"
    exit 1
fi

echo
echo "✅ Checking odometry response..."
if timeout 2 ros2 topic echo /diff_drive_controller/odom --once > /dev/null 2>&1; then
    echo "✅ Odometry data available"
else
    echo "❌ No odometry data received"
    exit 1
fi

echo
echo "🎉 TELEOPERATION SYSTEM VALIDATION COMPLETE!"
echo
echo "To use manual control:"
echo "1. Make sure simulation is running:"
echo "   ros2 launch mower_bringup sim.launch.py"
echo
echo "2. In a new terminal, launch teleoperation:"
echo "   ros2 launch mower_bringup teleop.launch.py"
echo
echo "3. Use these controls in the teleop terminal:"
echo "   w - Forward    s - Backward"
echo "   a - Turn left  d - Turn right"
echo "   q - Fwd+left   e - Fwd+right"
echo "   z - Back+left  c - Back+right"
echo "   x - Stop       SPACE - Emergency stop"
echo "   ESC - Exit"
echo
echo "The teleoperation system is ready for manual control! ✅"
