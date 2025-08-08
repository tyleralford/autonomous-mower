"""
Teleoperation Launch File for Autonomous Mower

This launch file provides keyboard teleoperation control for the mower robot.
It launches a custom simple teleop node that publishes TwistStamped messages
to the differential drive controller.

Usage:
    ros2 launch mower_bringup teleop.launch.py
    
This will open a new gnome-terminal window for keyboard input.
Keep the teleop terminal window focused for keyboard control!
    
Controls:
    w - Move forward
    s - Move backward  
    a - Turn left
    d - Turn right
    q - Forward + left
    e - Forward + right
    z - Backward + left
    c - Backward + right
    x - Stop
    
    SPACE - Emergency stop
    ESC or Ctrl+C - Exit
    
Press Ctrl+C in the teleop terminal to stop.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Simple Teleoperation Node (Custom Implementation)
        Node(
            package='mower_bringup',
            executable='simple_teleop',
            name='simple_teleop',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            prefix='gnome-terminal --'
        )
    ])
