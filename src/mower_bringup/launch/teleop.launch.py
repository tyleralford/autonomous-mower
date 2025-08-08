"""
Teleoperation Launch File for Autonomous Mower

This launch file provides keyboard teleoperation control for the mower robot.
It launches the teleop_twist_keyboard node and properly maps the command
velocity topic to the differential drive controller.

Usage:
    ros2 launch mower_bringup teleop.launch.py
    
Controls:
    i - Move forward
    k - Stop  
    j - Turn left
    l - Turn right
    u - Forward + left
    o - Forward + right
    m - Backward + left
    . - Backward + right
    , - Move backward
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
        
        # Keyboard Teleoperation Node
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                # Map teleop output to diff_drive_controller input
                ('cmd_vel', '/diff_drive_controller/cmd_vel')
            ],
            prefix='xterm -e',  # Run in separate terminal window
        )
    ])
