#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():

    # --- Configuration ---
    package_name = 'px4_offboard'  # Change this to your package name
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    # ---------------------

    # Path to your ROS 2 setup file
    ros_setup_path = f'/opt/ros/{ros_distro}/setup.bash'
    # Workspace overlay (install/setup.bash), so the new terminal can find this package
    ws_setup_path = os.path.join(os.path.dirname(get_package_prefix(package_name)), 'setup.bash')

    # Launch the main offboard controller node
    controller_node = Node(
        package=package_name,
        executable='velocity_offboard_control.py',
        name='velocity_offboard_control',
        output='screen',
        emulate_tty=True,  # This ensures Python's print() statements are shown
    )
    
    # Launch the custom teleop keyboard node in a new gnome-terminal
    teleop_node_process = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--',
            'bash', '-c',
            # Command to run in the new terminal:
            f'export DRONE_SIM_WINDOW=1; source {ros_setup_path} && source {ws_setup_path} && '
            f'echo "Terminal for Keyboard Teleop (Press Space to Arm)" && '
            f'ros2 run {package_name} teleop_keyboard.py; '
            'exec bash' # Keeps the terminal open after the script exits
        ],
        output='screen',
        shell=False
    )

    return LaunchDescription([
        controller_node,
        teleop_node_process
    ])