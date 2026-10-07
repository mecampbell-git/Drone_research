#!/usr/bin/env python3

"""
PX4 Minimal Launch File
Launches PX4 SITL, MicroXRCE Agent, the Gazebo camera bridge and the camera monitor
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from string import Template
import glob
import os
import tempfile


def camera_bridge(context, model):
    """Start ros_gz_bridge with the config template filled in for the chosen world."""
    world = LaunchConfiguration('world').perform(context)
    template_path = os.path.join(
        get_package_share_directory('px4_bringup'), 'config', 'bridge_config.yaml')
    with open(template_path) as f:
        config = Template(f.read()).substitute(world=world, model=model)
    with tempfile.NamedTemporaryFile(
            'w', prefix=f'bridge_{world}_', suffix='.yaml', delete=False) as f:
        f.write(config)
    return [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        parameters=[{'config_file': f.name}],
        output='screen',
    )]


def generate_launch_description():

    # Define PX4 directory path
    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    qgc_path = os.path.expanduser('~/Downloads/QGroundControl-x86_64.AppImage')
    # PX4 airframe to run (make px4_sitl gz_<model>); Gazebo names the instance <model>_0
    model = 'my_x500_mono_cam'

    # Gazebo worlds shipped with PX4 (Tools/simulation/gz/worlds/<name>.sdf)
    worlds = sorted(
        os.path.splitext(os.path.basename(f))[0]
        for f in glob.glob(os.path.join(px4_dir, 'Tools/simulation/gz/worlds/*.sdf'))
    ) or None

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value='default',
        choices=worlds,
        description='Gazebo world to load (sets PX4_GZ_WORLD)'
    )

    return LaunchDescription([
        world_arg,

        # 1. Launch PX4 SITL with Gazebo in a new terminal
        ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--title=PX4 SITL',
                '--',
                'bash', '-c',
                [f'cd {px4_dir} && PX4_GZ_WORLD=', LaunchConfiguration('world'),
                 f' make px4_sitl gz_{model}; exec bash']
            ],
            output='screen',
            name='px4_sitl'
        ),
        
        # 2. Launch MicroXRCE Agent after 3 seconds
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal',
                        '--title=MicroXRCE Agent',
                        '--',
                        'bash', '-c',
                        'MicroXRCEAgent udp4 -p 8888; exec bash'
                    ],
                    output='screen',
                    name='microxrce_agent'
                )
            ]
        ),

        # 3. Allow arming without QGroundControl. The gz_x500 airframe (PX4 >= 1.16)
        #    sets NAV_DLL_ACT=2, which denies arming with "No connection to the GCS".
        #    Wait for commander (it starts after the airframe script), then set it once;
        #    PX4 saves it to parameters.bson so later runs keep it.
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'B={px4_dir}/build/px4_sitl_default/bin; '
                'for i in $(seq 1 600); do '
                '  $B/px4-listener vehicle_status -n 1 >/dev/null 2>&1 && break; sleep 1; '
                'done; '
                '$B/px4-param compare NAV_DLL_ACT 0 || '
                '{ $B/px4-param set NAV_DLL_ACT 0 && $B/px4-param save && echo "Set NAV_DLL_ACT=0"; }'
            ],
            output='screen',
            name='px4_allow_arm_without_gcs'
        ),

        # 4. Bridge the Gazebo camera to /camera/image_raw and /camera/camera_info.
        #    Gazebo topic names contain the world, so the config is rendered per launch.
        OpaqueFunction(function=camera_bridge, kwargs={'model': model}),

        # 5. Log the camera rate with a RELIABLE subscriber (BEST_EFFORT drops large frames)
        Node(
            package='px4_offboard',
            executable='camera_monitor.py',
            name='camera_monitor',
            output='screen',
            emulate_tty=True,
        ),

        # # Launch QGroundControl after 5 seconds
        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         ExecuteProcess(
        #             cmd=[qgc_path],
        #             output='screen',
        #             name='qgroundcontrol'
        #         )
        #     ]
        # ),

    ])