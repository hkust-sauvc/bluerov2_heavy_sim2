#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    stereo_cam_description_dir = get_package_share_directory('my_stereo_cam')
    cam_file = os.path.join(stereo_cam_description_dir, 'worlds', 'stereo.world')
    print("SDF file path:", cam_file)
    assert os.path.exists(cam_file), "SDF file does not exist"
    # sim_left_ini = os.path.join(orca_bringup_dir, 'cfg', 'sim_left.ini')
    # sim_right_ini = os.path.join(orca_bringup_dir, 'cfg', 'sim_right.ini')
    return LaunchDescription([
    

        # Launch Gazebo Sim
        # gz must be on the $PATH
        # libArduPilotPlugin.so must be on the GZ_SIM_SYSTEM_PLUGIN_PATH
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', cam_file],
            output='screen',
            # condition=IfCondition(LaunchConfiguration('gzclient')),
        ),

        # Launch Gazebo Sim server-only
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', '-s', cam_file],
            output='screen',
            # condition=UnlessCondition(LaunchConfiguration('gzclient')),
        ),

        # Get images from Gazebo Sim to ROS
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['stereo_left', 'stereo_right'],
            output='screen',
        ),

     
    ])
