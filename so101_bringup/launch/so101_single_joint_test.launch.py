# Copyright 2025 nimiCurtis
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_pkg = get_package_share_directory('so101_bringup')
    bridge_pkg = get_package_share_directory('so101_ros2_bridge')
    description_pkg = get_package_share_directory('so101_description')

    display_config_arg = DeclareLaunchArgument(
        'display_config',
        default_value=os.path.join(bringup_pkg, 'rviz', 'robot_display_with_cameras.rviz'),
    )
    display_arg = DeclareLaunchArgument('display', default_value='false')
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(description_pkg, 'urdf', 'so101_new_calib.urdf.xacro'),
    )
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyACM0')
    joint_arg = DeclareLaunchArgument('joint', default_value='gripper')
    delta_rad_arg = DeclareLaunchArgument('delta_rad', default_value='0.01')
    enable_write_arg = DeclareLaunchArgument('enable_write', default_value='false')
    max_abs_delta_rad_arg = DeclareLaunchArgument('max_abs_delta_rad', default_value='0.05')

    model = LaunchConfiguration('model')
    display_config = LaunchConfiguration('display_config')
    display = LaunchConfiguration('display')
    port = LaunchConfiguration('port')
    joint = LaunchConfiguration('joint')
    delta_rad = LaunchConfiguration('delta_rad')
    enable_write = LaunchConfiguration('enable_write')
    max_abs_delta_rad = LaunchConfiguration('max_abs_delta_rad')

    follower_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_pkg, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'model': model,
            'mode': 'real',
            'type': 'follower',
        }.items(),
    )

    write_test_node = Node(
        package='so101_direct_driver',
        executable='single_joint_write_test_node',
        name='so101_single_joint_write_test',
        namespace='follower',
        output='screen',
        parameters=[
            os.path.join(bridge_pkg, 'config', 'so101_direct_follower_readonly.yaml'),
            {
                'port': port,
                'joint': joint,
                'delta_rad': ParameterValue(delta_rad, value_type=float),
                'enable_write': ParameterValue(enable_write, value_type=bool),
                'max_abs_delta_rad': ParameterValue(max_abs_delta_rad, value_type=float),
            },
        ],
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_pkg, 'launch', 'display.launch.py')),
        launch_arguments={
            'joint_states_gui': 'false',
            'display_config': display_config,
        }.items(),
    )

    delayed_display = TimerAction(
        period=3.0, actions=[display_launch], condition=IfCondition(display)
    )

    return LaunchDescription(
        [
            display_config_arg,
            display_arg,
            model_arg,
            port_arg,
            joint_arg,
            delta_rad_arg,
            enable_write_arg,
            max_abs_delta_rad_arg,
            follower_rsp,
            write_test_node,
            delayed_display,
        ]
    )
