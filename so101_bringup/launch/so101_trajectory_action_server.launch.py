import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bridge_pkg = get_package_share_directory('so101_ros2_bridge')
    description_pkg = get_package_share_directory('so101_description')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(description_pkg, 'urdf', 'so101_new_calib.urdf.xacro'),
    )
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyACM0')
    time_scale_arg = DeclareLaunchArgument('time_scale', default_value='5.0')
    max_step_rad_arg = DeclareLaunchArgument('max_step_rad', default_value='0.20')
    max_velocity_rad_s_arg = DeclareLaunchArgument('max_velocity_rad_s', default_value='0.25')
    command_rate_arg = DeclareLaunchArgument('command_rate', default_value='25.0')
    command_deadband_ticks_arg = DeclareLaunchArgument('command_deadband_ticks', default_value='2')

    model = LaunchConfiguration('model')
    port = LaunchConfiguration('port')
    time_scale = LaunchConfiguration('time_scale')
    max_step_rad = LaunchConfiguration('max_step_rad')
    max_velocity_rad_s = LaunchConfiguration('max_velocity_rad_s')
    command_rate = LaunchConfiguration('command_rate')
    command_deadband_ticks = LaunchConfiguration('command_deadband_ticks')

    follower_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_pkg, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'model': model,
            'mode': 'real',
            'type': 'follower',
        }.items(),
    )

    action_server = Node(
        package='so101_direct_driver',
        executable='follow_joint_trajectory_server_node',
        name='so101_follow_joint_trajectory_server',
        output='screen',
        parameters=[
            os.path.join(bridge_pkg, 'config', 'so101_direct_follower_readonly.yaml'),
            {
                'port': port,
                'time_scale': ParameterValue(time_scale, value_type=float),
                'max_step_rad': ParameterValue(max_step_rad, value_type=float),
                'max_velocity_rad_s': ParameterValue(max_velocity_rad_s, value_type=float),
                'command_rate': ParameterValue(command_rate, value_type=float),
                'command_deadband_ticks': ParameterValue(command_deadband_ticks, value_type=int),
            },
        ],
        remappings=[
            ('joint_states', '/follower/joint_states'),
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            port_arg,
            time_scale_arg,
            max_step_rad_arg,
            max_velocity_rad_s_arg,
            command_rate_arg,
            command_deadband_ticks_arg,
            follower_rsp,
            action_server,
        ]
    )
