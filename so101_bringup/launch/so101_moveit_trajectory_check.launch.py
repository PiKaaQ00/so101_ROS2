from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    trajectory_topic_arg = DeclareLaunchArgument(
        'trajectory_topic',
        default_value='/display_planned_path',
    )
    max_step_rad_arg = DeclareLaunchArgument(
        'max_step_rad',
        default_value='0.20',
    )

    trajectory_topic = LaunchConfiguration('trajectory_topic')
    max_step_rad = LaunchConfiguration('max_step_rad')

    check_node = Node(
        package='so101_direct_driver',
        executable='moveit_trajectory_check_node',
        name='so101_moveit_trajectory_check',
        output='screen',
        parameters=[
            {
                'trajectory_topic': trajectory_topic,
                'max_step_rad': ParameterValue(max_step_rad, value_type=float),
            },
        ],
    )

    return LaunchDescription(
        [
            trajectory_topic_arg,
            max_step_rad_arg,
            check_node,
        ]
    )
