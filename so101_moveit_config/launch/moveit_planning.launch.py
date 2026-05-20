import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_file(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, 'r', encoding='utf-8') as file:
        return file.read()


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    moveit_pkg = get_package_share_directory('so101_moveit_config')
    description_pkg = get_package_share_directory('so101_description')

    display_arg = DeclareLaunchArgument('display', default_value='true')
    allow_execute_arg = DeclareLaunchArgument('allow_execute', default_value='false')
    use_fake_joint_states_arg = DeclareLaunchArgument('use_fake_joint_states', default_value='true')
    joint_states_topic_arg = DeclareLaunchArgument(
        'joint_states_topic',
        default_value='/joint_states',
        description='JointState topic used by MoveIt and robot_state_publisher',
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(description_pkg, 'urdf', 'so101_new_calib.urdf.xacro'),
    )

    display = LaunchConfiguration('display')
    allow_execute = LaunchConfiguration('allow_execute')
    use_fake_joint_states = LaunchConfiguration('use_fake_joint_states')
    joint_states_topic = LaunchConfiguration('joint_states_topic')
    model = LaunchConfiguration('model')

    robot_description = {
        'robot_description': ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name='xacro')]),
                    ' ',
                    model,
                    ' ',
                    'mode:=real ',
                    'joint_states_gui:=true',
                ]
            ),
            value_type=str,
        )
    }

    robot_description_semantic = {
        'robot_description_semantic': load_file('so101_moveit_config', 'config/so101.srdf')
    }

    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml('so101_moveit_config', 'config/kinematics.yaml')
    }

    joint_limits = {
        'robot_description_planning': load_yaml('so101_moveit_config', 'config/joint_limits.yaml')
    }

    initial_positions = load_yaml('so101_description', 'config/initial_positions.yaml')[
        'initial_positions'
    ]

    ompl_planning = load_yaml('so101_moveit_config', 'config/ompl_planning.yaml')
    moveit_controllers = load_yaml('so101_moveit_config', 'config/moveit_controllers.yaml')
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    trajectory_execution_parameters = {
        'trajectory_execution.allowed_execution_duration_scaling': 20.0,
        'trajectory_execution.allowed_goal_duration_margin': 60.0,
        'trajectory_execution.allowed_start_tolerance': 0.15,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            ompl_planning,
            moveit_controllers,
            planning_scene_monitor_parameters,
            trajectory_execution_parameters,
            {'allow_trajectory_execution': ParameterValue(allow_execute, value_type=bool)},
            {'use_sim_time': False},
        ],
        remappings=[
            ('joint_states', joint_states_topic),
            ('/joint_states', joint_states_topic),
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[
            ('joint_states', joint_states_topic),
            ('/joint_states', joint_states_topic),
        ],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[
            robot_description,
            {'zeros': initial_positions},
        ],
        condition=IfCondition(use_fake_joint_states),
    )

    rviz_config = os.path.join(moveit_pkg, 'rviz', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            ompl_planning,
        ],
        remappings=[
            ('joint_states', joint_states_topic),
            ('/joint_states', joint_states_topic),
        ],
        condition=IfCondition(display),
    )

    return LaunchDescription(
        [
            display_arg,
            allow_execute_arg,
            use_fake_joint_states_arg,
            joint_states_topic_arg,
            model_arg,
            robot_state_publisher,
            joint_state_publisher_gui,
            move_group_node,
            rviz_node,
        ]
    )
