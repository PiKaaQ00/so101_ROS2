import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vision_pkg = get_package_share_directory('so101_vision')

    video_device_arg = DeclareLaunchArgument('video_device', default_value='/dev/video0')
    image_width_arg = DeclareLaunchArgument('image_width', default_value='640')
    image_height_arg = DeclareLaunchArgument('image_height', default_value='480')
    framerate_arg = DeclareLaunchArgument('framerate', default_value='30.0')
    pixel_format_arg = DeclareLaunchArgument('pixel_format', default_value='mjpeg2rgb')

    parent_frame_arg = DeclareLaunchArgument('parent_frame', default_value='follower/gripper_link')
    camera_frame_arg = DeclareLaunchArgument('camera_frame', default_value='camera_link')
    optical_frame_arg = DeclareLaunchArgument('optical_frame', default_value='camera_optical_frame')
    camera_x_arg = DeclareLaunchArgument('camera_x', default_value='-0.005')
    camera_y_arg = DeclareLaunchArgument('camera_y', default_value='0.032')
    camera_z_arg = DeclareLaunchArgument('camera_z', default_value='-0.020')
    camera_roll_arg = DeclareLaunchArgument('camera_roll', default_value='0.0')
    camera_pitch_arg = DeclareLaunchArgument('camera_pitch', default_value='0.0')
    camera_yaw_arg = DeclareLaunchArgument('camera_yaw', default_value='1.5708')

    video_device = LaunchConfiguration('video_device')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate = LaunchConfiguration('framerate')
    pixel_format = LaunchConfiguration('pixel_format')

    parent_frame = LaunchConfiguration('parent_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    optical_frame = LaunchConfiguration('optical_frame')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            os.path.join(vision_pkg, 'config', 'wrist_usb_camera.yaml'),
            {
                'video_device': video_device,
                'frame_id': optical_frame,
                'image_width': ParameterValue(image_width, value_type=int),
                'image_height': ParameterValue(image_height, value_type=int),
                'framerate': ParameterValue(framerate, value_type=float),
                'pixel_format': pixel_format,
            },
        ],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
    )

    wrist_camera_tf = Node(
        package='so101_vision',
        executable='wrist_camera_tf_node',
        name='wrist_camera_tf_node',
        output='screen',
        parameters=[
            {
                'parent_frame': parent_frame,
                'camera_frame': camera_frame,
                'optical_frame': optical_frame,
                'x': ParameterValue(camera_x, value_type=float),
                'y': ParameterValue(camera_y, value_type=float),
                'z': ParameterValue(camera_z, value_type=float),
                'roll': ParameterValue(camera_roll, value_type=float),
                'pitch': ParameterValue(camera_pitch, value_type=float),
                'yaw': ParameterValue(camera_yaw, value_type=float),
            },
        ],
    )

    return LaunchDescription(
        [
            video_device_arg,
            image_width_arg,
            image_height_arg,
            framerate_arg,
            pixel_format_arg,
            parent_frame_arg,
            camera_frame_arg,
            optical_frame_arg,
            camera_x_arg,
            camera_y_arg,
            camera_z_arg,
            camera_roll_arg,
            camera_pitch_arg,
            camera_yaw_arg,
            camera_node,
            wrist_camera_tf,
        ]
    )
