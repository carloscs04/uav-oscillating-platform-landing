from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        # 1. MicroXRCEAgent for Physical PX4 Controller (Serial connection)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyACM0', '-b', '921600'],
            output='screen'
        ),
        # 2. Hardware Camera Driver Node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='hardware_camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480]
            }]
        ),
        # 3. Vision Node
        Node(
            package='uav_opl_vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        # 4. Control Node
        Node(
            package='uav_opl_control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ])