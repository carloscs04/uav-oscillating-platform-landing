from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        # 1. MicroXRCEAgent for Gazebo PX4 SITL (UDP port 8888)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen'
        ),
        # 2. Vision Node
        Node(
            package='uav_opl_vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # 3. Control Node
        Node(
            package='uav_opl_control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])