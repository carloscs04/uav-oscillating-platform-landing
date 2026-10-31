from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Launch in simulation mode (true) or real hardware mode (false)'
    )

    use_sim = LaunchConfiguration('use_sim')

    mode_param = PythonExpression([
        "'SIMULATION' if '", use_sim, "'.lower() == 'true' else 'HARDWARE'"
    ])

    px4_mavlink_node = Node(
        package='px4_mavlink',
        executable='px4_node',
        name='px4_node',
        output='screen',
        parameters=[{
            'mode': mode_param
        }]
    )

    return LaunchDescription([
        use_sim_arg,
        px4_mavlink_node,
    ])