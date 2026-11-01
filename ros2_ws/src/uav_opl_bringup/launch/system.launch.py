import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    uav_mode = os.environ.get('UAV_MODE', 'HARDWARE')
    px4_dir = os.environ.get('PX4_DIR', os.path.expanduser('~/Tools/PX4-Autopilot'))
    
    # Absolute path to current workspace setup script
    ws_setup = os.path.abspath('install/setup.bash')

    # Environment isolation wrapper to prevent GTK/GSettings crashes
    clean_env_prefix = [
        'env',
        'XDG_DATA_DIRS=/usr/share/ubuntu:/usr/local/share:/usr/share',
        'GSETTINGS_SCHEMA_DIR=',
        'GTK_PATH=',
        'gnome-terminal'
    ]

    actions = []

    # 1. Terminal 1: Master Node Terminal (BOTH SIMULATION and HARDWARE)
    master_terminal_cmd = ExecuteProcess(
        cmd=clean_env_prefix + [
            '--title=System Master Console',
            '--',
            'bash', '-c', f'source /opt/ros/jazzy/setup.bash && source "{ws_setup}" && ros2 run system_master master_node; exec bash'
        ],
        output='screen'
    )
    actions.append(master_terminal_cmd)
    

    # 2. Terminal 2: Gazebo SITL (SIMULATION ONLY)
    if uav_mode == 'SIMULATION':
        gazebo_terminal_cmd = ExecuteProcess(
            cmd=clean_env_prefix + [
                '--title=PX4 SITL Gazebo',
                '--',
                'bash', '-c', f'cd "{px4_dir}" && make px4_sitl gz_x500; exec bash'
            ],
            output='screen'
        )
        actions.append(gazebo_terminal_cmd)

    
    return LaunchDescription(actions)