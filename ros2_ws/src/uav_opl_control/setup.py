from setuptools import setup

package_name = 'uav_opl_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@todo.todo',
    description='ROS 2 wrapper for UAV Oscillating Platform Landing control algorithms',
    license='MIT',
    entry_points={
        'console_scripts': [
            'control_node = uav_opl_control.control_node:main',
        ],
    },
)