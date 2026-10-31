from setuptools import find_packages, setup

package_name = 'px4_mavlink'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAV Team',
    maintainer_email='dev@uav-opl.org',
    description='MAVLink bridge node interfacing PX4 flight controller with ROS 2 topics.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_node = px4_mavlink.px4_node:main',
        ],
    },
)