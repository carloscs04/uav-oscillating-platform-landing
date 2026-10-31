from setuptools import find_packages, setup

package_name = 'distance_sensor'

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
    description='Driver node publishing altitude data from rangefinders (Lidar/Sonar/TFmini).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rangefinder_node = distance_sensor.rangefinder_node:main',
        ],
    },
)