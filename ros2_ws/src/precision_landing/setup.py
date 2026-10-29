import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'precision_landing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carloscs',
    maintainer_email='carlosc@todo.todo',
    description='ROS 2 precision landing node for PX4 SITL',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Syntax: 'executable_name = package_name.python_file:main_function'
        ],
    },
)