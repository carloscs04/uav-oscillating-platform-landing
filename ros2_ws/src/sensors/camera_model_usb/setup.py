from setuptools import find_packages, setup

package_name = 'camera_model_usb'

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
    description='Hardware driver node publishing raw images from a USB camera device.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_model_usb.camera_node:main',
        ],
    },
)