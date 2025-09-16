from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'altinet'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('altinet/launch/*.py')),
        ('share/' + package_name + '/config', glob('altinet/config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Altinet Maintainer',
    maintainer_email='maintainer@example.com',
    description='Altinet perception stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = altinet.nodes.camera_node:main',
            'detector_node = altinet.nodes.detector_node:main',
            'tracker_node = altinet.nodes.tracker_node:main',
            'event_manager_node = altinet.nodes.event_manager_node:main',
            'lighting_control_node = altinet.nodes.lighting_control_node:main',
            'ros2_django_bridge_node = altinet.nodes.ros2_django_bridge_node:main',
        ],
    },
)
