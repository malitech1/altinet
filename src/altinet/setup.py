from setuptools import setup

package_name = 'altinet'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Altinet Maintainer',
    maintainer_email='maintainer@example.com',
    description='Altinet ROS 2 package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'minimal_node = altinet.nodes.minimal_node:main',
            'camera_node = altinet.nodes.camera_node:main',
            'face_detector_node = altinet.nodes.face_detector_node:main',
            'face_identifier_node = altinet.nodes.face_identifier_node:main',
=======
            'minimal_node = altinet.nodes.minimal_node:main'
 main
        ],
    },
)
