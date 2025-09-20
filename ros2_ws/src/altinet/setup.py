from glob import glob
from pathlib import Path

from setuptools import find_packages, setup
from setuptools.command.develop import develop as _DevelopCommand


class DevelopCommand(_DevelopCommand):
    """Custom develop command that tolerates ``--editable``."""

    user_options = list(_DevelopCommand.user_options)
    if not any(option[0].startswith('editable') for option in user_options):
        user_options.append(
            (
                'editable',
                'e',
                'Install the package in editable mode (compatibility flag).',
            )
        )

    boolean_options = list(getattr(_DevelopCommand, 'boolean_options', []))
    if 'editable' not in boolean_options:
        boolean_options.append('editable')

    def initialize_options(self):
        super().initialize_options()
        if not hasattr(self, 'editable'):
            self.editable = False
        if not hasattr(self, 'build_directory'):
            self.build_directory = None

    def finalize_options(self):
        if getattr(self, 'editable', False) and not getattr(self, 'build_directory', None):
            build_base = getattr(self, 'build_base', None)
            default_build_dir = Path(build_base) if build_base else Path('build') / 'develop'
            self.build_directory = str(default_build_dir)
        super().finalize_options()

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
    install_requires=['setuptools', 'onnxruntime>=1.16'],
    zip_safe=True,
    maintainer='Altinet Maintainer',
    maintainer_email='maintainer@example.com',
    description='Altinet perception stack',
    license='MIT',
    tests_require=['pytest'],
    cmdclass={'develop': DevelopCommand},
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
