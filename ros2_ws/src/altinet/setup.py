import shutil
import sys
from glob import glob
from pathlib import Path

from setuptools import find_packages, setup
from setuptools.command.develop import develop as _DevelopCommand
from setuptools.command.install import install as _InstallCommand


# ``colcon`` forwards the ``--editable`` flag as a *global* option when invoking
# ``setup.py``.  ``setuptools`` does not recognise this global option which
# results in an early failure before our custom ``develop`` command gets a
# chance to handle it.  Strip the compatibility flag from ``sys.argv`` so the
# command parser can continue normally.  The flag is still honoured by the
# custom develop command below.
def _strip_editable_flags(argv):
    """Return ``argv`` without any variant of the ``--editable`` flag."""

    sanitized = []
    for arg in argv:
        if arg in ('--editable', '-e'):
            continue
        if arg.startswith('--editable='):
            continue
        sanitized.append(arg)
    return sanitized


sys.argv[:] = _strip_editable_flags(sys.argv)


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


class InstallCommand(_InstallCommand):
    """Custom install command that understands the ``--uninstall`` flag."""

    user_options = list(_InstallCommand.user_options)
    if not any(option[0].startswith('uninstall') for option in user_options):
        user_options.append(
            (
                'uninstall',
                'u',
                'Uninstall this distribution before installing (compatibility flag).',
            )
        )

    boolean_options = list(getattr(_InstallCommand, 'boolean_options', []))
    if 'uninstall' not in boolean_options:
        boolean_options.append('uninstall')

    def initialize_options(self):
        super().initialize_options()
        if not hasattr(self, 'uninstall'):
            self.uninstall = False

    def run(self):
        if getattr(self, 'uninstall', False):
            self._uninstall_previous_installation()
        super().run()

    # ------------------------------------------------------------------
    # Compatibility helpers
    # ------------------------------------------------------------------
    def _uninstall_previous_installation(self):
        record_option = getattr(self, 'record', None)
        record_path = None
        if record_option:
            record_path = Path(record_option)
            if not record_path.is_absolute():
                record_path = Path.cwd() / record_path

        records = []
        if record_path and record_path.exists():
            try:
                with record_path.open() as record_file:
                    records = [line.strip() for line in record_file if line.strip()]
            except OSError:
                records = []

        root_path = None
        root_value = getattr(self, 'root', None)
        if root_value:
            root_path = Path(root_value)

        for entry in records:
            recorded_path = Path(entry)
            target_path = self._resolve_recorded_path(recorded_path, root_path)
            self._remove_path(target_path, root_path)

        if record_path and record_path.exists():
            try:
                record_path.unlink()
            except OSError:
                pass

        if not records:
            self._best_effort_cleanup_without_record()

    def _resolve_recorded_path(self, recorded_path, root_path):
        if root_path is None:
            return recorded_path
        if recorded_path.is_absolute():
            parts = recorded_path.parts[1:]
            if parts:
                return root_path.joinpath(*parts)
            return root_path
        return root_path / recorded_path

    def _remove_path(self, path, root_path):
        try:
            exists = path.exists()
        except OSError:
            exists = False
        if not exists:
            return
        if path.is_dir():
            shutil.rmtree(path, ignore_errors=True)
        else:
            try:
                path.unlink()
            except IsADirectoryError:
                shutil.rmtree(path, ignore_errors=True)
            except OSError:
                pass
        if root_path:
            self._cleanup_empty_parents(path.parent, root_path)

    def _cleanup_empty_parents(self, start_dir, root_path):
        try:
            root_resolved = root_path.resolve()
        except OSError:
            root_resolved = root_path
        current = start_dir
        while current and current != current.parent:
            try:
                if current.resolve() == root_resolved:
                    break
            except OSError:
                break
            try:
                current.rmdir()
            except OSError:
                break
            current = current.parent

    def _best_effort_cleanup_without_record(self):
        install_lib = getattr(self, 'install_lib', None)
        if install_lib:
            package_dir = Path(install_lib) / self.distribution.get_name()
            if package_dir.exists():
                shutil.rmtree(package_dir, ignore_errors=True)
        install_scripts = getattr(self, 'install_scripts', None)
        if install_scripts:
            scripts_dir = Path(install_scripts)
            if scripts_dir.exists():
                for script in scripts_dir.glob(f"{self.distribution.get_name()}*"):
                    try:
                        script.unlink()
                    except OSError:
                        pass

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
        ('share/' + package_name + '/assets/models', glob('assets/models/*')),
    ],
    install_requires=['setuptools', 'onnxruntime>=1.16'],
    zip_safe=True,
    maintainer='Altinet Maintainer',
    maintainer_email='maintainer@example.com',
    description='Altinet perception stack',
    license='MIT',
    tests_require=['pytest'],
    cmdclass={'develop': DevelopCommand, 'install': InstallCommand},
    entry_points={
        'console_scripts': [
            'camera_node = altinet.nodes.camera_node:main',
            'camera_viewer_node = altinet.nodes.camera_viewer_node:main',
            'detector_node = altinet.nodes.detector_node:main',
            'face_capture_node = altinet.nodes.face_capture_node:main',
            'identity_node = altinet.nodes.identity_node:main',
            'tracker_node = altinet.nodes.tracker_node:main',
            'event_manager_node = altinet.nodes.event_manager_node:main',
            'lighting_control_node = altinet.nodes.lighting_control_node:main',
            'ros2_django_bridge_node = altinet.nodes.ros2_django_bridge_node:main',
            'visualizer_node = altinet.nodes.visualizer_node:main',
        ],
    },
)
