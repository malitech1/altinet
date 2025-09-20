from pathlib import Path
import sys

from setuptools import find_packages, setup


def _relocate_command_option(option: str, command: str) -> None:
    """Ensure command-specific options are passed after the command itself.

    ``setuptools`` only recognises command-specific options (such as
    ``--build-directory`` for the ``develop`` command) when they appear after
    the command name.  Recent versions of ``colcon`` invoke ``setup.py`` with
    the option placed before the command, which causes ``setuptools`` to abort
    early with ``error: option --build-directory not recognized``.  To remain
    compatible with those build tools we relocate the option so that it follows
    the command as expected by ``setuptools``.
    """

    original_args = sys.argv[:]

    if command not in original_args:
        return

    option_prefix = f"{option}="
    values = []
    new_args = []

    i = 0
    while i < len(original_args):
        arg = original_args[i]
        if arg == option:
            if i + 1 >= len(original_args):
                # No value provided; keep the original argument intact.
                new_args.append(arg)
                i += 1
                continue
            values.append(original_args[i + 1])
            i += 2
            continue
        if arg.startswith(option_prefix):
            values.append(arg[len(option_prefix):])
            i += 1
            continue
        new_args.append(arg)
        i += 1

    if not values:
        return

    sys.argv = new_args

    insertion_index = sys.argv.index(command) + 1
    for value in values:
        sys.argv.insert(insertion_index, f"{option}={value}")
        insertion_index += 1


def _normalize_setup_arguments():
    """Work around build tools passing unsupported or misplaced options.

    Some build workflows (notably ``colcon build --symlink-install``) invoke the
    package's ``setup.py`` script with a top-level ``--editable`` flag.  The
    flag is understood by ``pip`` but it is not a recognized global option for
    ``setuptools``'s ``setup.py`` entry point, which causes the build to abort
    before any of the actual commands run.  To keep the build working we strip
    this flag and, when it is the only argument provided, fall back to the
    ``develop`` command that pip would normally select for editable installs.

    The same build workflows can also prepend the ``--build-directory`` option
    before the ``develop`` command.  The option is valid for ``develop`` but
    only when it follows the command itself, so we relocate it if necessary.
    """

    if "--editable" in sys.argv:
        # Remove all occurrences of the unsupported flag so setuptools does not
        # exit with ``error: option --editable not recognized``.
        sys.argv = [arg for arg in sys.argv if arg != "--editable"]

        # When ``--editable`` was the only argument, replicate pip's behaviour
        # by defaulting to the ``develop`` command.  This keeps editable
        # installs functional when invoked directly (e.g. ``python setup.py
        # --editable``).
        if len(sys.argv) == 1:
            sys.argv.append("develop")

    _relocate_command_option("--build-directory", "develop")


_normalize_setup_arguments()

package_name = "altinet_interfaces"

interface_patterns = (
    ("msg", "*.msg"),
    ("srv", "*.srv"),
)

def list_interface_files():
    data = []
    for subdirectory, pattern in interface_patterns:
        matches = [
            str(Path(match))
            for match in sorted(Path(subdirectory).glob(pattern))
            if Path(match).is_file()
        ]
        if matches:
            data.append((f"share/{package_name}/{subdirectory}", matches))
    return data

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test", "tests")),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *list_interface_files(),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Altinet Maintainer",
    maintainer_email="maintainer@example.com",
    description="Message and service definitions for the Altinet perception stack.",
    license="MIT",
    tests_require=["pytest"],
    classifiers=["Programming Language :: Python"],
)
