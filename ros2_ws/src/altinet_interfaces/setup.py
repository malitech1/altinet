from pathlib import Path
import sys

from setuptools import find_packages, setup


def _normalize_setup_arguments():
    """Work around build tools passing unsupported global options.

    Some build workflows (notably `colcon build --symlink-install`) invoke the
    package's ``setup.py`` script with a top-level ``--editable`` flag.  The
    flag is understood by ``pip`` but it is not a recognized global option for
    ``setuptools``'s ``setup.py`` entry point, which causes the build to abort
    before any of the actual commands run.  To keep the build working we strip
    this flag and, when it is the only argument provided, fall back to the
    ``develop`` command that pip would normally select for editable installs.
    """

    if "--editable" not in sys.argv:
        return

    # Remove all occurrences of the unsupported flag so setuptools does not
    # exit with ``error: option --editable not recognized``.
    sys.argv = [arg for arg in sys.argv if arg != "--editable"]

    # When ``--editable`` was the only argument, replicate pip's behaviour by
    # defaulting to the ``develop`` command.  This keeps editable installs
    # functional when invoked directly (e.g. ``python setup.py --editable``).
    if len(sys.argv) == 1:
        sys.argv.append("develop")


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
