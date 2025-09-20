from pathlib import Path

from setuptools import find_packages, setup

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
