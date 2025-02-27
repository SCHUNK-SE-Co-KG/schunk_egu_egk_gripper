from setuptools import find_packages, setup

package_name = "schunk_gripper_library"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Stefan Scherzinger",
    maintainer_email="stefan.scherzinger@de.schunk.com",
    description="Low-level driver library for Modbus RTU and Ethernet SCHUNK grippers",
    license="GPL-3.0-or-later",
    tests_require=["pytest", "coverage"],
    entry_points={
        "console_scripts": [],
    },
)
