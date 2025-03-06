from setuptools import find_packages, setup

package_name = "schunk_gripper_library"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pymodbus", "pyserial"],
    zip_safe=True,
    author="Stefan Scherzinger",
    author_email="stefan.scherzinger@de.schunk.com",
    maintainer="Stefan Scherzinger",
    maintainer_email="stefan.scherzinger@de.schunk.com",
    description="Low-level driver library for Modbus RTU and Ethernet SCHUNK grippers",
    license="GPL-3.0-or-later",
    tests_require=["pytest", "coverage"],
    entry_points={
        "console_scripts": [],
    },
)
