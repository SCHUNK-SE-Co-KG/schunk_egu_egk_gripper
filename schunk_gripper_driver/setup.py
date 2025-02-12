from setuptools import find_packages, setup

package_name = "schunk_gripper_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="stefan",
    maintainer_email="stefan.scherzinger@de.schunk.com",
    description="ROS2 driver for SCHUNK`s EGU, EGK, and EZU mechatronic grippers",
    license="GPL-3.0-or-later",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["driver = schunk_gripper_driver.driver:main"],
    },
)
