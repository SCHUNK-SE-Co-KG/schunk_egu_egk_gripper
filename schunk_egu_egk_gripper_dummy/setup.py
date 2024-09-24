from setuptools import find_packages, setup
import os

package_name = "schunk_egu_egk_gripper_dummy"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("lib", package_name), ["start_dummy"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Stefan Scherzinger",
    maintainer_email="stefan.scherzinger@de.schunk.com",
    description="A minimalist dummy for simulating the gripper's communication",
    license="GPL-3.0-or-later",
    tests_require=["pytest", "coverage"],
    entry_points={
        "console_scripts": [],
    },
)
