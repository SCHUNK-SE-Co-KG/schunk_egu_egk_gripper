from setuptools import find_packages, setup
import os
from glob import glob

package_name = "schunk_gripper_dummy"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["tests"]),
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("lib", package_name), ["start_dummy"]),
        (
            os.path.join("share", package_name, "config"),
            glob(package_name + "/config/*.json"),
        ),
    ],
    install_requires=[
        "setuptools",
        "fastapi",
        "uvicorn",
        "requests",
        "python-multipart",
    ],
    zip_safe=True,
    maintainer="Stefan Scherzinger",
    maintainer_email="stefan.scherzinger@de.schunk.com",
    description="A minimalist dummy for simulating the gripper's communication",
    license="GPL-3.0-or-later",
    tests_require=["pytest", "coverage"],
    entry_points={
        "console_scripts": [],
    },
    scripts=["start_dummy"],
)
