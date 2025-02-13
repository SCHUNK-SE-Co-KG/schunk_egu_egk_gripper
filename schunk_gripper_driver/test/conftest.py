# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_pytest
import subprocess
from ament_index_python.packages import get_package_share_directory


# We avoid black's F811, F401 linting warnings
# by using pytest's special conftest.py file.
# See documentation here:
# https://docs.pytest.org/en/7.1.x/reference/fixtures.html#conftest-py-sharing-fixtures-across-multiple-files  # noqa: E501


@pytest.fixture(scope="module")
def isolated():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def gripper_dummy():
    package_name = "schunk_egu_egk_gripper_dummy"
    dummy_dir = get_package_share_directory(package_name)
    start_cmd = " uvicorn main:server --port 8000"
    p = subprocess.Popen(
        "exec" + start_cmd, stdin=subprocess.PIPE, cwd=dummy_dir, shell=True
    )

    print("------------ Started gripper dummy")
    yield
    p.kill()
    print("------------ Stopped gripper dummy")


@launch_pytest.fixture(scope="module")
def launch_description():
    setup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("schunk_gripper_driver"),
                "launch",
                "driver.launch.py",
            ]
        )
    )
    return LaunchDescription([setup, launch_pytest.actions.ReadyToTest()])
