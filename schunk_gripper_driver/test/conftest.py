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
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


@pytest.fixture(scope="module")
def ros2():
    rclpy.init()
    yield
    rclpy.shutdown()


@launch_pytest.fixture(scope="module")
def driver(ros2):
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


class LifecycleInterface(object):
    def __init__(self):
        self.node = Node("lifecycle_interface")
        self.change_state_client = self.node.create_client(
            ChangeState, "/schunk/driver/change_state"
        )
        self.change_state_client.wait_for_service(timeout_sec=2)
        self.get_state_client = self.node.create_client(
            GetState, "/schunk/driver/get_state"
        )
        self.get_state_client.wait_for_service(timeout_sec=2)

    def change_state(self, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def check_state(self, state_id):
        req = GetState.Request()
        future = self.get_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().current_state.id == state_id


@pytest.fixture(scope="module")
def lifecycle_interface(driver):
    return LifecycleInterface()
