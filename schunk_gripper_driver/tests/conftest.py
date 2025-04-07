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
from launch import LaunchDescription  # type: ignore [attr-defined]
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_pytest
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

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
        self.get_state_client = self.node.create_client(
            GetState, "/schunk/driver/get_state"
        )
        self.set_params_client = self.node.create_client(
            SetParameters, "/schunk/driver/set_parameters"
        )
        self.change_state_client.wait_for_service(timeout_sec=2)
        self.get_state_client.wait_for_service(timeout_sec=2)
        self.set_params_client.wait_for_service(timeout_sec=2)

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

    def exists(self, service: str) -> bool:
        existing = getattr(self.node, "get_service_names_and_types")()
        advertised = [i[0] for i in existing]
        return service in advertised

    def use_protocol(self, protocol: str) -> bool:
        if protocol not in ["modbus", "tcpip"]:
            return False
        if protocol == "modbus":
            parameters = [
                Parameter(
                    name="host",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING, string_value=""  # empty
                    ),
                )
            ]
        if protocol == "tcpip":
            parameters = [
                Parameter(
                    name="host",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING, string_value="0.0.0.0"
                    ),
                ),
                Parameter(
                    name="port",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_INTEGER, integer_value=8000
                    ),
                ),
            ]
        future = self.set_params_client.call_async(
            SetParameters.Request(parameters=parameters)
        )
        rclpy.spin_until_future_complete(self.node, future)
        return True


@pytest.fixture(scope="module")
def lifecycle_interface(driver):
    return LifecycleInterface()
