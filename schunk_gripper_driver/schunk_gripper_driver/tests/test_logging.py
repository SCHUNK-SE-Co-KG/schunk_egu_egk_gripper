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
from schunk_gripper_library.utility import skip_without_gripper
from lifecycle_msgs.msg import Transition
import time
import pytest
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import rclpy

LOG_SIZE_BYTES = 500  # Minimal output for driver startup and shutdown


@skip_without_gripper
@pytest.mark.parametrize(
    "log_monitor", [{"max_log_size": LOG_SIZE_BYTES}], indirect=True
)
def test_driver_doesnt_fill_disk_space_by_default(log_monitor, lifecycle_interface):
    driver = lifecycle_interface

    for _ in range(10):

        driver.change_state(Transition.TRANSITION_CONFIGURE)
        driver.change_state(Transition.TRANSITION_ACTIVATE)

        time.sleep(0.1)

        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        driver.change_state(Transition.TRANSITION_CLEANUP)

    # log_monitor does the final asserting


@skip_without_gripper
def test_driver_supports_log_level_changes_via_parameter_call(driver):
    client = rclpy.create_node("test_log_level")
    set_params_client = client.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    assert set_params_client.wait_for_service(timeout_sec=2)
    get_params_client = client.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    assert get_params_client.wait_for_service(timeout_sec=2)
    future = set_params_client.call_async(
        SetParameters.Request(
            parameters=[
                Parameter(
                    name="log_level",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING, string_value="DEBUG"
                    ),
                )
            ]
        )
    )
    rclpy.spin_until_future_complete(client, future)
    future = get_params_client.call_async(GetParameters.Request(names=["log_level"]))
    rclpy.spin_until_future_complete(client, future)
    assert future.result().values[0].string_value == "DEBUG"
    future = set_params_client.call_async(
        SetParameters.Request(
            parameters=[
                Parameter(
                    name="log_level",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING, string_value="INFO"
                    ),
                )
            ]
        )
    )
    rclpy.spin_until_future_complete(client, future)
    future = get_params_client.call_async(GetParameters.Request(names=["log_level"]))
    rclpy.spin_until_future_complete(client, future)
    assert future.result().values[0].string_value == "INFO"


@skip_without_gripper
def test_driver_doesnt_set_invalid_log_level(driver):
    client = rclpy.create_node("test_bad_log_level")
    set_params_client = client.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    assert set_params_client.wait_for_service(timeout_sec=2)
    get_params_client = client.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    assert get_params_client.wait_for_service(timeout_sec=2)

    # Attempt to set an invalid log level
    future = set_params_client.call_async(
        SetParameters.Request(
            parameters=[
                Parameter(
                    name="log_level",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING,
                        string_value="BAD_LOG_LEVEL",
                    ),
                )
            ]
        )
    )
    rclpy.spin_until_future_complete(client, future, timeout_sec=2)
    assert future.result().results[0].successful is False
    # Verify the log level was not set to the invalid value
    future = get_params_client.call_async(GetParameters.Request(names=["log_level"]))
    rclpy.spin_until_future_complete(client, future)
    assert future.result().values[0].string_value != "BAD_LOG_LEVEL"
