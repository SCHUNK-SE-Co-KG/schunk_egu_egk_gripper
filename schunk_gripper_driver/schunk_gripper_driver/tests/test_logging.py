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
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import rclpy


@skip_without_gripper
def test_driver_rejects_invalid_log_level(driver):
    client = rclpy.create_node("test_driver_bad_log_level")
    set_params_client = client.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    assert set_params_client.wait_for_service(timeout_sec=2)
    get_params_client = client.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    assert get_params_client.wait_for_service(timeout_sec=2)

    invalid_levels = ["Debug", "debug", "*!€@", "-1"]

    for invalid_level in invalid_levels:
        future = set_params_client.call_async(
            SetParameters.Request(
                parameters=[
                    Parameter(
                        name="log_level",
                        value=ParameterValue(
                            type=ParameterType.PARAMETER_STRING,
                            string_value=invalid_level,
                        ),
                    )
                ]
            )
        )

        # Spin the client node until the future is completed
        rclpy.spin_until_future_complete(client, future)
        assert (
            not future.result().results[0].successful
        ), f"Setting log level to {invalid_level} should have failed"

    # Check that the log level is still set to INFO
    future = get_params_client.call_async(GetParameters.Request(names=["log_level"]))
    rclpy.spin_until_future_complete(client, future)

    result_values = future.result().values
    assert (
        result_values and result_values[0].string_value == "INFO"
    ), "Log level should be set to INFO after invalid log level was set"


@skip_without_gripper
def test_driver_logs_correct_level(log_level_checker, log_helper):
    driver = log_helper
    driver.change_log_level("DEBUG")

    # log_level_checker does the final asserting
