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
from rcl_interfaces.msg import Log
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import rclpy
from schunk_gripper_driver.driver import Driver
import threading

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
def test_driver_rejects_invalid_log_level(request):
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
    print(f"####result {future.result().values}")
    result_values = future.result().values
    assert (
        result_values and result_values[0].string_value == "INFO"
    ), "Log level should be set to INFO after invalid log level was set"


@skip_without_gripper
def test_driver_logs_correct_level():
    # Set up driver with log level set to ERROR
    driver = Driver("test_driver_logs_correct_level")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(driver)

    # Create client to set log level parameter
    client = rclpy.create_node("test_log_client")
    executor.add_node(client)

    # Set log level to ERROR
    set_params_client = client.create_client(
        SetParameters, "/test_driver_logs_correct_level/set_parameters"
    )
    assert set_params_client.wait_for_service(timeout_sec=2), "Service not available"
    request = SetParameters.Request(
        parameters=[
            Parameter(
                name="log_level",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value="ERROR"
                ),
            )
        ]
    )
    future = set_params_client.call_async(request)
    executor.spin_until_future_complete(future)
    assert future.result().results[0].successful, "Failed to set log level"

    # Set up rosout subscription to capture logs
    log_levels = []
    subscription_client = rclpy.create_node("test_log_listener")
    subscription_client.create_subscription(
        Log, "/rosout", lambda msg: log_levels.append(msg.level), 10
    )
    executor.add_node(subscription_client)

    # Start spinning in a separate thread
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    # Generate logs at all levels
    logs = {"debug": 2, "info": 2, "warn": 2, "error": 2, "fatal": 2}

    for level, count in logs.items():
        for _ in range(count):
            match level:
                case "debug":
                    driver.get_logger().debug("Debug message")
                case "info":
                    driver.get_logger().info("Info message")
                case "warn":
                    driver.get_logger().warn("Warn message")
                case "error":
                    driver.get_logger().error("Error message")
                case "fatal":
                    driver.get_logger().fatal("Fatal message")

    # Wait for logs to be processed
    timeout = time.time() + 3.0
    while len(log_levels) < 4 and time.time() < timeout:
        time.sleep(0.1)

    # Shutdown and cleanup
    executor.shutdown()
    thread.join(timeout=1.0)

    # Verify log levels
    assert all(
        level in [40, 50] for level in log_levels
    ), "Unexpected log levels received"
    assert 40 in log_levels, "ERROR level logs not received"
    assert 50 in log_levels, "FATAL level logs not received"
    assert len(log_levels) == 4, f"Expected 4 logs, got {len(log_levels)}"
