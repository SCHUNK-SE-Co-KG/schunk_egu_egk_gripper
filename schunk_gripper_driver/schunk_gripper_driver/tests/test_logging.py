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
from rclpy.node import Node
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
def test_driver_rejects_invalid_log_level(driver):
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


@skip_without_gripper
def test_driver_logs_correct_level(driver):
    log_level = []

    driver = Driver("test_driver_logs_correct_level")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(driver)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    client = rclpy.create_node("test_log_output_set_level")
    set_params_client = client.create_client(
        SetParameters, "/test_driver_logs_correct_level/set_parameters"
    )

    assert set_params_client.wait_for_service(timeout_sec=5), "Service not available"

    future = set_params_client.call_async(
        SetParameters.Request(
            parameters=[
                Parameter(
                    name="log_level",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING, string_value="ERROR"
                    ),
                )
            ]
        )
    )

    # Spin the client node until the future is completed
    rclpy.spin_until_future_complete(client, future)
    assert future.result().results[0].successful, "Failed to set log level"

    subscription_client = Node("test_log_output")

    def callback(data: Log):
        log_level.append(data.level)

    subscription = subscription_client.create_subscription(
        Log,
        "/rosout",
        callback,
        10,
    )
    if not subscription:
        # use of subscription variable needed for checkstyle
        raise RuntimeError("Failed to create subscription")

    debug_num = 2
    info_num = 2
    warn_num = 2
    error_num = 2
    fatal_num = 2
    for i in range(debug_num):
        driver.get_logger().debug("This is a debug message")
    for i in range(info_num):
        driver.get_logger().info("This is an info message")
    for i in range(warn_num):
        driver.get_logger().warn("This is a warning message")
    for i in range(error_num):
        driver.get_logger().error("This is an error message")
    for i in range(fatal_num):
        driver.get_logger().fatal("This is a fatal message")

    subscription_executer = rclpy.executors.SingleThreadedExecutor()
    subscription_executer.add_node(subscription_client)

    subscription_thread = threading.Thread(
        target=subscription_executer.spin, daemon=True
    )
    subscription_thread.start()

    while len(log_level) != error_num + fatal_num:
        time.sleep(0.1)

    subscription_executer.shutdown()
    subscription_thread.join()

    executor.shutdown()
    executor_thread.join()

    expected_log_level = [
        40,
        50,
    ]
    unexpected_log_level = [
        10,
        20,
        30,
    ]

    for level in expected_log_level:
        assert level in log_level, f"Expected log level {level} not found in log output"
    for level in unexpected_log_level:
        assert (
            level not in log_level
        ), f"Unexpected log level {level} found in log output"
