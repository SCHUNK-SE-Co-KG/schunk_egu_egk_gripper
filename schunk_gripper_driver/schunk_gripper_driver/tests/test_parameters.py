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
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from schunk_gripper_library.utility import skip_without_gripper


DRIVER_PARAMETERS = [
    "host",
    "port",
    "serial_port",
    "device_id",
    "log_level",
    "start_empty",
]


@skip_without_gripper
def test_whether_we_cover_all_driver_parameters(driver):
    node = Node("test_startup_parameters")
    list_params_client = node.create_client(
        ListParameters, "/schunk/driver/list_parameters"
    )
    assert list_params_client.wait_for_service(timeout_sec=2)

    # Meta-check if we test all our node parameters
    future = list_params_client.call_async(ListParameters.Request())
    rclpy.spin_until_future_complete(node, future)
    assert (
        len(DRIVER_PARAMETERS) == len(future.result().result.names) - 1
    )  # There's a default node parameter: `use_sim_time``


@skip_without_gripper
def test_driver_has_expected_parameters_after_startup(driver):
    node = Node("test_startup_parameters")
    get_params_client = node.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    assert get_params_client.wait_for_service(timeout_sec=2)

    default_parameters = [
        Parameter(
            name="host",
            value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=""),
        ),
        Parameter(
            name="port",
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=80
            ),
        ),
        Parameter(
            name="serial_port",
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value="/dev/ttyUSB0"
            ),
        ),
        Parameter(
            name="device_id",
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=12
            ),
        ),
        Parameter(
            name="log_level",
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value="INFO"
            ),
        ),
        Parameter(
            name="start_empty",
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False),
        ),
    ]
    future = get_params_client.call_async(
        GetParameters.Request(names=DRIVER_PARAMETERS)
    )
    rclpy.spin_until_future_complete(node, future)
    for param, expected in zip(default_parameters, future.result().values):
        assert param.value == expected


@skip_without_gripper
def test_driver_supports_setting_parameters(driver):
    node = Node("test_setting_parameters")
    get_params_client = node.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    set_params_client = node.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    assert get_params_client.wait_for_service(timeout_sec=2)
    assert set_params_client.wait_for_service(timeout_sec=2)

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
                type=ParameterType.PARAMETER_INTEGER, integer_value=1234
            ),
        ),
        Parameter(
            name="serial_port",
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value="/dev/serial-port"
            ),
        ),
        Parameter(
            name="device_id",
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=123
            ),
        ),
        Parameter(
            name="log_level",
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value="DEBUG"
            ),
        ),
        Parameter(
            name="start_empty",
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True),
        ),
    ]

    future = set_params_client.call_async(SetParameters.Request(parameters=parameters))
    rclpy.spin_until_future_complete(node, future)
    print(future.result())
    future = get_params_client.call_async(
        GetParameters.Request(names=DRIVER_PARAMETERS)
    )
    rclpy.spin_until_future_complete(node, future)

    for param, expected in zip(parameters, future.result().values):
        assert param.value == expected
