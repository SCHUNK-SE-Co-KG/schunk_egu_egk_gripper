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
from rcl_interfaces.srv import GetParameters, SetParameters
from schunk_gripper_library.tests.conftest import skip_without_gripper
from collections import OrderedDict


@skip_without_gripper
def test_driver_has_expected_parameters_after_startup(driver):
    node = Node("test_parameters")
    get_params_client = node.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    set_params_client = node.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    assert get_params_client.wait_for_service(timeout_sec=2)
    assert set_params_client.wait_for_service(timeout_sec=2)

    expected = OrderedDict(
        {"host": "", "port": 80, "serial_port": "/dev/ttyUSB0", "device_id": 12}
    )
    future = get_params_client.call_async(GetParameters.Request(names=expected.keys()))
    rclpy.spin_until_future_complete(node, future)

    for result, value in zip(future.result().values, expected.values()):
        if result.type == 1:
            assert result.bool_value == value
        if result.type == 2:
            assert result.integer_value == value
        if result.type == 3:
            assert result.double_value == value
        if result.type == 4:
            assert result.string_value == value
