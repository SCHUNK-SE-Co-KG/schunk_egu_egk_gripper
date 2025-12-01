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

from schunk_gripper_driver.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper
from schunk_gripper_interfaces.srv import (  # type: ignore [attr-defined]
    ScanGrippers,
)
from unittest.mock import patch
from rclpy.node import Node
import rclpy


def test_driver_has_an_ethernet_scanner(ros2):
    driver = Driver("driver")
    assert driver.ethernet_scanner is not None


@skip_without_gripper
def test_driver_scans_info_from_existing_ethernet_grippers(ros2):
    driver = Driver("driver")

    # Patch our scan method to return a connection to the web dummy
    # and check if we read the module type correctly
    with patch.object(
        driver.ethernet_scanner,
        "scan",
        return_value=[{"host": "0.0.0.0", "port": 8000}],
    ):

        request = ScanGrippers.Request()
        response = ScanGrippers.Response()
        driver._scan_grippers_cb(request=request, response=response)
        assert len(response.grippers) == 1
        assert len(response.connections) == 1

        assert response.grippers[0].startswith("EG")
        assert response.connections[0].host == "0.0.0.0"
        assert response.connections[0].port == 8000


@skip_without_gripper
def test_driver_implements_scan(driver):
    node = Node("check_scan")
    client = node.create_client(ScanGrippers, "/schunk/driver/scan")
    assert client.wait_for_service(timeout_sec=2)

    future = client.call_async(ScanGrippers.Request())
    rclpy.spin_until_future_complete(node, future)

    # Check that we can call the interface
    assert future.result()
