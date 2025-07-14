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
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
import rclpy
from schunk_gripper_interfaces.srv import (  # type: ignore [attr-defined]
    ShowConfiguration,
)


def test_normal_startup_works(driver):
    node = Node("test_startup")
    client = node.create_client(GetState, "/schunk/driver/get_state")

    # The driver started correctly if the lifecycle interface is reachable
    assert client.wait_for_service(timeout_sec=2)


# The driver is started without default gripper
create_default_gripper = False


def test_show_configuration_is_empty_without_default_gripper(driver):
    node = Node("test_show_configuration_node")

    client = node.create_client(ShowConfiguration, "/schunk/driver/show_configuration")

    assert client.wait_for_service(timeout_sec=2.0)

    request = ShowConfiguration.Request()
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    response = future.result()
    assert response.configuration == [], "Expected empty configuration list"

    node.destroy_node()
