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

from schunk_gripper_library.tests.conftest import skip_without_gripper
from lifecycle_msgs.msg import Transition
from std_srvs.srv import Trigger
from rclpy.node import Node
import rclpy


@skip_without_gripper
def test_driver_advertises_services_when_configured(lifecycle_interface):
    driver = lifecycle_interface
    for protocol in ["modbus", "tcpip"]:
        driver.use_protocol(protocol)
        service_list = ["/schunk/driver/acknowledge", "/schunk/driver/fast_stop"]
        # In unconfigured state
        for service in service_list:
            assert not driver.exists(service)

        # Should appear after configure
        driver.change_state(Transition.TRANSITION_CONFIGURE)
        for service in service_list:
            driver.exists(service)

        # Should disappear after cleanup
        driver.change_state(Transition.TRANSITION_CLEANUP)
        for service in service_list:
            not driver.exists(service)


@skip_without_gripper
def test_driver_implements_acknowledge(lifecycle_interface):
    driver = lifecycle_interface
    for protocol in ["modbus", "tcpip"]:
        driver.use_protocol(protocol)
        driver.change_state(Transition.TRANSITION_CONFIGURE)

        node = Node("check_acknowledge")
        client = node.create_client(Trigger, "/schunk/driver/acknowledge")
        client.wait_for_service(timeout_sec=2)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future)
        driver.change_state(Transition.TRANSITION_CLEANUP)

        assert future.result().success
        expected_msg = (
            "error_code: 0x0, warning_code: 0x0, additional_code: 0x0"  # everything ok
        )
        assert future.result().message == expected_msg


@skip_without_gripper
def test_driver_implements_fast_stop(lifecycle_interface):
    driver = lifecycle_interface
    for protocol in ["modbus", "tcpip"]:
        driver.use_protocol(protocol)
        driver.change_state(Transition.TRANSITION_CONFIGURE)

        node = Node("check_fast_stop")
        client = node.create_client(Trigger, "/schunk/driver/fast_stop")
        client.wait_for_service(timeout_sec=2)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future)
        driver.change_state(Transition.TRANSITION_CLEANUP)

        assert future.result().success
        expected_msg = "error_code: 0xD9, warning_code: 0x0, additional_code: 0x0"
        assert future.result().message == expected_msg
