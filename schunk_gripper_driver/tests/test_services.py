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
from lifecycle_msgs.msg import Transition, State
from std_srvs.srv import Trigger
from schunk_gripper_interfaces.srv import ListGrippers  # type: ignore [attr-defined]
from rclpy.node import Node
import rclpy
import time


@skip_without_gripper
def test_driver_advertises_services(lifecycle_interface):
    driver = lifecycle_interface
    for _ in range(3):

        list_grippers = "/schunk/driver/list_grippers"
        until_change_takes_effect = 0.1

        # After startup -> unconfigured
        assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)
        assert not driver.exists(list_grippers)

        # After configure -> inactive
        driver.change_state(Transition.TRANSITION_CONFIGURE)
        time.sleep(until_change_takes_effect)
        assert driver.exists(list_grippers)

        # After activate -> active
        driver.change_state(Transition.TRANSITION_ACTIVATE)
        time.sleep(until_change_takes_effect)
        assert driver.exists(list_grippers)

        # After deactivate -> inactive
        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        time.sleep(until_change_takes_effect)
        assert driver.exists(list_grippers)

        # After cleanup -> unconfigured
        driver.change_state(Transition.TRANSITION_CLEANUP)
        time.sleep(until_change_takes_effect)
        assert not driver.exists(list_grippers)


@skip_without_gripper
def test_driver_implements_list_grippers(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)

    node = Node("check_list_grippers")
    client = node.create_client(ListGrippers, "/schunk/driver/list_grippers")
    assert client.wait_for_service(timeout_sec=2)
    future = client.call_async(ListGrippers.Request())
    rclpy.spin_until_future_complete(node, future)
    assert len(future.result().grippers) >= 1
    driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_driver_implements_acknowledge(lifecycle_interface):
    driver = lifecycle_interface
    for protocol in ["modbus", "tcpip"]:
        driver.use_protocol(protocol)
        driver.change_state(Transition.TRANSITION_CONFIGURE)

        node = Node("check_acknowledge")
        client = node.create_client(Trigger, "/schunk/driver/acknowledge")
        assert client.wait_for_service(timeout_sec=2)
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
        assert client.wait_for_service(timeout_sec=2)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future)
        driver.change_state(Transition.TRANSITION_CLEANUP)

        assert future.result().success
        expected_msg = "error_code: 0xD9, warning_code: 0x0, additional_code: 0x0"
        assert future.result().message == expected_msg
