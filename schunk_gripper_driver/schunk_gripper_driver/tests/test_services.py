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
from schunk_gripper_interfaces.srv import (  # type: ignore [attr-defined]
    ListGrippers,
    AddGripper,
    MoveToAbsolutePosition,
    Grip,
    Release,
    ShowConfiguration,
)
from schunk_gripper_interfaces.msg import (  # type: ignore [attr-defined]
    Gripper as GripperConfig,
)
from rclpy.node import Node
import rclpy
import time


@skip_without_gripper
def test_driver_advertises_state_depending_services(lifecycle_interface):
    driver = lifecycle_interface
    list_grippers = ["/schunk/driver/list_grippers"]
    config_services = [
        "/schunk/driver/add_gripper",
        "/schunk/driver/reset_grippers",
        "/schunk/driver/show_configuration",
    ]
    gripper_services = [
        "acknowledge",
        "fast_stop",
        "move_to_absolute_position",
        "grip",
        "release",
    ]
    until_change_takes_effect = 0.1

    def exist(services: list[str]) -> bool:
        for gripper in driver.list_grippers():
            for service in services:
                if not driver.exist([f"/schunk/driver/{gripper}/{service}"]):
                    return False
        return True

    for run in range(3):

        # After startup -> unconfigured
        driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)
        assert driver.exist(config_services)
        assert not driver.exist(list_grippers)

        # After configure -> inactive
        driver.change_state(Transition.TRANSITION_CONFIGURE)
        time.sleep(until_change_takes_effect)
        assert driver.exist(list_grippers)
        assert not driver.exist(config_services)
        assert not exist(gripper_services)

        # After activate -> active
        driver.change_state(Transition.TRANSITION_ACTIVATE)
        time.sleep(until_change_takes_effect)
        assert driver.exist(list_grippers)
        assert exist(gripper_services)
        assert not driver.exist(config_services)

        # After deactivate -> inactive
        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        time.sleep(until_change_takes_effect)
        assert driver.exist(list_grippers)
        assert not driver.exist(config_services)
        assert not exist(gripper_services)

        # After cleanup -> unconfigured
        driver.change_state(Transition.TRANSITION_CLEANUP)
        time.sleep(until_change_takes_effect)
        assert driver.exist(config_services)
        assert not driver.exist(list_grippers)


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
def test_driver_implements_adding_and_resetting_grippers(driver):
    node = Node("check_adding_and_resetting_grippers")
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    # Empty request
    request = AddGripper.Request()
    future = add_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert not future.result().success

    for _ in range(3):

        # Reset gripper list
        request = Trigger.Request()
        future = reset_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        assert future.result().success

        # Add Modbus gripper
        request = AddGripper.Request()
        request.gripper.serial_port = "/dev/ttyUSB0"
        request.gripper.device_id = 12
        future = add_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        assert future.result().success

        # Add TCP/IP gripper
        request = AddGripper.Request()
        request.gripper.host = "0.0.0.0"
        request.gripper.port = 8000
        future = add_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        assert future.result().success


def test_driver_implements_show_configuration(driver):
    node = Node("check_listing_configuration")
    client = node.create_client(ShowConfiguration, "/schunk/driver/show_configuration")
    assert client.wait_for_service(timeout_sec=2)
    future = client.call_async(ShowConfiguration.Request())
    rclpy.spin_until_future_complete(node, future)
    configuration = future.result().configuration
    assert len(configuration) > 0
    assert isinstance(configuration[0], GripperConfig)


@skip_without_gripper
def test_driver_implements_acknowledge(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("check_acknowledge")
    for gripper in driver.list_grippers():
        client = node.create_client(Trigger, f"/schunk/driver/{gripper}/acknowledge")
        assert client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1)

        assert future.result().success
        expected_msg = (
            "error_code: 0x0, warning_code: 0x0, additional_code: 0x0"  # everything ok
        )
        assert future.result().message == expected_msg

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_driver_implements_fast_stop(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("check_fast_stop")
    for gripper in driver.list_grippers():
        client = node.create_client(Trigger, f"/schunk/driver/{gripper}/fast_stop")
        assert client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1)

        assert future.result().success

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_driver_implements_move_to_absolute_position(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    assert driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("check_move_to_absolute_position")
    for gripper in driver.list_grippers():
        client = node.create_client(
            MoveToAbsolutePosition,
            f"/schunk/driver/{gripper}/move_to_absolute_position",
        )
        assert client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        targets = [
            {"position": 0.023, "velocity": 0.02, "use_gpe": False},
            {"position": 0.005, "velocity": 0.02, "use_gpe": True},
        ]
        for target in targets:
            request = MoveToAbsolutePosition.Request()
            request.position = target["position"]
            request.velocity = target["velocity"]
            request.use_gpe = target["use_gpe"]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            assert future.result().success, f"{future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_driver_implements_grip_and_release(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_grip")
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    # Drop default modbus gripper because that can't grip in simulation.
    request = Trigger.Request()
    future = reset_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    # Add TCP/IP gripper
    request = AddGripper.Request()
    request.gripper.host = "0.0.0.0"
    request.gripper.port = 8000
    future = add_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Get the gripper's services
    for gripper in driver.list_grippers():
        grip_client = node.create_client(
            Grip,
            f"/schunk/driver/{gripper}/grip",
        )
        assert grip_client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"
        release_client = node.create_client(
            Release,
            f"/schunk/driver/{gripper}/release",
        )
        assert release_client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        targets = [
            {"force": 50, "use_gpe": False, "outward": False},
            {"force": 100, "use_gpe": True, "outward": False},
            {"force": 75, "use_gpe": False, "outward": True},
            {"force": 88, "use_gpe": True, "outward": True},
        ]
        for target in targets:

            # Grip
            request = Grip.Request()
            request.force = target["force"]
            request.use_gpe = target["use_gpe"]
            request.outward = target["outward"]
            future = grip_client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

            # Release
            future = release_client.call_async(Release.Request())
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
