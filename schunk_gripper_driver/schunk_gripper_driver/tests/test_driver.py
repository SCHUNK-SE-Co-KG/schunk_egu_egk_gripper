# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from schunk_gripper_driver.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper
from schunk_gripper_library.driver import Driver as GripperDriver
from std_srvs.srv import Trigger
from schunk_gripper_interfaces.srv import (  # type: ignore [attr-defined]
    AddGripper,
    MoveToAbsolutePosition,
    Grip,
    Release,
)
from schunk_gripper_interfaces.msg import (  # type: ignore [attr-defined]
    Gripper as GripperConfig,
)
from schunk_gripper_driver.driver import Gripper


def test_driver_manages_a_list_of_grippers(ros2: None):
    driver = Driver("driver")
    assert len(driver.grippers) == 1
    assert isinstance(driver.grippers[0], dict)
    entries = ["host", "port", "serial_port", "device_id", "driver", "gripper_id"]
    for entry in entries:
        assert entry in driver.grippers[0]
    assert isinstance(driver.grippers[0]["driver"], GripperDriver)
    assert driver.grippers[0]["gripper_id"] == ""


@skip_without_gripper
def test_driver_manages_individual_drivers_for_each_gripper(ros2: None):
    driver = Driver("driver")

    driver.on_configure(state=None)
    for gripper in driver.grippers:
        assert gripper["driver"].connected

    driver.on_cleanup(state=None)
    for gripper in driver.grippers:
        assert not gripper["driver"].connected


@skip_without_gripper
def test_driver_offers_list_of_connected_grippers(ros2: None):
    driver = Driver("driver")

    def assert_gripper_ids(device_ids: list[str]) -> None:
        # Device IDs should contain the module type and a trailing count, e.g.
        # EGK_40_M_B_1, EGK_40_M_B_2
        for id in device_ids:
            module_type = id[:-2]
            assert module_type in GripperDriver().valid_module_types.values()
            nr = int(id.split("_")[-1])
            assert nr >= 1

    # When unconfigured
    assert driver.list_grippers() == []

    # When inactive
    driver.on_configure(state=None)
    assert len(driver.list_grippers()) >= 1  # default setting
    assert_gripper_ids(driver.list_grippers())

    # When active
    driver.on_activate(state=None)
    assert len(driver.list_grippers()) >= 1
    assert_gripper_ids(driver.list_grippers())

    # Finish
    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)


@skip_without_gripper
def test_driver_manages_services_for_each_gripper(ros2: None):
    driver = Driver("driver")

    for _ in range(3):
        assert driver.gripper_services == []
        driver.on_configure(state=None)
        assert driver.gripper_services == []

        driver.on_activate(state=None)
        assert len(driver.gripper_services) >= 1

        driver.on_deactivate(state=None)
        assert driver.gripper_services == []
        driver.on_cleanup(state=None)
        assert driver.gripper_services == []


@skip_without_gripper
def test_driver_manages_publishers_for_each_gripper(ros2: None):
    driver = Driver("driver")

    for _ in range(3):
        assert driver.joint_state_publishers == {}
        assert driver.gripper_state_publishers == {}
        driver.on_configure(state=None)
        assert driver.joint_state_publishers == {}
        assert driver.gripper_state_publishers == {}

        driver.on_activate(state=None)
        nr_grippers = len(driver.list_grippers())
        assert len(driver.joint_state_publishers) == nr_grippers
        assert len(driver.gripper_state_publishers) == nr_grippers

        driver.on_deactivate(state=None)
        assert driver.joint_state_publishers == {}
        assert driver.gripper_state_publishers == {}
        driver.on_cleanup(state=None)
        assert driver.joint_state_publishers == {}
        assert driver.gripper_state_publishers == {}


@skip_without_gripper
def test_driver_manages_timers_for_each_gripper(ros2: None):
    driver = Driver("driver")

    for _ in range(3):
        assert driver.gripper_timers == []
        driver.on_configure(state=None)
        assert driver.gripper_timers == []

        driver.on_activate(state=None)
        assert len(driver.gripper_timers) >= 1

        driver.on_deactivate(state=None)
        assert driver.gripper_timers == []

        driver.on_cleanup(state=None)
        assert driver.gripper_timers == []


def test_driver_checks_if_grippers_need_synchronization(ros2: None):
    driver = Driver("driver")  # with default gripper

    # Same serial port
    gripper = Gripper(
        {
            "host": "",
            "port": 0,
            "serial_port": "/dev/ttyUSB0",
            "device_id": 12,
            "driver": GripperDriver(),
            "gripper_id": "",
        }
    )
    driver.grippers.append(gripper)
    assert driver.needs_synchronize(gripper)

    # Unique serial port
    serial_ports = {
        "/dev/unique-port",
        "",
        "/dev/ttyUSB1",
        "/dev/ttyUSB01",
        "/dev/ttyUSB\0",
    }
    for serial_port in serial_ports:
        gripper = Gripper(
            {
                "host": "",
                "port": 0,
                "serial_port": serial_port,
                "device_id": 12,
                "driver": GripperDriver(),
                "gripper_id": "",
            }
        )
        driver.grippers.append(gripper)
        assert not driver.needs_synchronize(gripper)


@skip_without_gripper
def test_driver_offers_callbacks_for_acknowledge_and_fast_stop(ros2: None):
    driver = Driver("driver")
    driver.on_configure(state=None)
    driver.on_activate(state=None)

    req = Trigger.Request()
    res = Trigger.Response()
    for idx, _ in enumerate(driver.grippers):
        gripper = driver.grippers[idx]
        gripper_id = gripper["gripper_id"]
        assert driver._acknowledge_cb(
            request=req, response=res, gripper=gripper
        ), f"gripper_id: {gripper_id}"
        assert res.success
        assert driver._fast_stop_cb(
            request=req, response=res, gripper=gripper
        ), f"gripper_id: {gripper_id}"
        assert res.success

    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)


@skip_without_gripper
def test_driver_offers_callback_for_move_to_absolute_position(ros2: None):
    driver = Driver("driver")
    driver.on_configure(state=None)
    driver.on_activate(state=None)

    # Check if we can call the interface.
    # It will fail with an empty request, but that's ok.
    req = MoveToAbsolutePosition.Request()
    res = MoveToAbsolutePosition.Response()
    for idx, _ in enumerate(driver.grippers):
        gripper = driver.grippers[idx]
        driver._move_to_absolute_position_cb(request=req, response=res, gripper=gripper)
        assert not res.success

    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)


@skip_without_gripper
def test_driver_offers_callback_for_grip(ros2: None):
    driver = Driver("driver")
    driver.on_configure(state=None)
    driver.on_activate(state=None)

    # Check if we can call the interface.
    req = Grip.Request()
    res = Grip.Response()
    for idx, _ in enumerate(driver.grippers):
        gripper = driver.grippers[idx]
        driver._grip_cb(request=req, response=res, gripper=gripper)
        assert not res.success
        assert res.message != ""

    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)


@skip_without_gripper
def test_driver_offers_callback_for_release(ros2: None):
    driver = Driver("driver")
    driver.on_configure(state=None)
    driver.on_activate(state=None)

    # Check if we can call the interface.
    req = Release.Request()
    res = Release.Response()
    for idx, _ in enumerate(driver.grippers):
        gripper = driver.grippers[idx]
        driver._release_cb(request=req, response=res, gripper=gripper)
        assert not res.success
        assert res.message != ""

    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)


@skip_without_gripper
def test_driver_runs_a_scheduler_for_concurrent_tasks(ros2: None):
    driver = Driver("driver")
    assert driver.scheduler is not None

    def scheduler_running() -> bool:
        return driver.scheduler.worker_thread.is_alive()

    assert not scheduler_running()
    driver.on_configure(state=None)
    assert scheduler_running()
    driver.on_activate(state=None)
    assert scheduler_running()
    driver.on_deactivate(state=None)
    assert scheduler_running()
    driver.on_cleanup(state=None)
    assert not scheduler_running()


def test_driver_offers_adding_grippers(ros2: None):
    driver = Driver("driver")
    driver.grippers.clear()
    assert driver.add_gripper(
        host="0.0.0.0", port=8000, serial_port="/dev/ttyUSB0", device_id=12
    )
    assert len(driver.grippers) == 1

    # Empty arguments
    driver.grippers.clear()
    assert not driver.add_gripper()

    # Incomplete arguments
    driver.grippers.clear()
    assert not driver.add_gripper(host="0.0.0.0")
    assert not driver.add_gripper(port=8000)
    assert not driver.add_gripper(serial_port="/dev/ttyUSB0")
    assert not driver.add_gripper(device_id=12)
    assert len(driver.grippers) == 0

    # Valid arguments
    driver.grippers.clear()
    assert driver.add_gripper(host="0.0.0.0", port=8000)
    assert driver.add_gripper(serial_port="/dev/ttyUSB0", device_id=12)
    assert len(driver.grippers) == 2


def test_driver_rejects_adding_duplicate_grippers(ros2: None):
    driver = Driver("driver")
    driver.grippers.clear()
    unique_setups = [
        {"host": "1", "port": 1},
        {"host": "2", "port": 2},
        # TCP/IP takes preference when both are given
        {"host": "2", "port": 3, "serial_port": "/", "device_id": 12},
        {"host": "2", "port": 4, "serial_port": "/", "device_id": 12},
        {"serial_port": "/dev/1", "device_id": 12},
        {"serial_port": "/dev/2", "device_id": 13},
    ]
    for setup in unique_setups:
        assert driver.add_gripper(**setup)  # type: ignore [arg-type]

    driver.grippers.clear()
    driver.add_gripper(host="1", port=1, serial_port="/dev/1", device_id=12)
    overlapping_setups = [
        {"host": "1", "port": 1},
        {"host": "1", "port": 1, "serial_port": "/dev/1"},
        {"port": 1, "serial_port": "/dev/1", "device_id": 12},
        {"serial_port": "/dev/1", "device_id": 12},
    ]
    for setup in overlapping_setups:
        assert not driver.add_gripper(**setup)  # type: ignore [arg-type]


def test_driver_offers_resetting_grippers(ros2: None):
    driver = Driver("driver")
    assert len(driver.grippers) == 1
    assert driver.reset_grippers()
    assert len(driver.grippers) == 0

    # Repeated reset
    for _ in range(3):
        assert driver.reset_grippers()


@skip_without_gripper
def test_driver_schedules_concurrent_tasks(ros2: None):
    driver = Driver("driver")
    driver.reset_grippers()

    # Modbus gripper
    request_1 = AddGripper.Request()
    request_1.gripper.serial_port = "/dev/ttyUSB0"
    request_1.gripper.device_id = 12
    response = AddGripper.Response()
    driver._add_gripper_cb(request=request_1, response=response)
    assert response.success

    # Use a tcp/ip gripper but give it the same serial port
    # so that task scheduling kicks in.
    request_2 = AddGripper.Request()
    request_2.gripper.host = "0.0.0.0"
    request_2.gripper.port = 8000
    request_2.gripper.serial_port = "/dev/ttyUSB0"
    request_2.gripper.device_id = 42
    response = AddGripper.Response()
    driver._add_gripper_cb(request=request_2, response=response)
    assert response.success

    # Grippers with a concurrent serial port can't have an update cycle.
    driver.on_configure(state=None)
    assert len(driver.grippers) == 2
    assert not driver.grippers[0]["driver"].polling_thread.is_alive()
    assert not driver.grippers[1]["driver"].polling_thread.is_alive()

    driver.on_activate(state=None)

    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)


def test_driver_shows_configuration(ros2: None):
    driver = Driver("driver")
    config = driver.show_configuration()
    assert len(config) == 1  # with default setting
    assert isinstance(config[0], GripperConfig)

    # Add some grippers and check the information
    gripper1 = {
        "host": "abc",
        "port": 1234,
        "serial_port": "$asd/123/?",
        "device_id": 55,
    }
    gripper2 = {"host": "xyz", "port": 42, "serial_port": "/dev/0", "device_id": 66}
    assert driver.add_gripper(**gripper1)  # type: ignore [arg-type]
    assert driver.add_gripper(**gripper2)  # type: ignore [arg-type]
    config = driver.show_configuration()

    assert gripper1["host"] == config[1].host
    assert gripper1["port"] == config[1].port
    assert gripper1["serial_port"] == config[1].serial_port
    assert gripper1["device_id"] == config[1].device_id

    assert gripper2["host"] == config[2].host
    assert gripper2["port"] == config[2].port
    assert gripper2["serial_port"] == config[2].serial_port
    assert gripper2["device_id"] == config[2].device_id

    # After reset
    driver.reset_grippers()
    assert driver.show_configuration() == []


@skip_without_gripper
def test_driver_uses_separate_callback_group_for_publishers(ros2: None):
    driver = Driver("driver")

    driver.on_configure(state=None)
    driver.on_activate(state=None)

    # Joint states
    for publisher in driver.joint_state_publishers.values():
        for handler in publisher.event_handlers:
            assert handler.callback_group != driver.default_callback_group

    # Gripper state
    for publisher in driver.gripper_state_publishers.values():
        for handler in publisher.event_handlers:
            assert handler.callback_group != driver.default_callback_group

    # Timers
    for timer in driver.gripper_timers:
        assert timer.callback_group != driver.default_callback_group

    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)
