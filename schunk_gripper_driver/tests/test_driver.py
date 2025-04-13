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


def test_driver_manages_a_list_of_grippers(ros2):
    driver = Driver("driver")
    assert len(driver.grippers) == 1
    assert isinstance(driver.grippers[0], dict)
    entries = ["host", "port", "serial_port", "device_id", "driver", "gripper_id"]
    for entry in entries:
        assert entry in driver.grippers[0]
    assert driver.grippers[0]["driver"] is None
    assert driver.grippers[0]["gripper_id"] is None


@skip_without_gripper
def test_driver_manages_individual_drivers_for_each_gripper(ros2):
    driver = Driver("driver")

    driver.on_configure(state=None)
    for gripper in driver.grippers:
        assert isinstance(gripper["driver"], GripperDriver)

    driver.on_cleanup(state=None)
    for gripper in driver.grippers:
        assert gripper["driver"] is None


@skip_without_gripper
def test_driver_offers_list_of_connected_grippers(ros2):
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
def test_driver_manages_services_for_each_gripper(ros2):
    driver = Driver("driver")

    for _ in range(3):
        assert driver.gripper_services == []
        driver.on_configure(state=None)
        assert driver.gripper_services == []

        driver.on_activate(state=None)
        assert len(driver.gripper_services) >= 1

        driver.on_deactivate(state=None)
        assert driver.gripper_services == []
        driver.on_cleanup
        assert driver.gripper_services == []


def test_driver_checks_if_grippers_need_synchronization():
    driver = Driver("driver")  # with default gripper

    # Same serial port
    gripper = {
        "serial_port": "/dev/ttyUSB0",
    }
    driver.grippers.append(gripper)
    assert driver.needs_synchronize(gripper)

    # Unique serial port
    serial_ports = {
        "/dev/unique-port",
        None,
        "",
        "/dev/ttyUSB1",
        "/dev/ttyUSB01",
        "/dev/ttyUSB\0",
    }
    for serial_port in serial_ports:
        gripper = {"serial_port": serial_port}
        driver.grippers.append(gripper)
        assert not driver.needs_synchronize(gripper)
