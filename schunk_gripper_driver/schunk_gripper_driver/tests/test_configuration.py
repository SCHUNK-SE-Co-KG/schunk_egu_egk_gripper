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
import json
from pathlib import Path
from unittest import mock


def test_driver_offers_saving_gripper_configuration(ros2):
    driver = Driver("driver")

    # When empty
    driver.reset_grippers()
    assert not driver.save_configuration()

    # Non-writable locations
    invalid_locations = ["/", "/dev"]
    for location in invalid_locations:
        assert not driver.save_configuration(location=location), f"location: {location}"

    # When filled
    config = {"host": "1.2.3.4", "port": 77, "serial_port": "abc", "device_id": 42}
    driver.add_gripper(**config)
    assert driver.save_configuration()


def test_driver_offers_loading_previous_gripper_configuration(ros2):
    driver = Driver("driver")
    driver.reset_grippers()

    # Nothing previously saved
    assert not driver.load_previous_configuration(location="non-existent")
    assert len(driver.grippers) == 0

    # With saved configuration
    config = {"host": "1.2.3.4", "port": 77, "serial_port": "abc", "device_id": 42}
    driver.add_gripper(**config)
    assert driver.save_configuration()
    driver.reset_grippers()

    assert driver.load_previous_configuration()
    assert len(driver.grippers) == 1
    for key, value in config.items():
        assert driver.grippers[0][key] == value


def test_driver_rejects_loading_invalid_configuration(ros2):
    driver = Driver("driver")
    driver.reset_grippers()

    invalid_configurations = [
        {"host": "", "port": 0, "serial_port": "", "device_id": 0},  # all empty
        {"host": "", "port": 0, "serial_port": ""},  # missing entry
        {"hots": "", "prot": 0, "sirial_port": "", "divice_id": 0},  # typos
        {
            "host": "",
            "port": "not-ok",
            "serial_port": "",
            "device_id": "not-ok",
        },  # wrong types
        {},  # completely empty
        [
            {"host": "0.0.0.0", "port": 8000, "serial_port": "", "device_id": 0}
        ],  # valid entries, but nested
    ]

    location = Path("/tmp/schunk_gripper")

    for config in invalid_configurations:
        with open(location.joinpath("configuration.json"), "w") as f:
            json.dump([config], f)

        assert not driver.load_previous_configuration(location=location)
        assert len(driver.grippers) == 0


def test_driver_rejects_loading_incorrectly_formatted_configuration(ros2):
    driver = Driver("driver")
    driver.reset_grippers()

    location = Path("/tmp/schunk_gripper")

    # With nonsense content
    with open(location.joinpath("configuration.json"), "w") as f:
        f.write("This should not crash the driver! #^$%&\0\n")

    assert not driver.load_previous_configuration(location=location)
    assert len(driver.grippers) == 0

    # Simulate permission errors
    with mock.patch("builtins.open", side_effect=PermissionError()):
        assert not driver.load_previous_configuration(location=location)
        assert len(driver.grippers) == 0
