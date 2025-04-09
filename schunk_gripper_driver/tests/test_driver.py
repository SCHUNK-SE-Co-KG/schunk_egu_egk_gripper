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
    entries = ["host", "port", "serial_port", "device_id", "driver"]
    for entry in entries:
        assert entry in driver.grippers[0]
    assert driver.grippers[0]["driver"] is None


@skip_without_gripper
def test_driver_connects_each_gripper_on_configure(ros2):
    driver = Driver("driver")
    driver.on_configure(state=None)
    for gripper in driver.grippers:
        assert isinstance(gripper["driver"], GripperDriver)
