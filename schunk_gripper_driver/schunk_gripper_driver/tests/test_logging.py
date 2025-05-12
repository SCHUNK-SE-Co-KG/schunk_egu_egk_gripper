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
from lifecycle_msgs.msg import Transition
import time
import pytest

LOG_SIZE_BYTES = 100


@skip_without_gripper
@pytest.mark.parametrize(
    "log_monitor", [{"max_log_size": LOG_SIZE_BYTES}], indirect=True
)
def test_driver_doesnt_fill_disk_space_by_default(log_monitor, lifecycle_interface):
    driver = lifecycle_interface

    for _ in range(10):

        driver.change_state(Transition.TRANSITION_CONFIGURE)
        driver.change_state(Transition.TRANSITION_ACTIVATE)

        time.sleep(0.1)

        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        driver.change_state(Transition.TRANSITION_CLEANUP)

    # log_monitor does the final asserting
