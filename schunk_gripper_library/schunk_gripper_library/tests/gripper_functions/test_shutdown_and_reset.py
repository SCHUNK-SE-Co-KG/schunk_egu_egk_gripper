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

"""Unit tests for shutdown and reset gripper functions.

Tests cover:
    - Prepare for shutdown
    - Soft reset
"""

import time
from schunk_gripper_library.tests.utils.functions import skip_if_no_drivers


def test_prepare_for_shutdown_and_soft_reset(drivers):
    """Tests prepare_for_shutdown and soft_reset functions for all connected drivers.

    Both functions are tested jointly, because soft_reset can only be called
    after prepare_for_shutdown.
    """
    skip_if_no_drivers(drivers)

    for driver in drivers:
        assert driver.prepare_for_shutdown(), \
            f"Failed to prepare for shutdown. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
        assert driver.soft_reset(), \
            f"Failed to perform soft reset. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
        time.sleep(10)  # wait for reset to complete
