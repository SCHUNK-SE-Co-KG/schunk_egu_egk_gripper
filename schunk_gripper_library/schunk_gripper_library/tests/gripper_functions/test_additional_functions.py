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

"""Unit tests for additional gripper functions.

Tests cover:
    - Acknowledge
    - Brake Test
"""
from schunk_gripper_library.tests.utils.functions import skip_if_no_drivers


def test_acknowledge(drivers):
    skip_if_no_drivers(drivers)

    for driver in drivers:
        assert driver.acknowledge(), \
            f"Failed to acknowledge driver. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"


def test_brake_test(drivers):
    skip_if_no_drivers(drivers)

    for driver in drivers:
        if not driver.gpe_available():
            return  # brake test not available for this gripper

        # for brake test, the gripper must be standing still and not holding any workpiece.
        driver.release()  # this may fail if no workpiece is held, but that's ok

        assert driver.brake_test(), \
            f"Failed to perform brake test. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
