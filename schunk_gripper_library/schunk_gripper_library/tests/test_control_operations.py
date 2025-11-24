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

from schunk_gripper_library.driver import Driver
import pytest


def test_driver_supports_reading_and_writing_control_bits():
    driver = Driver()
    for bit in driver.valid_control_bits:
        driver.set_control_bit(bit=bit, value=True)
        result = driver.get_control_bit(bit=bit)
        assert isinstance(result, int)  # successful calls get the bit's value
        assert result == 1


def test_driver_rejects_writing_reserved_control_bits():
    driver = Driver()
    invalid_bits = [-1, 999]
    for bit in invalid_bits + driver.reserved_control_bits:
        assert not driver.set_control_bit(bit, True)


def test_driver_rejects_reading_reserved_or_invalid_control_bits():
    driver = Driver()
    invalid_bits = [-1, 32]
    for bit in driver.reserved_control_bits + invalid_bits:
        assert isinstance(driver.get_control_bit(bit), bool)  # call fails
        assert not driver.get_control_bit(bit)


def test_driver_supports_toggling_control_bits():
    driver = Driver()
    for bit in driver.valid_control_bits:
        before = driver.get_control_bit(bit)
        driver.toggle_control_bit(bit=bit)
        after = driver.get_control_bit(bit)
        assert after != before
        driver.toggle_control_bit(bit=bit)
        assert driver.get_control_bit(bit=bit) == before


def test_driver_rejects_toggling_reserved_or_invalid_control_bits():
    driver = Driver()
    invalid_bits = [-1, 32]
    for bit in driver.reserved_control_bits + invalid_bits:
        assert not driver.toggle_control_bit(bit)


def test_driver_only_touches_specified_control_bits():
    driver = Driver()
    before = driver.get_plc_output()
    for bit in driver.valid_control_bits:
        initial_value = driver.get_control_bit(bit=bit)
        driver.set_control_bit(bit=bit, value=True)
        driver.set_control_bit(bit=bit, value=bool(initial_value))

    assert driver.get_plc_output() == before


def test_driver_supports_reading_and_writing_target_position():
    driver = Driver()

    invalid_positions = [12.34, -0.5, -7500.0, "17.3"]
    for pos in invalid_positions:
        try:
            driver.set_target_position(pos)
            assert False, f"Setting target position with wrong type (value: '{pos}') should have raised an exception."
        except Exception:
            pass

    valid_positions = [12300, -15001, 1, 0]  # um
    for target in valid_positions:
        driver.set_target_position(target)
        assert pytest.approx(driver.get_target_position(), rel=1e-3) == target


def test_driver_supports_reading_and_writing_target_speed():
    driver = Driver()

    invalid_speeds = [12.34, -0.5, -200, "0.3"]
    for speed in invalid_speeds:
        try:
            driver.set_target_speed(speed)
            assert False, f"Setting target speed with wrong type (value: '{speed}') should have raised an exception."
        except Exception:
            pass

    valid_speed = 55300  # um/s
    driver.set_target_speed(valid_speed)
    assert driver.get_target_speed() == valid_speed


def test_driver_supports_reading_and_writing_gripping_force():
    driver = Driver()
    invalid_forces = [0.0, 0.75, 60.0, "80%"]
    for force in invalid_forces:
        try:
            driver.set_gripping_force(force)
            assert False, f"Setting gripping force with wrong type (value: '{force}') should have raised an exception."
        except Exception:
            pass

    valid_forces = [60, 75, 100]
    for force in valid_forces:
        assert driver.set_gripping_force(force)
        assert driver.get_gripping_force() == force
