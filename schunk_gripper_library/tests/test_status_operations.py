from ..schunk_gripper_library.driver import Driver
import struct
import pytest


def test_driver_supports_reading_status_bits():
    driver = Driver()
    for bit in driver.valid_status_bits:
        driver._set_status_bit(bit=bit, value=True)
        result = driver.get_status_bit(bit=bit)
        assert isinstance(result, int)  # successful calls get the bit's value
        assert result == 1


def test_driver_rejects_reading_reserved_or_invalid_status_bits():
    driver = Driver()
    invalid_bits = [-1, 32]
    for bit in driver.reserved_status_bits + invalid_bits:
        assert isinstance(driver.get_status_bit(bit), bool)  # call fails
        assert not driver.get_status_bit(bit)


def test_driver_supports_reading_error_code():
    driver = Driver()
    error_codes = ["AA", "bb", "0xcc"]
    expected = ["0xAA", "0xBB", "0xCC"]
    for error, expected in zip(error_codes, expected):
        driver.plc_input_buffer[driver.error_byte] = int(error, 16)
        assert driver.get_error_code() == expected


def test_driver_supports_reading_warning_code():
    driver = Driver()
    warning_codes = ["A0", "c1", "0xef"]
    expected = ["0xA0", "0xC1", "0xEF"]
    for warning, expected in zip(warning_codes, expected):
        driver.plc_input_buffer[driver.warning_byte] = int(warning, 16)
        assert driver.get_warning_code() == expected


def test_driver_supports_reading_additional_code():
    driver = Driver()
    additional_code = "0xEF"
    driver.plc_input_buffer[driver.additional_byte] = int(additional_code, 16)
    assert driver.get_additional_code() == additional_code


def test_driver_supports_reading_full_diagnostics():
    driver = Driver()
    everything_ok = "error_code: 0x0, warning_code: 0x0, additional_code: 0x0"
    assert driver.get_status_diagnostics() == everything_ok


def test_driver_supports_reading_actual_position():
    driver = Driver()
    pos = 12300  # um
    driver.plc_input_buffer[4:8] = bytes(struct.pack("i", pos))
    assert pytest.approx(driver.get_actual_position()) == pos
