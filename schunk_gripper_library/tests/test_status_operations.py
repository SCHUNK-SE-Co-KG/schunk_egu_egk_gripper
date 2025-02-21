from src.driver import Driver
import struct
import pytest


def test_driver_supports_reading_status_bits():
    driver = Driver()
    for bit in driver.valid_status_bits:
        driver._set_status_bit(bit=bit, value=True)
        result = driver.get_status_bit(bit=bit)
        assert isinstance(result, int)  # successful calls get the bit's value
        assert result == 1


def test_driver_rejects_reading_reserved_status_bits():
    driver = Driver()
    for bit in driver.reserved_status_bits:
        assert isinstance(driver.get_status_bit(bit), bool)  # call fails
        assert not driver.get_status_bit(bit)


def test_driver_supports_reading_status_error():
    driver = Driver()
    error_codes = ["AA", "bb", "0xcc"]
    expected = ["AA", "BB", "CC"]
    for error, expected in zip(error_codes, expected):
        driver.plc_input_buffer[driver.error_byte] = int(error, 16)
        assert driver.get_status_error() == expected


def test_driver_supports_reading_status_diagnostics():
    driver = Driver()
    diagnostics_code = "EF"
    driver.plc_input_buffer[driver.diagnostics_byte] = int(diagnostics_code, 16)
    assert driver.get_status_diagnostics() == diagnostics_code


def test_driver_supports_reading_actual_position():
    driver = Driver()
    pos = 12300  # um
    driver.plc_input_buffer[4:8] = bytes(struct.pack("i", pos))
    assert pytest.approx(driver.get_actual_position()) == pos
