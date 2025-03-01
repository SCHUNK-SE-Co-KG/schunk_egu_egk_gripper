from ..schunk_gripper_library.driver import Driver


def test_driver_initializes_plc_data_buffers():
    driver = Driver()
    assert driver.plc_input_buffer.hex().upper() == "00" * 16
    assert driver.plc_output_buffer.hex().upper() == "00" * 16


def test_driver_supports_reading_plc_data_buffers():
    driver = Driver()
    input_buffer = "0011223344556677".upper() * 2
    output_buffer = "8899aabbccddeeff".upper() * 2
    driver.plc_input_buffer = bytearray(bytes.fromhex(input_buffer))
    driver.plc_output_buffer = bytearray(bytes.fromhex(output_buffer))
    assert driver.get_plc_input() == input_buffer
    assert driver.get_plc_output() == output_buffer


def test_driver_supports_setting_plc_data_buffers():
    driver = Driver()
    input_buffer = "0011223344556677".upper() * 2
    output_buffer = "8899aabbccddeeff".upper() * 2
    driver.set_plc_input(input_buffer)
    driver.set_plc_output(output_buffer)
    assert driver.plc_input_buffer.hex().upper() == input_buffer
    assert driver.plc_output_buffer.hex().upper() == output_buffer


def test_driver_rejects_setting_invalid_buffer_arguments():
    driver = Driver()
    invalid_buffers = [
        "0011",  # too short
        "00112233445566778899aabbccdd00112233",  # too long
        "0011223344556677hey!]]&% #?_-@§x",  # right size, but non-hex
    ]
    for buffer in invalid_buffers:
        assert not driver.set_plc_input(buffer)
        assert not driver.set_plc_output(buffer)


def test_check_for_non_hex_characters_behaves_as_expected():
    driver = Driver()
    hex_strings = ["012345", "abcde", "AAB"]
    non_hex_strings = ["\n$34", "0123X", "@?"]
    for string in hex_strings:
        assert not driver.contains_non_hex_chars(string)
    for string in non_hex_strings:
        assert driver.contains_non_hex_chars(string)
