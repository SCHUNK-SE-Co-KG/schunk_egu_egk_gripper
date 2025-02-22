from schunk_gripper_library.src.driver import Driver


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


def test_driver_supports_writing_plc_data_buffers():
    driver = Driver()
    input_buffer = "0011223344556677".upper() * 2
    output_buffer = "8899aabbccddeeff".upper() * 2
    driver.set_plc_input(input_buffer)
    driver.set_plc_output(output_buffer)
    assert driver.plc_input_buffer.hex().upper() == input_buffer
    assert driver.plc_output_buffer.hex().upper() == output_buffer
