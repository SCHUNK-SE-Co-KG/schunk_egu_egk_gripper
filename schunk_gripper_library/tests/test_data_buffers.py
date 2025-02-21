from src.driver import Driver

# [1]: https://stb.cloud.schunk.com/media/IM0046706.PDF


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
