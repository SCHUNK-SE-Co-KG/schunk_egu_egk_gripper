from schunk_gripper_library.src.driver import Driver
from threading import Thread


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


def test_accessing_input_output_buffers_is_threadsafe():
    # Write concurrently into the data buffers and check whether
    # their content is consistent

    driver = Driver()
    nr_accesses = 10000
    all_zeros = "00" * 16
    all_ones = "FF" * 16

    def write(buffer: str):
        for _ in range(nr_accesses):
            driver.set_plc_input(buffer)
            driver.set_plc_output(buffer)

    def check_data_integrity():
        for _ in range(nr_accesses):
            input = driver.get_plc_input()
            output = driver.get_plc_output()
            assert input == all_zeros or input == all_ones
            assert output == all_zeros or output == all_ones

    updating_threads = []
    for i in range(10):
        if i % 2 == 0:
            buffer = all_zeros
        else:
            buffer = all_ones
        thread = Thread(target=write, args=(buffer,))
        thread.start()
        updating_threads.append(thread)

    client_thread = Thread(target=check_data_integrity)
    client_thread.start()

    client_thread.join()
    for thread in updating_threads:
        thread.join()
