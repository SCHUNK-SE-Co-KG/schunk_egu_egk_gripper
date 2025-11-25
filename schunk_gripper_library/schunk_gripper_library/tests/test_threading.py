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
from threading import Thread
from schunk_gripper_library.tests.utils.functions import skip_if_no_drivers


def test_writing_entire_buffers_keeps_data_consistent():
    # Write concurrently into the data buffers and check whether
    # their content is consistent

    driver = Driver()
    nr_accesses = 1000
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
        thread = Thread(target=write, args=(buffer,), daemon=True)
        thread.start()
        updating_threads.append(thread)

    client_thread = Thread(target=check_data_integrity, daemon=True)
    client_thread.start()

    client_thread.join()
    for thread in updating_threads:
        thread.join()


def test_concurrent_input_buffer_reads_dont_deadlock():
    driver = Driver()
    nr_iterations = 100

    def read():
        for n in range(nr_iterations):
            driver.get_error_code()
            driver.get_warning_code()
            driver.get_additional_code()
            driver.get_status_diagnostics()
            for bit in driver.valid_status_bits:
                driver.get_status_bit(bit)

    threads = []
    for i in range(10):
        thread = Thread(target=read, daemon=True)
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()
        assert not thread.is_alive()


def test_concurrent_output_buffer_reads_and_writes_dont_deadlock():
    driver = Driver()
    nr_iterations = 100

    def read():
        for n in range(nr_iterations):
            driver.get_target_position()
            driver.get_target_speed()
            for bit in driver.valid_control_bits:
                driver.get_control_bit(bit)

    def write():
        for n in range(nr_iterations):
            driver.set_target_position(12345)
            driver.set_target_speed(1234)
            for bit in driver.valid_control_bits:
                driver.set_control_bit(bit, True)
                driver.toggle_control_bit(bit)

    threads = []
    for i in range(10):
        reading_thread = Thread(target=read, daemon=True)
        reading_thread.start()
        threads.append(reading_thread)

        writing_thread = Thread(target=write, daemon=True)
        writing_thread.start()
        threads.append(writing_thread)

    for thread in threads:
        thread.join()
        assert not thread.is_alive()


def test_concurrent_receive_calls_dont_deadlock(drivers):
    skip_if_no_drivers(drivers)

    for driver in drivers:
        nr_iterations = 10

        def receive():
            for n in range(nr_iterations):
                assert driver.receive_plc_input()

        threads = []
        for i in range(10):
            thread = Thread(target=receive, daemon=True)
            thread.start()
            threads.append(thread)

        for thread in threads:
            thread.join()
            assert not thread.is_alive()


def test_concurrent_parameter_reads_and_writes_dont_deadlock(drivers):
    skip_if_no_drivers(drivers)

    for driver in drivers:
        nr_iterations = 10

        def read():
            for n in range(nr_iterations):
                assert driver.read_param(param="0x0500")  # read module type

        def write():
            for n in range(nr_iterations):
                assert driver.write_param(
                    param="0x0048", data=bytearray(bytes.fromhex("00" * 16))  # write plc_output (control word)
                )

        threads = []
        for i in range(10):
            reading_thread = Thread(target=read, daemon=True)
            reading_thread.start()
            threads.append(reading_thread)

            writing_thread = Thread(target=write, daemon=True)
            writing_thread.start()
            threads.append(writing_thread)

        for thread in threads:
            thread.join()
            assert not thread.is_alive()
