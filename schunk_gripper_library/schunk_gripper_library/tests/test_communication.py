from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper
import asyncio
import pytest


@skip_without_gripper
def test_driver_implements_connect_and_disconnect():
    driver = Driver()

    # Modbus
    device_id = 12  # SChUNK default
    assert driver.connect(serial_port="/dev/ttyUSB0", device_id=device_id)
    assert driver.mb_device_id == device_id
    assert driver.mb_client.connected
    assert driver.disconnect()
    assert not driver.mb_client.connected

    # TCP/IP
    host = "0.0.0.0"
    port = 8000
    assert driver.connect(host=host, port=port)
    assert driver.host == host
    assert driver.port == port
    assert driver.web_client is not None
    assert driver.disconnect()
    assert driver.web_client is None


def test_driver_rejects_invalid_connection_arguments():
    driver = Driver()

    # Modbus
    assert not driver.connect(serial_port=42)
    assert not driver.connect(serial_port=42, device_id=12)
    assert not driver.connect(serial_port="/dev/ttyUSB0")  # missing device_id
    assert not driver.connect(
        serial_port="/dev/ttyUSB0", device_id="12"
    )  # wrong device_id type
    assert not driver.connect(serial_port="not ok", device_id=-1)
    assert not driver.connect(serial_port="not ok", device_id=0)
    assert not driver.connect(
        serial_port="non-existent", device_id=12
    )  # non-existent port

    # TCP/IP
    assert not driver.connect(host="0.0.0.0", port=-10)
    assert not driver.connect(host="a.b.c.d")
    assert not driver.connect(host="1.3.3")
    assert not driver.connect(host="some arbitrary string #!?")

    # Wrong update cycles
    invalid_cycles = [-1, -0.001, 0.0, 0, 0.0001]
    for cycle in invalid_cycles:
        assert not driver.connect(update_cycle=cycle)


@skip_without_gripper
def test_driver_supports_repeated_connects_and_disconnects():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        for _ in range(3):
            assert driver.connect(
                host=host, port=port, serial_port=serial_port, device_id=12
            )
            assert driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_switching_between_protocols():
    driver = Driver()
    for _ in range(3):
        assert driver.connect(host="0.0.0.0", port=8000)
        assert driver.disconnect()
        assert driver.connect(port="/dev/ttyUSB0", device_id=12)
        assert driver.disconnect()


@skip_without_gripper
def test_driver_rejects_new_connect_without_disconnect():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        assert driver.connect(
            host=host, port=port, serial_port=serial_port, device_id=12
        )
        assert not driver.connect(host=host, port=port, device_id=12)
        driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_disconnects():
    driver = Driver()
    assert driver.disconnect()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        for _ in range(3):
            assert driver.disconnect()


@skip_without_gripper
def test_driver_implements_sending_plc_output():
    driver = Driver()
    driver.connect(port="/dev/ttyUSB0", device_id=12)
    assert driver.send_plc_output()
    driver.disconnect()
    driver.connect(host="0.0.0.0", port=8000)
    assert driver.send_plc_output()
    driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_sending():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        for _ in range(5):
            assert driver.send_plc_output()
        driver.disconnect()


@skip_without_gripper
def test_driver_rejects_sending_when_not_connected():
    driver = Driver()
    assert not driver.send_plc_output()


@skip_without_gripper
def test_driver_implements_receiving_plc_input():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        before = driver.get_plc_input()
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        assert driver.receive_plc_input()
        after = driver.get_plc_input()
        assert after != before
        driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_receiving():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        for _ in range(5):
            assert driver.receive_plc_input()
        driver.disconnect()


@skip_without_gripper
def test_driver_supports_waiting_for_desired_status():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)

        # Timeout for bitsets that don't come
        impossible_bits = {"0": 1, "7": 1}  # operational + error
        assert not asyncio.run(
            driver.wait_for_status(bits=impossible_bits, timeout_sec=0.1)
        )

        # Default timeout works
        impossible_bits = {"0": 1, "7": 1}
        assert not asyncio.run(driver.wait_for_status(bits=impossible_bits))

        # Success when bits match
        matching_bits = {"0": 0, "7": 1}  # error on startup
        assert asyncio.run(driver.wait_for_status(bits=matching_bits, timeout_sec=0.1))

        # Fails but survives invalid bits
        invalid_bits = {"33": 1, "-1": 0}
        assert not asyncio.run(
            driver.wait_for_status(bits=invalid_bits, timeout_sec=0.1)
        )

        # Fails but survives invalid timeouts
        matching_bits = {"0": 0, "7": 1}
        invalid_timeouts = [0.0, 0, -1.5]
        for timeout in invalid_timeouts:
            assert not asyncio.run(
                driver.wait_for_status(bits=matching_bits, timeout_sec=timeout)
            )

        # Fails with empty bits
        assert not asyncio.run(driver.wait_for_status(bits={}))

        # Async calls don't block
        async def wait() -> bool:
            matching_bits = {"0": 0, "7": 1}
            return await driver.wait_for_status(bits=matching_bits)

        async def test() -> bool:
            succeeded = await asyncio.gather(wait(), wait(), wait())
            return all(succeeded)

        assert asyncio.run(test())

        # Finish
        driver.disconnect()


@skip_without_gripper
def test_driver_supports_sending_and_receiving_after_switching_protocols():
    driver = Driver()
    for _ in range(3):
        for host, port, serial_port in zip(
            ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
        ):
            driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
            assert driver.send_plc_output()
            assert driver.receive_plc_input()
            driver.disconnect()


@skip_without_gripper
def test_driver_supports_reading_module_parameters():
    driver = Driver()

    # Can't read when not connected
    for param in driver.readable_parameters.keys():
        assert not driver.read_module_parameter(param=param)

    # Reject unsupported parameters
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        for param in ["not-ok", "?!#" "-1"]:
            driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
            assert not driver.read_module_parameter(param)
        driver.disconnect()

    # All params have the correct size
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        for key, value in driver.readable_parameters.items():
            result = driver.read_module_parameter(key)
            assert len(result) == value["registers"] * 2  # two bytes per register
        driver.disconnect()


@skip_without_gripper
def test_driver_supports_writing_module_parameters():
    driver = Driver()

    # Can't write when not connected
    data = bytearray()
    assert not driver.write_module_parameter("0x0048", data)

    # Reject unsupported parameters
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        for param in ["not-existent", "1234" "0x0"]:
            driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
            assert not driver.write_module_parameter(param, bytearray())
        driver.disconnect()

    # Data arguments must have the correct sizes
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)

        # Correct
        for key, value in driver.writable_parameters.items():
            byte_size = value["registers"] * 2
            data = bytearray(bytes.fromhex("00" * byte_size))
            assert driver.write_module_parameter(key, data)

        # Incorrect
        for key, value in driver.writable_parameters.items():
            data = bytearray()
            assert not driver.write_module_parameter(key, data)
        driver.disconnect()


@skip_without_gripper
def test_connected_driver_has_module_type():
    driver = Driver()
    assert not driver.module_type  # empty on startup

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        assert driver.module_type in driver.valid_module_types.values()

        driver.disconnect()
        assert not driver.module_type  # empty after disconnect


def test_driver_can_check_for_gpe_support():
    driver = Driver()
    assert not driver.gpe_available()  # when unconnected

    types_with_gpe = [
        "EGU_50_M_B",
        "EGK_25_M_B",
        "EZU_40_M_B",
        "xyz_00_M_B",
    ]
    types_without_gpe = [
        "EGU_80_N_B",
        "EGK_40_N_B",
        "EGU_70_N_SD",
        "EGH",
        "UG4_DIO_80",
        "_",
        "123x",
        "?#\0\n",
        "EGU_40_m_B",
        "0x0048",
        "EGU_M_40_B",
        "_EGU_40_M_B",
        "MMM_not_ok",
    ]

    for type in types_with_gpe:
        driver.module_type = type
        assert driver.gpe_available()

    for type in types_without_gpe:
        driver.module_type = type
        assert not driver.gpe_available()


def test_driver_estimates_duration_of_lasting_operations():
    driver = Driver()
    combinations = [
        # Invalid velocity arguments will lead to an invalid error,
        # and should return at once
        {
            "args": {"position_abs": 30500, "velocity": 0},
            "should_take": 0,
        },
        {
            "args": {"position_abs": 30500, "velocity": -123},
            "should_take": 0,
        },
        # Valid position and velocity arguments
        {
            "args": {"position_abs": 10000, "velocity": 5000},
            "should_take": 2.0,
        },
        {
            "args": {"position_abs": -10000, "velocity": 5000},
            "should_take": 2.0,
        },
        # Invalid force arguments
        {
            "args": {"force": 75.0},
            "should_take": 0.0,
        },
        {
            "args": {"force": -70},
            "should_take": 0.0,
        },
        {
            "args": {"force": 270},
            "should_take": 0.0,
        },
        # Valid force arguments
        {
            "args": {"force": 50},
            "should_take": 1.0,
        },
        {
            "args": {"force": 75},
            "should_take": 1.0,
        },
        {
            "args": {"force": 100},
            "should_take": 1.0,
        },
    ]

    for entry in combinations:
        duration_sec = driver.estimate_duration(**entry["args"])
        assert (
            pytest.approx(duration_sec) == entry["should_take"]
        ), f"args: {entry['args']}"
