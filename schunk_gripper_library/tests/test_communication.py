from ..schunk_gripper_library.driver import Driver
from ..tests.conftest import skip_without_gripper
import asyncio


@skip_without_gripper
def test_driver_implements_connect_and_disconnect():
    driver = Driver()

    # Modbus
    device_id = 12  # SChUNK default
    assert driver.connect(port="/dev/ttyUSB0", device_id=device_id)
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
    assert not driver.connect(port=42)  # no host
    assert not driver.connect(port=42, device_id=12)
    assert not driver.connect(port="/dev/ttyUSB0")  # missing device_id
    assert not driver.connect(
        port="/dev/ttyUSB0", device_id="12"
    )  # wrong device_id type
    assert not driver.connect(port="not ok", device_id=-1)
    assert not driver.connect(port="not ok", device_id=0)
    assert not driver.connect(port="non-existent", device_id=12)  # non-existent port

    # TCP/IP
    assert not driver.connect(host="0.0.0.0", port=-10)
    assert not driver.connect(host="a.b.c.d")
    assert not driver.connect(host="1.3.3")
    assert not driver.connect(host="some arbitrary string #!?")

    # Wrong update cycles
    invalid_cycles = [-1, -0.001, 0.0, 0, 0.0001]
    for cycle in invalid_cycles:
        assert not driver.connect("/dev/ttyUSB0", device_id=12, update_cycle=cycle)


@skip_without_gripper
def test_driver_supports_repeated_connects_and_disconnects():
    driver = Driver()
    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
        for _ in range(3):
            assert driver.connect(host=host, port=port, device_id=12)
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
    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
        assert driver.connect(host=host, port=port, device_id=12)
        assert not driver.connect(host=host, port=port, device_id=12)
        driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_disconnects():
    driver = Driver()
    assert driver.disconnect()
    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
        driver.connect(host=host, port=port, device_id=12)
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
    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
        driver.connect(host=host, port=port, device_id=12)
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
    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
        before = driver.get_plc_input()
        driver.connect(host=host, port=port, device_id=12)
        assert driver.receive_plc_input()
        after = driver.get_plc_input()
        assert after != before
        driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_receiving():
    driver = Driver()
    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
        driver.connect(host=host, port=port, device_id=12)
        for _ in range(5):
            assert driver.receive_plc_input()
        driver.disconnect()


@skip_without_gripper
def test_driver_supports_waiting_for_desired_status():
    driver = Driver()
    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
        driver.connect(host=host, port=port, device_id=12)

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
        for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
            driver.connect(host=host, port=port, device_id=12)
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
    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
        for param in ["not-ok", "?!#" "-1"]:
            driver.connect(host=host, port=port, device_id=12)
            assert not driver.read_module_parameter(param)
        driver.disconnect()

    # All params have the correct size
    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
        driver.connect(host=host, port=port, device_id=12)
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
    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
        for param in ["not-existent", "1234" "0x0"]:
            driver.connect(host=host, port=port, device_id=12)
            assert not driver.write_module_parameter(param, bytearray())
        driver.disconnect()

    # Data arguments must have the correct sizes
    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
        driver.connect(host=host, port=port, device_id=12)

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

    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
        driver.connect(host=host, port=port, device_id=12)
        assert driver.module_type in driver.valid_module_types.values()

        driver.disconnect()
        assert not driver.module_type  # empty after disconnect
