from ..schunk_gripper_library.driver import Driver
from ..tests.conftest import skip_without_gripper
import asyncio


@skip_without_gripper
def test_driver_implements_connect():

    # Modbus
    driver = Driver()
    device_id = 12  # SChUNK default
    assert driver.connect(port="/dev/ttyUSB0", device_id=device_id)
    assert driver.mb_device_id == device_id
    assert driver.disconnect()

    # TCP/IP
    assert driver.connect(host="0.0.0.0", port=8000)
    assert driver.disconnect()


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

    # Wrong update cycles
    invalid_cycles = [-1, -0.001, 0.0, 0, 0.0001]
    for cycle in invalid_cycles:
        assert not driver.connect("/dev/ttyUSB0", device_id=12, update_cycle=cycle)


@skip_without_gripper
def test_driver_supports_repeated_connects_and_disconnects():
    driver = Driver()
    # TCP/IP
    for _ in range(3):
        assert driver.connect(host="0.0.0.0", port=8000)
        assert driver.disconnect()
    # Modbus
    for _ in range(3):
        assert driver.connect(port="/dev/ttyUSB0", device_id=12)
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
    assert driver.connect(port="/dev/ttyUSB0", device_id=12)
    assert not driver.connect(port="/dev/ttyUSB0", device_id=34)
    driver.disconnect()
    assert driver.connect(host="0.0.0.0", port=8000)
    assert not driver.connect(host="0.0.0.0", port=8000)
    driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_disconnects():
    driver = Driver()
    assert driver.disconnect()
    driver.connect(port="/dev/ttyUSB0", device_id=12)
    for _ in range(3):
        assert driver.disconnect()
    driver.connect(host="0.0.0.0", port=8000)
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
def test_driver_supports_repeated_sending_without_sleep():
    driver = Driver()
    driver.connect(port="/dev/ttyUSB0", device_id=12)
    for _ in range(5):
        assert driver.send_plc_output()
    driver.disconnect()
    assert False  # Implement TCP/IP support


@skip_without_gripper
def test_driver_rejects_sending_when_not_connected():
    driver = Driver()
    assert not driver.send_plc_output()
    assert False  # Implement TCP/IP support


@skip_without_gripper
def test_driver_implements_receiving_plc_input():
    driver = Driver()
    before = driver.get_plc_input()
    driver.connect(port="/dev/ttyUSB0", device_id=12)
    assert driver.receive_plc_input()
    after = driver.get_plc_input()
    assert after != before
    driver.disconnect()
    assert False  # Implement TCP/IP support


@skip_without_gripper
def test_driver_supports_repeated_receiving_without_sleep():
    driver = Driver()
    driver.connect(port="/dev/ttyUSB0", device_id=12)
    for _ in range(5):
        assert driver.receive_plc_input()
    driver.disconnect()
    assert False  # Implement TCP/IP support


@skip_without_gripper
def test_driver_supports_waiting_for_desired_status():
    driver = Driver()
    driver.connect(port="/dev/ttyUSB0", device_id=12)

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
    assert not asyncio.run(driver.wait_for_status(bits=invalid_bits, timeout_sec=0.1))

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

    assert False  # Implement TCP/IP support
