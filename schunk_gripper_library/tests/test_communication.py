from ..schunk_gripper_library.driver import Driver
from ..tests.conftest import skip_without_gripper


@skip_without_gripper
def test_driver_implements_connect():
    driver = Driver()
    device_id = 12  # SChUNK default
    assert driver.connect("modbus", "/dev/ttyUSB0", device_id)
    assert driver.mb_device_id == device_id
    assert driver.disconnect()


def test_driver_rejects_invalid_connection_arguments():
    driver = Driver()
    invalid_protocols = ["This", "is", "an", "invalid", "protocol"]
    for protocol in invalid_protocols:
        assert not driver.connect(protocol, "doesn't matter")

    # Protocol ok but remaining args wrong or missing
    assert not driver.connect("modbus", 42)  # wrong port type
    assert not driver.connect("modbus", 42, 12)
    assert not driver.connect("modbus", "/dev/ttyusb0")  # missing device_id
    assert not driver.connect("modbus", "/dev/ttyusb0", "12")  # wrong device_id type
    assert not driver.connect("modbus", "ok", -1)
    assert not driver.connect("modbus", "ok", 0)

    # Arguments ok, but non-existent modbus port
    assert not driver.connect("modbus", "non-existent", 12)

    # Wrong update cycles
    invalid_cycles = [-1, -0.001, 0.0, 0, 0.0001]
    for cycle in invalid_cycles:
        assert not driver.connect("modbus", "/dev/ttyUSB0", 12, cycle)


@skip_without_gripper
def test_driver_supports_repeated_connects_and_disconnects():
    driver = Driver()
    for _ in range(3):
        assert driver.connect("modbus", "/dev/ttyUSB0", 12)
        assert driver.disconnect()


@skip_without_gripper
def test_driver_rejects_new_connect_without_disconnect():
    driver = Driver()
    assert driver.connect("modbus", "/dev/ttyUSB0", 12)
    assert not driver.connect("modbus", "/dev/ttyUSB0", 34)
    driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_disconnects():
    driver = Driver()
    assert driver.disconnect()
    driver.connect("modbus", "/dev/ttyUSB0", 12)
    for _ in range(3):
        assert driver.disconnect()


@skip_without_gripper
def test_driver_implements_sending_plc_output():
    driver = Driver()
    driver.connect("modbus", "/dev/ttyUSB0", 12)
    assert driver.send_plc_output()
    driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_sending_without_sleep():
    driver = Driver()
    driver.connect("modbus", "/dev/ttyUSB0", 12)
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
    before = driver.get_plc_input()
    driver.connect("modbus", "/dev/ttyUSB0", 12)
    assert driver.receive_plc_input()
    after = driver.get_plc_input()
    assert after != before
    driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_receiving_without_sleep():
    driver = Driver()
    driver.connect("modbus", "/dev/ttyUSB0", 12)
    for _ in range(5):
        assert driver.receive_plc_input()
    driver.disconnect()
