from schunk_gripper_library.src.driver import Driver


def test_driver_rejects_invalid_connection_arguments():
    driver = Driver()
    invalid_protocols = ["This", "is", "an", "invalid", "protocol"]
    for protocol in invalid_protocols:
        assert not driver.connect(protocol, "doesn't matter")

    # Procolo ok but remaining args wrong or missing
    assert not driver.connect("modbus", 42)  # wrong port type
    assert not driver.connect("modbus", 42, 12)
    assert not driver.connect("modbus", "/dev/ttyusb0")  # missing device_id
    assert not driver.connect("modbus", "/dev/ttyusb0", "12")  # wrong device_id type
    assert not driver.connect("modbus", "ok", -1)
    assert not driver.connect("modbus", "ok", 0)


def test_driver_connects_to_modbus_grippers(pseudo_terminals):
    driver = Driver()
    port = pseudo_terminals[0]
    device_id = 12  # SChUNK default
    assert driver.connect("modbus", port, device_id)
    assert driver.disconnect()
