from schunk_gripper_library.src.driver import Driver


def test_driver_implements_connect(pseudo_terminals):
    driver = Driver()
    port = pseudo_terminals[0]
    device_id = 12  # SChUNK default
    assert driver.connect("modbus", port, device_id)
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


def test_driver_rejects_repeated_connects(pseudo_terminals):
    driver = Driver()
    port = pseudo_terminals[0]
    device_id = 42
    driver.connect("modbus", port, device_id)
    for _ in range(3):
        assert not driver.connect("modbus", port, device_id)
        driver.disconnect()


def test_driver_rejects_new_connect_without_disconnect(pseudo_terminals):
    driver = Driver()
    port = pseudo_terminals[0]
    other_port = pseudo_terminals[1]
    assert driver.connect("modbus", port, 12)
    assert not driver.connect("modbus", other_port[1], 34)
    driver.disconnect()


def test_driver_supports_repeated_disconnects(pseudo_terminals):
    driver = Driver()
    assert driver.disconnect()
    driver.connect("modbus", pseudo_terminals[0], 123)
    for _ in range(3):
        assert driver.disconnect()


def test_driver_implements_sending_plc_output(modbus_server):
    driver = Driver()
    driver.connect("modbus", modbus_server, 12)
    assert driver.send_plc_output()
