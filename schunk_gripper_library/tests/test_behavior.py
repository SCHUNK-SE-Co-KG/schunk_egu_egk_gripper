from ..schunk_gripper_library.driver import Driver
from ..tests.conftest import skip_without_gripper
import asyncio


@skip_without_gripper
def test_acknowledge():
    driver = Driver()
    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):
        # Not connected
        assert not asyncio.run(driver.acknowledge())

        # Connected
        driver.connect(host=host, port=port, device_id=12)
        assert asyncio.run(driver.acknowledge())

        # Repetitive
        for _ in range(5):
            assert asyncio.run(driver.acknowledge())

        driver.disconnect()


@skip_without_gripper
def test_fast_stop():
    driver = Driver()
    for host, port in zip(["0.0.0.0", None], [8000, "/dev/ttyUSB0"]):

        # Not connected
        assert not asyncio.run(driver.fast_stop())

        # After fresh start
        driver.connect(host=host, port=port, device_id=12)
        assert asyncio.run(driver.fast_stop())

        # From operational
        assert asyncio.run(driver.acknowledge())
        assert asyncio.run(driver.fast_stop())

        # Repetitive
        for _ in range(5):
            assert asyncio.run(driver.fast_stop())

        driver.disconnect()


@skip_without_gripper
def test_move_to_absolute_position():
    test_position = 30000
    test_velocity = 60000
    test_gpe = False

    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # not connected
        assert not asyncio.run(
            driver.move_to_absolute_position(
                position=test_position, velocity=test_velocity, use_gpe=test_gpe
            )
        )

        # after connection
        assert driver.connect(
            host=host, port=port, serial_port=serial_port, device_id=12
        )
        assert asyncio.run(driver.acknowledge())
        assert asyncio.run(
            driver.move_to_absolute_position(
                position=test_position, velocity=test_velocity, use_gpe=test_gpe
            )
        )

        assert driver.disconnect()


@skip_without_gripper
def test_move_to_relative_position():
    test_position = -50000
    test_velocity = 73000
    test_gpe = True

    driver = Driver()

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # not connected
        assert not asyncio.run(
            driver.move_to_relative_position(
                position=test_position, velocity=test_velocity, use_gpe=test_gpe
            )
        )

        # after connection
        assert driver.connect(
            host=host, port=port, serial_port=serial_port, device_id=12
        )
        assert asyncio.run(driver.acknowledge())
        assert asyncio.run(
            driver.move_to_relative_position(
                position=test_position, velocity=test_velocity, use_gpe=test_gpe
            )
        )

        assert driver.disconnect()


@skip_without_gripper
def test_stop():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # Not connected
        assert not asyncio.run(driver.stop())

        # after connection
        assert driver.connect(
            host=host, port=port, serial_port=serial_port, device_id=12
        )
        assert asyncio.run(driver.acknowledge())

        assert asyncio.run(driver.stop())
        assert driver.disconnect()


@skip_without_gripper
def test_grip_workpiece():
    test_force = 60  # %
    test_gpe = True
    test_vel = 0
    driver = Driver()

    # Not connected
    assert not asyncio.run(
        driver.grip_workpiece(
            gripping_force=test_force, use_gpe=test_gpe, grip_inside=True
        )
    )

    # Connection to the Webserver
    assert driver.connect(host="0.0.0.0", port=8000, serial_port=None)

    assert asyncio.run(driver.acknowledge())

    assert not asyncio.run(
        driver.grip_workpiece(
            gripping_force=test_force,
            use_gpe=test_gpe,
            grip_inside=False,
            grip_outside=False,
        )
    )
    assert not asyncio.run(
        driver.grip_workpiece(
            gripping_force=test_force,
            use_gpe=test_gpe,
            grip_inside=True,
            grip_outside=True,
        )
    )

    assert asyncio.run(
        driver.grip_workpiece(
            gripping_force=test_force,
            use_gpe=test_gpe,
            grip_inside=True,
            gripping_velocity=test_vel,
        )
    )

    assert driver.disconnect()

    """
    try:
        # Connection to the Modbus
        assert driver.connect(
            host=None, port=None, serial_port="/dev/ttyUSB0", device_id=12
        )

        assert asyncio.run(driver.acknowledge())

        assert not asyncio.run(
            driver.grip_workpiece(
                gripping_force=test_force,
                use_gpe=test_gpe,
                grip_inside=False,
                grip_outside=False,
            )
        )
        assert asyncio.run(driver.acknowledge())
        assert not asyncio.run(
            driver.grip_workpiece(
                gripping_force=test_force,
                use_gpe=test_gpe,
                grip_inside=True,
                grip_outside=True,
            )
        )
        assert asyncio.run(driver.acknowledge())
        asyncio.run(
            driver.grip_workpiece(
                gripping_force=test_force,
                use_gpe=test_gpe,
                grip_inside=True,
                gripping_velocity=test_vel,
            )
        )

        driver.print_bits([3,6,11,12,4,16])
        desired_bits = {"11":1}
        time.sleep(4)
        assert asyncio.run(driver.wait_for_status(bits=desired_bits))

        assert driver.disconnect()
    except Exception as e:
        raise e
    finally:
        driver.disconnect()
    """


@skip_without_gripper
def test_release():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        if serial_port:
            return
        # not connected
        assert not asyncio.run(driver.release_workpiece())

        # after connection
        assert driver.connect(
            host=host, port=port, serial_port=serial_port, device_id=12
        )
        assert asyncio.run(driver.acknowledge())

        assert asyncio.run(driver.release_workpiece())

        assert driver.disconnect()
