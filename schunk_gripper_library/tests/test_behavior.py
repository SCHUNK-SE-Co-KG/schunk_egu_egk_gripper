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
