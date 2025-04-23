from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper, Scheduler
import asyncio
import pytest


@skip_without_gripper
def test_acknowledge():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # Not connected
        assert not asyncio.run(driver.acknowledge())

        # Connected
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        assert asyncio.run(driver.acknowledge())

        # Repetitive
        for _ in range(5):
            assert asyncio.run(driver.acknowledge())

        driver.disconnect()


@skip_without_gripper
def test_fast_stop():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):

        # Not connected
        assert not asyncio.run(driver.fast_stop())

        # After fresh start
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        assert asyncio.run(driver.fast_stop())

        # From operational
        assert asyncio.run(driver.acknowledge())
        assert asyncio.run(driver.fast_stop())

        # Repetitive
        for _ in range(5):
            assert asyncio.run(driver.fast_stop())

        driver.disconnect()


@skip_without_gripper
def test_all_gripper_commands_run_with_a_scheduler():
    driver = Driver()
    scheduler = Scheduler()
    scheduler.start()

    # Only relevant for Modbus
    driver.connect(serial_port="/dev/ttyUSB0", device_id=12)

    # Commands
    assert asyncio.run(driver.fast_stop(scheduler=scheduler))

    assert asyncio.run(driver.acknowledge(scheduler=scheduler))

    assert asyncio.run(
        driver.move_to_absolute_position(
            position=12345, velocity=10000, scheduler=scheduler
        )
    ), f"driver status: {driver.get_status_diagnostics()}"

    driver.disconnect()
    scheduler.stop()


@skip_without_gripper
def test_move_to_absolute_position_fails_with_invalid_arguments():
    driver = Driver()

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # Invalid arguments
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        asyncio.run(driver.acknowledge())
        combinations = [
            {"position": 0.1, "velocity": 1000},
            {"position": -0.1, "velocity": 1000},
            {"position": 1000, "velocity": -1.234},
            {"position": 1000, "velocity": -5005},
            {"position": 1000, "velocity": 177.33},
            {"position": 1000, "velocity": 0},
            {"position": 1000, "velocity": 0.0},
        ]
        for args in combinations:
            assert not asyncio.run(driver.move_to_absolute_position(**args))
        driver.disconnect()


def test_move_to_absolute_position_fails_when_not_connected():
    driver = Driver()
    assert not asyncio.run(driver.move_to_absolute_position(position=100, velocity=100))


@skip_without_gripper
def test_move_to_absolute_position_succeeds_with_valid_arguments():
    driver = Driver()

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # Valid arguments
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        asyncio.run(driver.acknowledge())
        combinations = [
            # consider <min_vel>, <max_vel>, <min_pos>, <max_pos>
            {"position": 12300, "velocity": 6300},
            {"position": 5200, "velocity": 38706},
            {"position": 10, "velocity": 10200},
            {"position": 10633, "velocity": 8400},
        ]
        for args in combinations:
            assert asyncio.run(
                driver.move_to_absolute_position(**args)
            ), f"module status: {driver.get_status_diagnostics()}"
        driver.disconnect()


@skip_without_gripper
def test_move_to_absolute_position_uses_gpe_only_when_available():
    driver = Driver()
    driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    asyncio.run(driver.acknowledge())

    # Test with _N_B gripper (without GPE).
    # The driver should survive use_gpe = True
    assert asyncio.run(
        driver.move_to_absolute_position(position=5200, velocity=10000, use_gpe=True)
    )
    driver.disconnect()


@pytest.mark.skip
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


@pytest.mark.skip
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
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # Not connected
        assert not asyncio.run(
            driver.grip_workpiece(
                gripping_force=test_force, use_gpe=test_gpe, grip_inside=True
            )
        )

        assert driver.connect(
            host=host, port=port, serial_port=serial_port, device_id=12
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

        # Modbus testing not possible, hence this skip
        if serial_port:
            assert driver.disconnect()
            return

        assert asyncio.run(
            driver.grip_workpiece(
                gripping_force=test_force,
                use_gpe=test_gpe,
                grip_inside=True,
                gripping_velocity=test_vel,
            )
        )

        assert driver.disconnect()


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
