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
