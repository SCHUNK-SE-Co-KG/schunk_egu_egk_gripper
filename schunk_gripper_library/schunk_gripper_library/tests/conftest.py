import pytest
import yaml  # type: ignore
import os
from schunk_gripper_library.driver import Driver
from typing import Generator, List


DEVICES_CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'config.yaml')  # this file contains the device configs
DEVICE_CONFIG_ETHERNET_FIELDS = ['host', 'port']  # fields required for ethernet device configs
DEVICE_CONFIG_MODBUS_FIELDS = ['serial_port', 'device_id']  # fields required for modbus device configs
DEVICE_CONFIG_WORKPIECE_AT_POSITION = 'workpiece_at_position'  # optional field for workpiece position

workpiece_position_map: dict[Driver, int | None] = {}  # stores for each driver the workpiece at position (None if not specified)


def _load_device_configs() -> dict:
    """Loads and asserts that the device configs from `DEVICES_CONFIG_PATH` are valid.

    Returns:
        Dictionary of device configurations (device names are keys).
        The structure is described in the config file itself.
    """
    device_configs = {}
    try:
        with open(DEVICES_CONFIG_PATH, 'r') as f:
            device_configs = yaml.safe_load(f)
    except FileNotFoundError:
        return {}

    if not device_configs:
        return {}

    assert isinstance(device_configs, dict), "Device configs must be a dictionary."

    # check required fields for ethernet and modbus devices
    for name, config in device_configs.items():
        is_ethernet = all(field in config for field in DEVICE_CONFIG_ETHERNET_FIELDS)
        has_ethernet = any(field in config for field in DEVICE_CONFIG_ETHERNET_FIELDS)
        is_modbus = all(field in config for field in DEVICE_CONFIG_MODBUS_FIELDS)
        has_modbus = any(field in config for field in DEVICE_CONFIG_MODBUS_FIELDS)

        assert not (has_ethernet and has_modbus), (
            f"Device config '{name}' has conflicting fields for both ethernet and modbus"
        )
        assert (is_ethernet or is_modbus), (
            f"Device config '{name}' is missing required fields for either ethernet or modbus"
        )

        if DEVICE_CONFIG_WORKPIECE_AT_POSITION in config:
            assert isinstance(config[DEVICE_CONFIG_WORKPIECE_AT_POSITION], int), "workpiece_at_position must be an integer."
            assert config[DEVICE_CONFIG_WORKPIECE_AT_POSITION] >= 0, "workpiece_at_position must be non-negative."

    return device_configs


@pytest.fixture(scope="session")
def drivers() -> Generator[List[Driver], None, None]:
    """
    This fixture creates a driver instance for each device defined in the device config file.
    The drivers are connected based on their configuration (ethernet or modbus).
    If the device config file is empty or missing, no drivers are created and an empty list is returned.
    """
    device_configs = _load_device_configs()
    if not device_configs:
        yield []  # no device configs available
        return

    drivers = []
    for name, config in device_configs.items():
        driver = Driver()
        is_ethernet = all(field in config for field in DEVICE_CONFIG_ETHERNET_FIELDS)
        is_modbus = all(field in config for field in DEVICE_CONFIG_MODBUS_FIELDS)

        if is_ethernet:
            connected = driver.connect(host=config['host'], port=config['port'])

        if is_modbus:
            connected = driver.connect(serial_port=config['serial_port'], device_id=config['device_id'])

        assert connected, f"Failed to connect to device '{name}' at {driver.addr_str}."
        drivers.append(driver)
        workpiece_position_map[driver] = config.get(DEVICE_CONFIG_WORKPIECE_AT_POSITION, None)

    yield drivers  # provide drivers to tests

    for driver in drivers:  # disconnect drivers after all tests are done
        driver.disconnect()


@pytest.fixture(scope="function", autouse=True)
def acknowledge_drivers(drivers):
    """Acknowledges all drivers before each test function.
    """
    for driver in drivers:
        assert driver.acknowledge(), f"Failed to acknowledge driver at {driver.addr_str}."


@pytest.fixture(scope="function")
def hms_chip():
    from .utils.hms_chip import HMSChip
    gripper = HMSChip()
    gripper.power_on()
    yield None
    gripper.power_off()
