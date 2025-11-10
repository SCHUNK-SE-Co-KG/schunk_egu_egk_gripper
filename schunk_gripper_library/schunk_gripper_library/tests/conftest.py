import pytest
import yaml
import os
from .etc.pseudo_terminals import Connection
from .etc.hms_chip import HMSChip
from unittest.mock import patch
import httpx
import pymodbus
from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import Scheduler
from typing import Generator, List
import concurrent.futures
from pymodbus.client import ModbusSerialClient


DEVICES_CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'config.yaml') # this file contains the device configs
DEVICE_CONFIG_ETHERNET_FIELDS = ['host', 'port']  # fields required for ethernet device configs
DEVICE_CONFIG_MODBUS_FIELDS = ['serial_port', 'device_id'] # fields required for modbus device configs
DEVICE_CONFIG_WORKPIECE_AT_POSITION = 'workpiece_at_position'  # optional field for workpiece position

workpiece_position_map: dict[Driver, int | None] = {} # stores for each driver the workpiece at position (None if not specified)

def load_device_configs() -> dict:
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
def scheduler() -> Generator[Scheduler, None, None]:
    scheduler = Scheduler()
    # scheduler.start()
    yield scheduler
    # scheduler.stop()


@pytest.fixture(scope="session")
def drivers(scheduler) -> Generator[List[Driver], None, None]:
    """
    This fixture creates a driver instance for each device defined in the device config file.
    The drivers are connected based on their configuration (ethernet or modbus).
    If the device config file is empty or missing, no drivers are created and an empty list is returned.
    """
    device_configs = load_device_configs()
    if not device_configs:
        yield []  # no device configs available
        return

    drivers = []
    for name, config in device_configs.items():
        driver = Driver()
        is_ethernet = all(field in config for field in DEVICE_CONFIG_ETHERNET_FIELDS)
        is_modbus = all(field in config for field in DEVICE_CONFIG_MODBUS_FIELDS)
    
        if is_ethernet:
            connected = driver.connect(host=config['host'], port=config['port'], scheduler=scheduler)

        if is_modbus:
            connected = driver.connect(serial_port=config['serial_port'], 
                                       device_id=config['device_id'],
                                       scheduler=scheduler)

        assert connected, f"Failed to connect to device '{name}' at {driver.addr_str}."
        drivers.append(driver)
        workpiece_position_map[driver] = config.get(DEVICE_CONFIG_WORKPIECE_AT_POSITION, None)
    
    yield drivers  # provide drivers to tests

    for driver in drivers:  # disconnect drivers after all tests are done
        driver.disconnect()


def get_workpiece_position(driver: Driver) -> int | None:
    """Returns the workpiece position for the given driver in [mm].
    """
    assert driver in workpiece_position_map, f"Driver at '{driver.addr_str}' not recognized."
    return workpiece_position_map[driver]

def skip_if_no_drivers(drivers):
    if not drivers:
        pytest.skip("No devices configured")


@pytest.fixture(scope="function", autouse=True)
def acknowledge_drivers(drivers, scheduler):
    """Acknowledges all drivers before each test function.
    """
    for driver in drivers:
        assert driver.acknowledge(scheduler=scheduler), f"Failed to acknowledge driver at {driver.addr_str}."


@pytest.fixture(scope="session")
def executor(drivers):
    num_workers = len(drivers)
    with concurrent.futures.ThreadPoolExecutor(max_workers=num_workers) as pool:
        yield pool


@pytest.fixture
def simulate_httpx_failure():
    controller = {"exception": None}
    pass_through = httpx.Client.get

    def side_effect(self, *args, **kwargs):
        if controller["exception"] is not None:
            raise controller["exception"]
        return pass_through(self, *args, **kwargs)

    patcher = patch("httpx.Client.get", new=side_effect)
    patcher.start()

    yield controller

    patcher.stop()


@pytest.fixture
def simulate_pymodbus_failure():
    controller = {"exception": None}
    pass_through = ModbusSerialClient.read_holding_registers

    def side_effect(self, *args, **kwargs):
        if controller["exception"] is not None:
            raise controller["exception"]
        return pass_through(self, *args, **kwargs)

    patcher = patch(
        "pymodbus.client.ModbusSerialClient.read_holding_registers", new=side_effect
    )
    patcher.start()

    yield controller

    patcher.stop()


@pytest.fixture(scope="function")
def ethernet_gripper():
    gripper = HMSChip()
    gripper.power_on()
    yield None
    gripper.power_off()
