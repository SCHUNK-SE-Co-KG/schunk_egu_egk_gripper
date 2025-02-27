import pytest
from schunk_gripper_library.tests.etc.pseudo_terminals import Connection
from schunk_gripper_library.tests.etc.modbus_server import ModbusServer
import asyncio
from pathlib import Path


@pytest.fixture(scope="module")
def pseudo_terminals():
    connection = Connection()
    pt1, pt2 = connection.open()
    print(f"Opening two pseudo terminals:\n{pt1}\n{pt2}")

    yield (pt1, pt2)

    print("Closing both pseudo terminals")
    connection.close()


@pytest.fixture(scope="module")
def modbus_server(pseudo_terminals):
    pt1, pt2 = pseudo_terminals
    server = ModbusServer()
    print("Opening Modbus server")
    asyncio.run(server.setup(port=pt1))
    asyncio.run(server.start())

    yield pt2

    print("Closing Modbus server")
    server.stop()


def modbus_gripper_available():
    if Path("/dev/ttyUSB0").exists():
        return True
    return False


skip_without_gripper = pytest.mark.skipif(
    not modbus_gripper_available(), reason="No modbus gripper available"
)
