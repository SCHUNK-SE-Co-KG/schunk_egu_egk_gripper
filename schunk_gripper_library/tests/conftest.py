import pytest
import pytest_asyncio
from schunk_gripper_library.tests.etc.pseudo_terminals import Connection
from schunk_gripper_library.tests.etc.modbus_server import ModbusServer
import asyncio
import threading
from pathlib import Path


@pytest.fixture(scope="module")
def pseudo_terminals():
    connection = Connection()
    pt1, pt2 = connection.open()
    print(f"Opening two pseudo terminals:\n{pt1}\n{pt2}")

    yield (pt1, pt2)

    print("Closing both pseudo terminals")
    connection.close()


@pytest_asyncio.fixture(scope="module")
async def modbus_server(pseudo_terminals):
    pt1, pt2 = pseudo_terminals

    async def periodic_modbus_task(pt1, stop_event):
        server = ModbusServer()
        print("Opening and starting Modbus server on port:", pt1)
        await server.setup(port=pt1)
        await server.start()

        try:
            while not stop_event.is_set():
                print("Modbus server is running")
                await asyncio.sleep(1)
        finally:
            print("Closing Modbus server")
            server.stop()

    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=lambda: loop.run_forever())
    thread.start()

    stop_event = asyncio.Event()
    task = asyncio.run_coroutine_threadsafe(periodic_modbus_task(pt1, stop_event), loop)
    yield pt2

    stop_event.set()
    while not task.done():
        await asyncio.sleep(0.1)

    loop.call_soon_threadsafe(loop.stop)
    thread.join()
    loop.close()


def modbus_gripper_available():
    if Path("/dev/ttyUSB0").exists():
        return True
    return False


skip_without_gripper = pytest.mark.skipif(
    not modbus_gripper_available(), reason="No modbus gripper available"
)
