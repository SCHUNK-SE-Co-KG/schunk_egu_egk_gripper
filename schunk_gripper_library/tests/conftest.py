import pytest
from schunk_gripper_library.tests.etc.pseudo_terminals import Connection
from schunk_gripper_library.tests.etc.modbus_server import ModbusServer
import asyncio
import threading
from pathlib import Path
from httpx import Client, ConnectTimeout, ConnectError


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

    async def periodic_modbus_task(pt1, stop_event):
        server = ModbusServer()
        await server.setup(port=pt1)
        await server.start()

        try:
            while not stop_event.is_set():
                await asyncio.sleep(1)
        finally:
            server.stop()

    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=lambda: loop.run_forever())
    thread.start()

    stop_event = asyncio.Event()
    task = asyncio.run_coroutine_threadsafe(periodic_modbus_task(pt1, stop_event), loop)
    yield pt2

    stop_event.set()
    task.result()

    loop.call_soon_threadsafe(loop.stop)
    thread.join()
    loop.close()


def gripper_available():
    client = Client()
    try:
        webserver_up = client.get(
            "http://0.0.0.0:8000/adi/data.json", timeout=1.0
        ).is_success
    except (ConnectTimeout, ConnectError):
        webserver_up = False
    modbus_server_up = Path("/dev/ttyUSB0").exists() or ""
    if webserver_up and modbus_server_up:
        return True
    return False


skip_without_gripper = pytest.mark.skipif(
    not gripper_available(), reason="No gripper/simulator available"
)
