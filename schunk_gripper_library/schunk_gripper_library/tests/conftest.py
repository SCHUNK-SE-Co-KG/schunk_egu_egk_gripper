import pytest
from .etc.pseudo_terminals import Connection
from .etc.modbus_server import ModbusServer
import asyncio
import threading
import time
from .etc.scanner_helper import stop_all


@pytest.fixture
def time_scan():
    """
    Fixture to measure the execution time of the scanner.scan function.
    """

    def measure_scan_time(scanner, *args, **kwargs):
        start_time = time.time()
        result = asyncio.run(scanner.scan(*args, **kwargs))
        end_time = time.time()
        execution_time = end_time - start_time
        return result, execution_time

    return measure_scan_time


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


@pytest.fixture()
def cleanup():
    assert (
        stop_all() is None
    ), "Failed to stop all scanners and drivers after test execution."
