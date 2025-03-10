#!/usr/bin/env python3
import sys

sys.path.append("..")
from tests.etc.pseudo_terminals import Connection  # noqa: E402
from tests.etc.modbus_server import ModbusServer  # noqa: E402
import asyncio  # noqa: E402


async def main():
    connection = Connection()
    pt1, pt2 = connection.open()
    server = ModbusServer()
    print(f"Opening Modbus server on {pt2}")

    async def start_server():
        print("Before start")
        await server.setup(port=pt1)
        await server.start()
        print("After start")

    loop = asyncio.get_event_loop()
    server_task = loop.create_task(start_server())

    try:
        while True:

            await asyncio.sleep(5)
    except asyncio.CancelledError:
        pass

    print("Closing Modbus server")
    server.stop()
    connection.close()
    loop.run_until_complete(server_task)


if __name__ == "__main__":
    asyncio.run(main())
