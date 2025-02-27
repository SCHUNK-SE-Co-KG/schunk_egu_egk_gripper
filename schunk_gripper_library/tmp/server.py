#!/usr/bin/env python3
import sys

sys.path.append("..")
from tests.etc.pseudo_terminals import Connection  # noqa: E402
from tests.etc.modbus_server import ModbusServer  # noqa: E402
import asyncio  # noqa: E402


def main():
    connection = Connection()
    pt1, pt2 = connection.open()
    server = ModbusServer()
    print(f"Opening Modbus server on {pt2}")
    asyncio.run(server.setup(port=pt1))
    print(f"Using these parameters: {server.get_params()}")
    asyncio.run(server.start())

    _ = input("Press enter to close")

    print("Closing Modbus server")
    server.stop()
    connection.close()


if __name__ == "__main__":
    main()
