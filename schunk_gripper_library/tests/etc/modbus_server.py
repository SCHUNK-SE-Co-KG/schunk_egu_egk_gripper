#!/usr/bin/env python3

import asyncio
from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusServerContext,
    ModbusSlaveContext,
)
from pymodbus.pdu import ModbusPDU
from pymodbus.server import ModbusSerialServer

# Adapted implementation from pymodbus' official documentation:
# https://github.com/pymodbus-dev/pymodbus/blob/dev/examples/server_hook.py
# Accessed: 2/23/2025

# Configure logging
import logging

logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)


class ModbusServer:
    server: ModbusSerialServer
    background_serving: asyncio.Task

    def trace_packet(self, sending: bool, data: bytes) -> bytes:
        txt = "REQUEST stream" if sending else "RESPONSE stream"
        print(f"---> {txt}: {data!r}")
        return data

    def trace_pdu(self, sending: bool, pdu: ModbusPDU) -> ModbusPDU:
        txt = "REQUEST pdu" if sending else "RESPONSE pdu"
        print(f"---> {txt}: {pdu}")
        return pdu

    def trace_connect(self, connect: bool) -> None:
        txt = "Connected" if connect else "Disconnected"
        print(f"---> {txt}")

    async def setup(self, port: str):
        datablock = ModbusSequentialDataBlock(0x00, [0] * 1000)
        slave_id = 12  # SCHUNK default
        slave_context = ModbusSlaveContext(
            di=datablock, co=datablock, hr=datablock, ir=datablock
        )
        context = ModbusServerContext(
            slaves={slave_id: slave_context},
            single=True,
        )
        self.server = ModbusSerialServer(
            context,
            port=port,
            baudrate=115200,
            parity="N",
            stopbits=1,
            bytesize=8,
            timeout=1,
            trace_packet=self.trace_packet,
            trace_pdu=self.trace_pdu,
            trace_connect=self.trace_connect,
        )

    async def start(self):
        self.background_serving = asyncio.create_task(self.server.serve_forever())

    def stop(self):
        self.background_serving.cancel()

    def get_params(self):
        if self.server:
            print(f"Slaves: {self.server.context.slaves()}")
            return self.server.comm_params
        else:
            return "Server not set-up yet."
