#!/usr/bin/env python3

import argparse
import time

from pymodbus.client import ModbusSerialClient


SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
STATUS_REGISTER = 0x0040 - 1  # minus 1, because Modbus addresses are 1-based while pymodbus uses 0-based addressing
CONTROL_REGISTER = 0x0048 - 1
CLEAR_COMMAND_FRAME = [0x0100, 0, 0, 0, 0, 0, 0, 0]
HEARTBEAT_INTERVAL = 1.0
LOOP_INTERVAL = 1.0 / 200.0


def registers_to_bytes(registers: list[int]) -> bytes:
    return b"".join(register.to_bytes(2, byteorder="big") for register in registers)


def read_status(client: ModbusSerialClient, device_id: int) -> bytes:
    start_time = time.perf_counter()
    try:
        response = client.read_holding_registers(
            address=STATUS_REGISTER,
            count=8,
            slave=device_id,
        )
    finally:
        elapsed_ms = (time.perf_counter() - start_time) * 1000
        print(f"Status read took {elapsed_ms:.3f} ms")
    if response.isError():
        raise RuntimeError(f"Failed to read status: {response}")
    return registers_to_bytes(response.registers)


def write_control_frame(
    client: ModbusSerialClient, device_id: int, frame: list[int]
) -> None:
    response = client.write_registers(
        address=CONTROL_REGISTER,
        values=frame,
        slave=device_id,
    )
    if response.isError():
        raise RuntimeError(f"Failed to write control frame: {response}")


def acknowledge(
    client: ModbusSerialClient,
    device_id: int,
    repeat_command_toggle: bool,
    wait_for_result: bool = True,
) -> bytes | None:
    if wait_for_result:
        status_before = read_status(client, device_id)
        expected_command_toggle = ((status_before[0] >> 5) & 1) ^ 1

    first_byte = 0x05 | (int(repeat_command_toggle) << 6)
    acknowledge_frame = [first_byte << 8, 0, 0, 0, 0, 0, 0, 0]
    write_control_frame(client, device_id, acknowledge_frame)

    if not wait_for_result:
        return None

    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        status = read_status(client, device_id)
        command_received = status[0] & 1
        command_toggle = (status[0] >> 5) & 1
        if command_received and command_toggle == expected_command_toggle:
            return status
        time.sleep(0.05)

    raise RuntimeError("Gripper did not acknowledge the command")


def print_diagnostics(status: bytes) -> None:
    diagnostics = status[-4:]
    print(f"Error code: 0x{diagnostics[0]:02X}")
    print(f"Warning code: 0x{diagnostics[2]:02X}")
    print(f"Additional code: 0x{diagnostics[3]:02X}")


def run_loop(
    client: ModbusSerialClient,
    device_id: int,
    repeat_command_toggle: bool,
) -> None:
    next_iteration = time.monotonic()
    next_heartbeat = time.monotonic() + HEARTBEAT_INTERVAL
    rate_window_start = time.monotonic()
    write_count = 0

    while True:
        acknowledge(
            client,
            device_id,
            repeat_command_toggle,
            wait_for_result=False,
        )
        repeat_command_toggle = not repeat_command_toggle
        write_count += 1

        now = time.monotonic()
        rate_window_elapsed = now - rate_window_start
        if rate_window_elapsed >= 1.0:
            write_rate = write_count / rate_window_elapsed
            print(f"Write rate: {write_rate:.2f} Hz")
            write_count = 0
            rate_window_start = now

        if now >= next_heartbeat:
            read_status(client, device_id)
            next_heartbeat = time.monotonic() + HEARTBEAT_INTERVAL

        next_iteration += LOOP_INTERVAL
        if next_iteration < time.monotonic():
            next_iteration = time.monotonic()
        time.sleep(max(0.0, next_iteration - time.monotonic()))


def main() -> int:
    parser = argparse.ArgumentParser(description="Test the Modbus connection to a gripper.")
    parser.add_argument("device_id", type=int, help="Modbus device ID")
    args = parser.parse_args()

    client = ModbusSerialClient(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        parity="E",
        stopbits=1,
        timeout=1.0,
    )
    try:
        if not client.connect():
            print(f"Failed to open {SERIAL_PORT} at {BAUDRATE} baud.")
            return 1

        write_control_frame(client, args.device_id, CLEAR_COMMAND_FRAME)
        status = acknowledge(client, args.device_id, repeat_command_toggle=True)
        if status is None:
            raise RuntimeError("Acknowledge completed without a status response")
        print(f"Connected to Modbus gripper with device ID {args.device_id}.")
        print_diagnostics(status)
        print("Heartbeat started. Press Ctrl+C to stop.")
        run_loop(client, args.device_id, repeat_command_toggle=False)
    except KeyboardInterrupt:
        print("\nHeartbeat stopped.")
        return 0
    except Exception as error:
        print(f"Failed to connect to Modbus gripper with device ID {args.device_id}: {error}")
        return 1
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())