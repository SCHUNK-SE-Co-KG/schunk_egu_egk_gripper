import struct
from threading import Lock
from pymodbus.client import ModbusSerialClient
from pymodbus.pdu import ModbusPDU
import re
from threading import Thread
import asyncio
import time
from httpx import Client


class Driver(object):
    def __init__(self) -> None:
        self.plc_input: str = "0x0040"
        self.plc_output: str = "0x0048"
        self.error_byte: int = 12
        self.warning_byte: int = 14
        self.additional_byte: int = 15
        # fmt: off
        self.valid_status_bits: list[int] = (
            list(range(0, 10)) + [11, 12, 13, 14, 16, 17, 31]
        )
        self.valid_control_bits: list[int] = (
            list(range(0, 10)) + [11, 12, 13, 14, 16, 30, 31]
        )
        # fmt:on
        self.reserved_status_bits: list[int] = [10, 15] + list(range(18, 31))
        self.reserved_control_bits: list[int] = [10, 15] + list(range(17, 30))

        self.plc_input_buffer: bytearray = bytearray(bytes.fromhex("00" * 16))
        self.plc_output_buffer: bytearray = bytearray(bytes.fromhex("00" * 16))
        self.input_buffer_lock: Lock = Lock()
        self.output_buffer_lock: Lock = Lock()

        self.mb_client: ModbusSerialClient | None = None
        self.mb_device_id: int | None = None
        self.web_client: Client | None = None
        self.connected: bool = False
        self.polling_thread: Thread = Thread()
        self.update_cycle: float = 0.05  # sec

    def connect(
        self,
        host: str = "",
        port: int | str = 80,
        device_id: int | None = None,
        update_cycle: float = 0.05,
    ) -> bool:
        if update_cycle < 0.001:
            return False
        if self.connected:
            return False

        # TCP/IP
        if host:
            if not isinstance(port, int):
                return False
            if isinstance(port, int) and port < 0:
                return False
            self.web_client = Client()
            self.connected = True

        # Modbus
        else:
            if not isinstance(port, str):
                return False
            if not isinstance(device_id, int):
                return False
            if isinstance(device_id, int) and device_id < 0:
                return False
            self.mb_device_id = device_id
            self.mb_client = ModbusSerialClient(
                port=port,
                baudrate=115200,
                parity="N",
                stopbits=1,
                trace_connect=self._trace_connect,
                trace_packet=self._trace_packet,
                trace_pdu=self._trace_pdu,
            )
            self.connected = self.mb_client.connect()

        if self.connected:
            self.update_cycle = update_cycle
            self.polling_thread = Thread(
                target=asyncio.run,
                args=(self._module_update(self.update_cycle),),
                daemon=True,
            )
            self.polling_thread.start()

        return self.connected

    def disconnect(self) -> bool:
        self.connected = False
        if self.polling_thread.is_alive():
            self.polling_thread.join()

        if self.mb_client and self.mb_client.connected:
            self.mb_client.close()

        return True

    async def acknowledge(self) -> bool:
        if not self.connected:
            return False
        self.clear_plc_output()
        self.send_plc_output()

        cmd_toggle_before = self.get_status_bit(bit=5)
        self.set_control_bit(bit=2, value=True)
        self.send_plc_output()

        desired_bits = {"0": 1, "5": cmd_toggle_before ^ 1}
        return await self.wait_for_status(bits=desired_bits)

    async def fast_stop(self) -> bool:
        if not self.connected:
            return False
        self.clear_plc_output()
        self.send_plc_output()

        cmd_toggle_before = self.get_status_bit(bit=5)
        self.set_control_bit(
            bit=0, value=False
        )  # activate fast stop (inverted behavior)
        self.send_plc_output()
        desired_bits = {"5": cmd_toggle_before ^ 1, "7": 1}
        return await self.wait_for_status(bits=desired_bits)

    def send_plc_output(self) -> bool:
        if self.mb_client and self.mb_client.connected:
            with self.output_buffer_lock:
                # Turn the 16-byte array into a list of 2-byte registers.
                # Pymodbus uses big endian internally for their encoding.
                values = [
                    int.from_bytes(self.plc_output_buffer[i : i + 2], byteorder="big")
                    for i in range(0, len(self.plc_output_buffer), 2)
                ]
                self.mb_client.write_registers(
                    address=int(self.plc_output, 16) - 1,  # Modbus convention
                    values=values,
                    slave=self.mb_device_id,
                    no_response_expected=False,
                )
            return True

        if self.web_client and self.connected:
            data = {"inst": self.plc_output, "value": self.get_plc_output()}
            response = self.web_client.post(
                url="http://0.0.0.0:8000/adi/update.json", data=data
            )
            return response.is_success

        return False

    def receive_plc_input(self) -> bool:
        if self.mb_client and self.connected:
            with self.input_buffer_lock:
                pdu = self.mb_client.read_holding_registers(
                    address=int(self.plc_input, 16) - 1,
                    count=8,
                    slave=self.mb_device_id,
                    no_response_expected=False,
                )
                # Parse the 2-byte registers into a 16-byte array.
                # Revert pymodbus' internal big endian decoding.
                data = bytearray()
                for reg in pdu.registers:
                    data.extend(reg.to_bytes(2, byteorder="big"))
                self.plc_input_buffer = data
                return True

        if self.web_client and self.connected:
            params = {"inst": self.plc_input, "count": "1"}
            response = self.web_client.get(
                "http://0.0.0.0:8000/adi/data.json", params=params
            )
            self.set_plc_input(response.json()[0])
            return response.is_success

        return False

    async def wait_for_status(
        self, bits: dict[str, int] = {}, timeout_sec: float = 1.0
    ) -> bool:
        if not timeout_sec > 0.0:
            return False
        if not bits:
            return False
        max_duration = time.time() + timeout_sec
        while not all(
            [self.get_status_bit(int(bit)) == value for bit, value in bits.items()]
        ):
            await asyncio.sleep(0.001)
            self.receive_plc_input()
            if time.time() > max_duration:
                return False
        return True

    def contains_non_hex_chars(self, buffer: str) -> bool:
        return bool(re.search(r"[^0-9a-fA-F]", buffer))

    def set_plc_input(self, buffer: str) -> bool:
        with self.input_buffer_lock:
            if len(buffer) != 32:
                return False
            if self.contains_non_hex_chars(buffer):
                return False
            self.plc_input_buffer = bytearray(bytes.fromhex(buffer))
            return True

    def get_plc_input(self) -> str:
        with self.input_buffer_lock:
            return self.plc_input_buffer.hex().upper()

    def set_plc_output(self, buffer: str) -> bool:
        with self.output_buffer_lock:
            if len(buffer) != 32:
                return False
            if self.contains_non_hex_chars(buffer):
                return False
            self.plc_output_buffer = bytearray(bytes.fromhex(buffer))
            return True

    def get_plc_output(self) -> str:
        with self.output_buffer_lock:
            return self.plc_output_buffer.hex().upper()

    def clear_plc_output(self) -> None:
        self.set_plc_output("00" * 16)
        self.set_control_bit(
            bit=0, value=True
        )  # deactivate fast stop (inverted behavior)

    def set_control_bit(self, bit: int, value: bool) -> bool:
        with self.output_buffer_lock:
            if bit < 0 or bit > 31:
                return False
            if bit in self.reserved_control_bits:
                return False
            byte_index, bit_index = divmod(bit, 8)
            if value:
                self.plc_output_buffer[byte_index] |= 1 << bit_index
            else:
                self.plc_output_buffer[byte_index] &= ~(1 << bit_index)
            return True

    def get_control_bit(self, bit: int) -> int | bool:
        with self.output_buffer_lock:
            if bit < 0 or bit > 31:
                return False
            if bit in self.reserved_control_bits:
                return False
            byte_index, bit_index = divmod(bit, 8)
            return (
                1 if self.plc_output_buffer[byte_index] & (1 << bit_index) != 0 else 0
            )

    def toggle_control_bit(self, bit: int) -> bool:
        with self.output_buffer_lock:
            if bit < 0 or bit > 31:
                return False
            if bit in self.reserved_control_bits:
                return False
            byte_index, bit_index = divmod(bit, 8)
            self.plc_output_buffer[byte_index] ^= 1 << bit_index
            return True

    def get_status_bit(self, bit: int) -> int | bool:
        with self.input_buffer_lock:
            if bit < 0 or bit > 31:
                return False
            if bit in self.reserved_status_bits:
                return False
            byte_index, bit_index = divmod(bit, 8)
            return 1 if self.plc_input_buffer[byte_index] & (1 << bit_index) != 0 else 0

    def get_error_code(self) -> str:
        with self.input_buffer_lock:
            return (
                hex(self.plc_input_buffer[self.error_byte]).upper().replace("0X", "0x")
            )

    def get_warning_code(self) -> str:
        with self.input_buffer_lock:
            return (
                hex(self.plc_input_buffer[self.warning_byte])
                .upper()
                .replace("0X", "0x")
            )

    def get_additional_code(self) -> str:
        with self.input_buffer_lock:
            return (
                hex(self.plc_input_buffer[self.additional_byte])
                .upper()
                .replace("0X", "0x")
            )

    def get_status_diagnostics(self) -> str:
        diagnostics = (
            f"error_code: {self.get_error_code()}"
            + f", warning_code: {self.get_warning_code()}"
            + f", additional_code: {self.get_additional_code()}"
        )
        return diagnostics

    def set_target_position(self, target_pos: int) -> bool:
        with self.output_buffer_lock:
            if not isinstance(target_pos, int):
                return False
            if target_pos < 0:
                return False
            self.plc_output_buffer[4:8] = bytes(struct.pack("i", target_pos))
            return True

    def get_target_position(self) -> int:  # um
        with self.output_buffer_lock:
            return struct.unpack("i", self.plc_output_buffer[4:8])[0]

    def set_target_speed(self, target_speed: int) -> bool:
        with self.output_buffer_lock:
            if not isinstance(target_speed, int):
                return False
            if target_speed < 0:
                return False
            self.plc_output_buffer[8:12] = bytes(struct.pack("i", target_speed))
            return True

    def get_target_speed(self) -> float:
        with self.output_buffer_lock:
            return struct.unpack("i", self.plc_output_buffer[8:12])[0]  # um/s

    def get_actual_position(self) -> int:  # um
        with self.input_buffer_lock:
            return struct.unpack("i", self.plc_input_buffer[4:8])[0]

    def _set_status_bit(self, bit: int, value: bool) -> bool:
        with self.input_buffer_lock:
            if bit < 0 or bit > 31:
                return False
            if bit in self.reserved_status_bits:
                return False
            byte_index, bit_index = divmod(bit, 8)
            if value:
                self.plc_input_buffer[byte_index] |= 1 << bit_index
            else:
                self.plc_input_buffer[byte_index] &= ~(1 << bit_index)
            return True

    async def _module_update(self, update_cycle: float) -> None:
        while self.connected:
            self.receive_plc_input()
            await asyncio.sleep(update_cycle)

    def _trace_packet(self, sending: bool, data: bytes) -> bytes:
        txt = "REQUEST stream" if sending else "RESPONSE stream"
        print(f"---> {txt}: {data!r}")
        return data

    def _trace_pdu(self, sending: bool, pdu: ModbusPDU) -> ModbusPDU:
        txt = "REQUEST pdu" if sending else "RESPONSE pdu"
        print(f"---> {txt}: {pdu}")
        return pdu

    def _trace_connect(self, connect: bool) -> None:
        txt = "Connected" if connect else "Disconnected"
        print(f"---> {txt}")
