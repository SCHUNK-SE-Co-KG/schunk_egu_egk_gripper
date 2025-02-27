import struct
from threading import Lock
from pymodbus.client import ModbusSerialClient
import re


class Driver(object):
    def __init__(self) -> None:
        self.plc_input: str = "0x0040"
        self.plc_output: str = "0x0048"
        self.error_byte: int = 12
        self.diagnostics_byte: int = 15
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
        self.connected: bool = False

    def connect(self, protocol: str, port: str, device_id: int | None = None) -> bool:
        if protocol not in ["modbus"]:
            return False
        if not isinstance(port, str):
            return False
        if protocol == "modbus" and not device_id:
            return False
        if device_id and not isinstance(device_id, int):
            return False
        if isinstance(device_id, int) and device_id < 0:
            return False
        if self.connected:
            return False

        if protocol == "modbus":
            self.mb_device_id = device_id
            self.mb_client = ModbusSerialClient(
                port=port,
                baudrate=115200,
                parity="N",
                timeout=1,
            )
            self.connected = self.mb_client.connect()
        return self.connected

    def disconnect(self) -> bool:
        if self.mb_client and self.mb_client.connected:
            self.mb_client.close()
        return True

    def send_plc_output(self) -> bool:
        if self.mb_client and self.mb_client.connected:
            with self.output_buffer_lock:
                # Turn the 16-byte array into a list of 2-byte registers
                values = [
                    int.from_bytes(
                        self.plc_output_buffer[i : i + 2], byteorder="little"
                    )
                    for i in range(0, len(self.plc_output_buffer), 2)
                ]
                self.mb_client.write_registers(
                    address=int(self.plc_output, 16) - 1,  # Modbus convention
                    values=values,
                    slave=self.mb_device_id,
                    no_response_expected=False,
                )
            return True
        return False

    def receive_plc_input(self) -> bool:
        if self.mb_client and self.mb_client.connected:
            with self.input_buffer_lock:
                pdu = self.mb_client.read_holding_registers(
                    address=int(self.plc_input, 16) - 1,
                    count=8,
                    slave=self.mb_device_id,
                    no_response_expected=False,
                )
                # Parse the 2-byte registers into a 16-byte array
                data = bytearray()
                for reg in pdu.registers:
                    data.extend(reg.to_bytes(2, byteorder="little"))
                self.plc_input_buffer = data
                return True
        return False

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

    def get_status_error(self) -> str:
        with self.input_buffer_lock:
            return hex(self.plc_input_buffer[self.error_byte]).replace("0x", "").upper()

    def get_status_diagnostics(self) -> str:
        with self.input_buffer_lock:
            return (
                hex(self.plc_input_buffer[self.diagnostics_byte])
                .replace("0x", "")
                .upper()
            )

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
