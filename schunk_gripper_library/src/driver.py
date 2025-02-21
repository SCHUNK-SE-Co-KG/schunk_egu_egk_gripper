import struct


class Driver(object):
    def __init__(self):
        self.plc_input = "0x0040"
        self.plc_output = "0x0048"
        self.error_byte = 12
        self.diagnostics_byte = 15
        self.valid_status_bits = list(range(0, 10)) + [11, 12, 13, 14, 16, 17, 31]
        self.valid_control_bits = list(range(0, 10)) + [11, 12, 13, 14, 16, 30, 31]
        self.reserved_status_bits = [10, 15] + list(range(18, 31))
        self.reserved_control_bits = [10, 15] + list(range(17, 30))

        self.plc_input_buffer = bytearray(bytes.fromhex("00" * 16))
        self.plc_output_buffer = bytearray(bytes.fromhex("00" * 16))

    def get_plc_input(self):
        return self.plc_input_buffer.hex().upper()

    def get_plc_output(self):
        return self.plc_output_buffer.hex().upper()

    def set_control_bit(self, bit: int, value: bool) -> bool:
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
        if bit < 0 or bit > 31:
            return False
        if bit in self.reserved_control_bits:
            return False
        byte_index, bit_index = divmod(bit, 8)
        return 1 if self.plc_output_buffer[byte_index] & (1 << bit_index) != 0 else 0

    def toggle_control_bit(self, bit: int) -> bool:
        if bit < 0 or bit > 31:
            return False
        if bit in self.reserved_control_bits:
            return False
        byte_index, bit_index = divmod(bit, 8)
        self.plc_output_buffer[byte_index] ^= 1 << bit_index
        return True

    def get_status_bit(self, bit: int) -> int | bool:
        if bit < 0 or bit > 31:
            return False
        if bit in self.reserved_status_bits:
            return False
        byte_index, bit_index = divmod(bit, 8)
        return 1 if self.plc_input_buffer[byte_index] & (1 << bit_index) != 0 else 0

    def get_status_error(self) -> str:
        return hex(self.plc_input_buffer[self.error_byte]).replace("0x", "").upper()

    def get_status_diagnostics(self) -> str:
        return (
            hex(self.plc_input_buffer[self.diagnostics_byte]).replace("0x", "").upper()
        )

    def set_target_position(self, target_pos: int) -> bool:
        if not isinstance(target_pos, int):
            return False
        if target_pos < 0:
            return False
        self.plc_output_buffer[4:8] = bytes(struct.pack("i", target_pos))
        return True

    def get_target_position(self) -> int:  # um
        return struct.unpack("i", self.plc_output_buffer[4:8])[0]

    def set_target_speed(self, target_speed: int) -> bool:
        if not isinstance(target_speed, int):
            return False
        if target_speed < 0:
            return False
        self.plc_output_buffer[8:12] = bytes(struct.pack("i", target_speed))
        return True

    def get_target_speed(self) -> float:
        return (
            struct.unpack("i", self.plc_output_buffer[8:12])[0] / 1000.0
        )  # um/s to mm/s

    def get_actual_position(self) -> int:  # um
        return struct.unpack("i", self.plc_input_buffer[4:8])[0]

    def _set_status_bit(self, bit: int, value: bool) -> bool:
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
