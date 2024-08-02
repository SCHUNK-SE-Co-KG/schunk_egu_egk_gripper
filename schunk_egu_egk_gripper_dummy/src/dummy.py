from threading import Thread
import time
import os
from pathlib import Path
import json
import struct


class Dummy(object):
    def __init__(self):
        self.thread = Thread(target=self._run)
        self.running = False
        self.done = False
        self.enum = None
        self.metadata = None
        self.data = None
        self.plc_input = "0x0040"
        self.plc_output = "0x0048"
        self.actual_position = "0x0230"
        self.actual_speed = "0x0238"
        self.error_byte = 12
        self.diagnostics_byte = 15
        self.valid_status_bits = list(range(0, 10)) + [11, 12, 13, 14, 16, 17, 31]
        self.valid_control_bits = list(range(0, 10)) + [11, 12, 13, 14, 16, 30, 31]
        self.reserved_status_bits = [10, 15] + list(range(18, 31))
        self.reserved_control_bits = [10, 15] + list(range(17, 30))

        enum_config = os.path.join(
            Path(__file__).resolve().parents[1], "config/enum.json"
        )
        metadata_config = os.path.join(
            Path(__file__).resolve().parents[1], "config/metadata.json"
        )
        data_config = os.path.join(
            Path(__file__).resolve().parents[1], "config/data.json"
        )
        with open(enum_config, "r") as f:
            self.enum = json.load(f)
        with open(metadata_config, "r") as f:
            self.metadata = json.load(f)
        with open(data_config, "r") as f:
            self.data = json.load(f)

        self.plc_input_buffer = bytearray(bytes.fromhex(self.data[self.plc_input][0]))
        self.plc_output_buffer = bytearray(bytes.fromhex(self.data[self.plc_output][0]))

    def start(self) -> None:
        if self.running:
            return
        self.thread.start()
        self.running = True

    def stop(self) -> None:
        self.done = True
        self.thread.join()
        self.running = False

    def _run(self) -> None:
        while not self.done:
            time.sleep(1)
        print("Done")

    def post(self, msg: dict) -> dict:
        if msg["inst"] == self.plc_output:
            self.plc_output_buffer = bytearray(bytes.fromhex(msg["value"]))
        else:
            self.data[msg["inst"]] = [msg["value"]]

        # Behavior
        self.process_control_bits()
        return {"result": 0}

    def get_info(self, query: dict[str, str]) -> dict:
        return {"dataformat": 0}  # 0: Little endian, 1: Big endian

    def get_enum(self, query: dict[str, str]) -> list:
        if "inst" not in query or "value" not in query:
            return []
        inst = query["inst"]
        value = int(query["value"])
        if inst in self.enum:
            string = self.enum[inst][value]["string"]
            return [{"string": string, "value": value}]
        else:
            return []

    def get_data(self, query: dict[str, str]) -> list:
        result: list = []
        if "offset" in query and "count" in query:
            offset = int(query["offset"])
            count = int(query["count"])
            if offset < 0 or count < 0:
                return result
            if offset + count >= len(self.metadata):
                return result
            for i in range(count):
                result.append(self.metadata[offset + i])
            return result

        if "inst" in query and "count" in query:
            inst = query["inst"]
            count = int(query["count"])
            if count != 1:
                return result
            if inst not in self.data:
                return result
            if inst == self.plc_input:
                return self.get_plc_input()
            if inst == self.plc_output:
                return self.get_plc_output()
            return self.data[inst]
        else:
            return []

    def get_plc_input(self):
        return [self.plc_input_buffer.hex().upper()]

    def get_plc_output(self):
        return [self.plc_output_buffer.hex().upper()]

    def set_status_bit(self, bit: int, value: bool) -> bool:
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

    def get_status_bit(self, bit: int) -> int | bool:
        if bit < 0 or bit > 31:
            return False
        if bit in self.reserved_status_bits:
            return False
        byte_index, bit_index = divmod(bit, 8)
        return 1 if self.plc_input_buffer[byte_index] & (1 << bit_index) != 0 else 0

    def toggle_status_bit(self, bit: int) -> bool:
        if bit < 0 or bit > 31:
            return False
        if bit in self.reserved_status_bits:
            return False
        byte_index, bit_index = divmod(bit, 8)
        self.plc_input_buffer[byte_index] ^= 1 << bit_index
        return True

    def set_status_error(self, error: str) -> bool:
        try:
            self.plc_input_buffer[self.error_byte] = int(error, 16)
            return True
        except ValueError:
            return False

    def get_status_error(self) -> str:
        return hex(self.plc_input_buffer[self.error_byte]).replace("0x", "").upper()

    def set_status_diagnostics(self, diagnostics: str) -> bool:
        try:
            self.plc_input_buffer[self.diagnostics_byte] = int(diagnostics, 16)
            return True
        except ValueError:
            return False

    def get_status_diagnostics(self) -> str:
        return (
            hex(self.plc_input_buffer[self.diagnostics_byte]).replace("0x", "").upper()
        )

    def get_control_bit(self, bit: int) -> int | bool:
        if bit < 0 or bit > 31:
            return False
        if bit in self.reserved_control_bits:
            return False
        byte_index, bit_index = divmod(bit, 8)
        return 1 if self.plc_output_buffer[byte_index] & (1 << bit_index) != 0 else 0

    def get_target_position(self) -> float:
        return struct.unpack("f", self.plc_output_buffer[4:8])[0]

    def get_target_speed(self) -> float:
        return struct.unpack("f", self.plc_output_buffer[8:12])[0]

    def set_actual_position(self, position: float) -> None:
        self.data[self.actual_position] = [
            bytes(struct.pack("f", position)).hex().upper()
        ]

    def set_actual_speed(self, speed: float) -> None:
        self.data[self.actual_speed] = [bytes(struct.pack("f", speed)).hex().upper()]

    def process_control_bits(self) -> None:

        # Acknowledge
        if self.get_control_bit(2) == 1:
            self.set_status_bit(bit=0, value=True)
            self.set_status_bit(bit=7, value=False)
            self.set_status_error("00")
            self.set_status_diagnostics("00")
