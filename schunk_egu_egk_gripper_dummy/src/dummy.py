from threading import Thread
import time
import os
from pathlib import Path
import json


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
        self.reserved_status_bits = [10, 15] + list(range(18, 31))

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
