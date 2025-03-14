from threading import Thread
import time
from importlib.resources import files
import json
import struct
from typing import Tuple


class LinearMotion(object):
    def __init__(
        self,
        initial_pos: float,
        initial_speed: float,
        target_pos: float,
        target_speed: float,
    ):
        self.min_speed = 0.001
        self.initial_pos = initial_pos
        self.initial_speed = max(0.0, initial_speed)
        self.target_pos = target_pos
        self.target_speed = max(self.min_speed, target_speed)
        self.time_finish = abs(self.target_pos / self.target_speed)

    def sample(self, t: float) -> Tuple[float, float]:
        if t <= 0.0:
            return (self.initial_pos, self.initial_speed)
        if t >= self.time_finish:
            return (self.target_pos, 0.0)

        v = self.target_speed
        if self.target_pos < self.initial_pos:
            v = -v
        current_pos = v * t + self.initial_pos
        current_speed = abs(v)
        return (current_pos, current_speed)


class Dummy(object):
    def __init__(self):
        self.starttime = time.time()
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
        self.system_uptime = "0x1400"
        self.error_byte = 12
        self.diagnostics_byte = 15
        self.valid_status_bits = list(range(0, 10)) + [11, 12, 13, 14, 16, 17, 31]
        self.valid_control_bits = list(range(0, 10)) + [11, 12, 13, 14, 16, 30, 31]
        self.reserved_status_bits = [10, 15] + list(range(18, 31))
        self.reserved_control_bits = [10, 15] + list(range(17, 30))

        enum_config = files(__package__).joinpath("config/enum.json")
        metadata_config = files(__package__).joinpath("config/metadata.json")
        data_config = files(__package__).joinpath("config/data.json")

        with open(enum_config, "r") as f:
            self.enum = json.load(f)
        with open(metadata_config, "r") as f:
            self.metadata = json.load(f)
        with open(data_config, "r") as f:
            self.data = json.load(f)

        self.plc_input_buffer = bytearray(bytes.fromhex(self.data[self.plc_input][0]))
        self.plc_output_buffer = bytearray(bytes.fromhex(self.data[self.plc_output][0]))
        self.initial_state = [self.plc_input_buffer.hex().upper()]

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
            elapsed = time.time() - self.starttime
            self.set_system_uptime(int(elapsed))
            time.sleep(1)
        print("Done")

    def move(self, target_pos: float, target_speed: float) -> None:
        motion = LinearMotion(
            initial_pos=self.get_actual_position(),
            initial_speed=self.get_actual_speed(),
            target_pos=target_pos,
            target_speed=target_speed,
        )
        start = time.time()
        actual_pos, actual_speed = motion.sample(0)
        while abs(target_pos - actual_pos) > 0.001:  # mm
            t = time.time() - start
            actual_pos, actual_speed = motion.sample(t)
            self.set_actual_position(actual_pos)
            self.set_actual_speed(actual_speed)
            time.sleep(0.01)

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
                id = self.metadata[offset + i]["instance"]
                inst = hex(id)[2:].upper().zfill(4)
                inst = "0x" + inst
                result.append(self.data[inst][0])
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

    def clear_plc_output(self) -> None:
        self.plc_output_buffer = bytearray(bytes.fromhex("00" * 16))
        self.set_control_bit(
            bit=0, value=True
        )  # deactivate fast stop (inverted behavior)

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

    def get_target_position(self) -> float:
        return struct.unpack("i", self.plc_output_buffer[4:8])[0] / 1000.0  # um to mm

    def get_target_speed(self) -> float:
        return (
            struct.unpack("i", self.plc_output_buffer[8:12])[0] / 1000.0
        )  # um/s to mm/s

    def set_actual_position(self, position: float) -> None:
        self.data[self.actual_position] = [
            bytes(struct.pack("f", position)).hex().upper()
        ]

    def set_actual_speed(self, speed: float) -> None:
        self.data[self.actual_speed] = [bytes(struct.pack("f", speed)).hex().upper()]

    def get_actual_position(self) -> float:
        read_pos = self.data[self.actual_position][0]
        return struct.unpack("f", bytes.fromhex(read_pos))[0]

    def get_actual_speed(self) -> float:
        read_speed = self.data[self.actual_speed][0]
        return struct.unpack("f", bytes.fromhex(read_speed))[0]

    def set_system_uptime(self, uptime: int) -> None:
        self.data[self.system_uptime] = [bytes(struct.pack("i", uptime)).hex().upper()]

    def get_system_uptime(self) -> int:
        uptime = self.data[self.system_uptime][0]
        return struct.unpack("i", bytes.fromhex(uptime))[0]

    def process_control_bits(self) -> None:
        """
        See the gripper's firmware documentation for EtherNet/IP [1]:
        https://stb.cloud.schunk.com/media/IM0046706.PDF

        """
        # Reset success of previous commands
        self.set_status_bit(bit=4, value=False)
        self.set_status_bit(bit=8, value=False)
        self.set_status_bit(bit=12, value=False)
        self.set_status_bit(bit=13, value=False)

        # Clearing all control bits doesn't trigger any action
        if self.get_plc_output()[0] == "01" + "00" * 15:
            return

        # Command received toggle
        self.toggle_status_bit(bit=5)

        # Acknowledge
        if self.get_control_bit(2) == 1:
            self.set_status_bit(bit=0, value=True)
            self.set_status_bit(bit=7, value=False)
            self.set_status_error("00")
            self.set_status_diagnostics("00")

        # Brake test
        if self.get_control_bit(30) == 1:
            self.set_status_bit(bit=4, value=True)

        # Fast stop
        if self.get_control_bit(0) == 0:  # fail-safe behavior
            self.set_status_bit(bit=7, value=True)
            self.set_status_bit(bit=0, value=False)
            self.set_status_diagnostics("D9")

        # Controlled stop
        if self.get_control_bit(1) == 1:
            self.set_status_bit(bit=13, value=True)
            self.set_status_bit(bit=4, value=True)

        # Manual release
        if self.get_control_bit(5) == 1:
            if self.get_status_bit(7) == 1:
                self.set_status_bit(bit=8, value=True)

        # Shutdown
        if self.get_control_bit(3) == 1:
            self.set_status_bit(bit=2, value=True)

        # Release workpiece
        if self.get_control_bit(bit=11) == 1:
            self.set_status_bit(bit=4, value=True)
            self.set_status_bit(bit=13, value=True)
            self.set_status_bit(bit=14, value=False)
            self.set_status_bit(bit=12, value=False)
            self.set_status_bit(bit=17, value=False)

        # Soft reset
        if self.get_control_bit(bit=4) == 1:
            self.set_system_uptime(0)
            self.data[self.plc_input] = self.initial_state

        # Move to absolute position
        if self.get_control_bit(bit=13) == 1:
            self.move(
                target_pos=self.get_target_position(),
                target_speed=self.get_target_speed(),
            )
            self.set_status_bit(bit=13, value=True)
            self.set_status_bit(bit=4, value=True)

        # Move to relative position
        if self.get_control_bit(bit=14) == 1:
            target_pos = (
                self.get_actual_position()
                + self.get_target_position()  # interpret relative
            )
            self.move(
                target_pos=target_pos,
                target_speed=self.get_target_speed(),
            )
            self.set_status_bit(bit=13, value=True)
            self.set_status_bit(bit=4, value=True)

        # Grip workpiece
        if self.get_control_bit(bit=12) == 1:
            self.set_status_bit(bit=12, value=True)
            self.set_status_bit(bit=4, value=True)

        # Grip workpiece at position
        if self.get_control_bit(bit=16) == 1:
            self.set_status_bit(bit=12, value=True)
            self.set_status_bit(bit=4, value=True)
            self.set_status_bit(bit=31, value=True)
