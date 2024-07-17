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

    def get(self, path: str, query: dict[str, str]) -> dict | list | None:
        print(f"path: {path}")
        print(f"query: {query}")

        if path == "info.json":
            return {"dataformat": 0}  # 0: Little endian, 1: Big endian

        if path == "enum.json":
            inst = query["inst"]
            value = int(query["value"])
            if inst in self.enum:
                string = self.enum[inst][value]["string"]
                return [{"string": string, "value": value}]
            else:
                return []

        if path == "data.json":
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
            else:
                return []
        return None
