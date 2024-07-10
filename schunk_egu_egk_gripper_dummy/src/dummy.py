from threading import Thread
import time
from urllib.parse import parse_qs


class Dummy(object):
    def __init__(self):
        self.thread = Thread(target=self._run)
        self.running = False
        self.done = False

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

    def process_get(self, path: str, query: dict[str, list[str]]) -> dict | list | None:
        print(f"path: {path}")
        query = parse_qs(str(query))
        print(f"query: {query}")

        if path == "info.json":
            return {}
        if path == "enum.json":
            return []
        if path == "data.json":
            return []
        return None
