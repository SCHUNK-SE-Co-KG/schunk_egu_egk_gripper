from threading import Thread
import time


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

    def _run(self):
        while not self.done:
            time.sleep(1)
        print("Done")

    def process(self, msg: str) -> bool:
        print(msg)
        return True
