from threading import Thread
from queue import PriorityQueue
from concurrent.futures import Future
from functools import partial
import time


class Task(object):
    def __init__(self, future: Future | None = None, func: partial | None = None):
        self.future: Future | None = future
        self.func: partial | None = func
        self.stamp: float = time.time()

    def __bool__(self):
        if self.func is None:
            return False
        return True

    def __lt__(self, other: "Task"):
        return self.stamp < other.stamp

    def __gt__(self, other: "Task"):
        return self.stamp > other.stamp


class Scheduler(object):
    def __init__(self):
        self.tasks: PriorityQueue = PriorityQueue()
        self.worker_thread: Thread = Thread()

    def start(self):
        if not self.worker_thread.is_alive():
            self.worker_thread = Thread(target=self._process, daemon=True)
            self.worker_thread.start()

    def stop(self):
        self.tasks.put((0, Task()))  # highest priority
        if self.worker_thread.is_alive():
            self.worker_thread.join()
        self.tasks = PriorityQueue()

    def execute(self, func: partial, priority: int = 2) -> Future:
        future: Future = Future()
        if not self.worker_thread.is_alive():
            future.set_result(False)
            return future
        if priority not in [1, 2]:
            future.set_result(False)
            return future
        task = Task(func=func, future=future)
        self.tasks.put((priority, task))
        return future

    def _process(self):
        while True:
            _, task = self.tasks.get()
            if not task:
                break
            result = task.func()
            task.future.set_result(result)
            self.tasks.task_done()
