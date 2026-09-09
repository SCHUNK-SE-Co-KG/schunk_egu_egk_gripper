from collections import deque
import shutil
import subprocess
import time


class StreamPlotter:
    """Live terminal plot of target positions sent during streaming."""

    def __init__(
        self,
        gripper_id: str,
        window_sec: float = 30.0,
        update_period_sec: float = 0.1,
    ):
        self.gripper_id = gripper_id
        self.window_sec = window_sec
        self.update_period_sec = update_period_sec
        self._process: subprocess.Popen[str] | None = None
        self._start_time: float | None = None
        self._last_update: float = 0.0
        self._samples: deque[tuple[float, float]] = deque()

    def start(self) -> None:
        self.stop()
        self._start_time = time.monotonic()
        if shutil.which("gnuplot") is None:
            return
        try:
            self._process = subprocess.Popen(
                ["gnuplot"], stdin=subprocess.PIPE, text=True
            )
            if self._process.stdin is None:
                self.stop()
                return
            self._process.stdin.write(
                "set terminal dumb 100 30\n"
                "set xlabel 'time (s)'\n"
                "set ylabel 'pos (mm)'\n"
                f"set title 'Streamed targets for {self.gripper_id}'\n"
                "unset key\n"
            )
            self._process.stdin.flush()
        except OSError:
            self.stop()

    def stop(self) -> None:
        if self._process is not None:
            self._process.terminate()
            try:
                self._process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
        self._process = None
        self._start_time = None
        self._last_update = 0.0
        self._samples.clear()

    def add_sample(self, position_um: int) -> None:
        if self._process is None or self._process.poll() is not None:
            return
        now = time.monotonic()
        if self._start_time is None:
            self._start_time = now
        elapsed_sec = now - self._start_time
        self._samples.append((elapsed_sec, position_um / 1000.0))
        while self._samples and elapsed_sec - self._samples[0][0] > self.window_sec:
            self._samples.popleft()
        if now - self._last_update < self.update_period_sec:
            return
        self._last_update = now
        try:
            if self._process.stdin is None:
                return
            start = max(0.0, elapsed_sec - self.window_sec)
            self._process.stdin.write(f"set xrange [{start}:{elapsed_sec}]\n")
            self._process.stdin.write("plot '-' using 1:2 with lines\n")
            self._process.stdin.writelines(
                f"{sample_time} {sample_position}\n"
                for sample_time, sample_position in self._samples
            )
            self._process.stdin.write("e\n")
            self._process.stdin.flush()
        except (BrokenPipeError, OSError):
            self.stop()
