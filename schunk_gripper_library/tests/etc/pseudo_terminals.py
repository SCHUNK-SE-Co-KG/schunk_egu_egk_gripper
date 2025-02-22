#!/usr/bin/env python3
import subprocess
import re
from typing import Any


class Connection(object):

    def __init__(self) -> None:
        self.process: Any | None = None

    def open(self) -> list[str]:
        self.process = subprocess.Popen(
            ["socat", "-d", "-d", "pty,rawer,echo=0", "pty,rawer,echo=0"],
            stderr=subprocess.PIPE,
            text=True,
        )
        pty1: str = ""
        pty2: str = ""

        # Socat will block while the pseudo terminals are active,
        # so read the output line by line
        while True:
            line: str = self.process.stderr.readline()  # type: ignore[union-attr]
            if not line:
                break
            print(line.strip())

            # Extract the pseudo-terminal names from the output
            # and exit once both pseudo terminals are found.
            # Socat will place them under /dev/pts/ by convention.
            match = re.search(r"PTY is (/dev/pts/\d+)", line)
            if match:
                if not pty1:
                    pty1 = match.group(1)
                elif not pty2:
                    pty2 = match.group(1)
                    break

        return [pty1, pty2]

    def close(self) -> None:
        if self.process:
            self.process.kill()
