#!/usr/bin/env python3
import sys

sys.path.append("..")
from src.driver import Driver  # noqa: E402


def main():
    driver = Driver()
    driver.connect("modbus", "/dev/pts/17", 12)
    driver.send_plc_output()


if __name__ == "__main__":
    main()
