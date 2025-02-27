#!/usr/bin/env python3
from src.driver import Driver


def main():
    driver = Driver()
    driver.connect("modbus", "/dev/pts/14", 12)
    driver.send_plc_output()


if __name__ == "__main__":
    main()
