#!/usr/bin/env python3
import requests  # type: ignore
import json
import argparse
from schunk_gripper_library.driver import Driver

parser = argparse.ArgumentParser()
parser.add_argument(
    "ip", help="The IP address of the gripper on the network.", type=str
)
args = parser.parse_args()

parameter_codes = "./gripper_parameter_codes"

driver = Driver()
driver.connect(host=args.ip)
outfile = f"./grippers/{driver.gripper}.json"
driver.disconnect()


def read_parameter_codes(filepath: str) -> list[str]:
    def contains_hex(line: str) -> bool:
        return True if "0x" in line and line[0] != "#" else False

    def remove_comments(line: str) -> str:
        if line.find("#") != -1:
            line = line[0 : line.find("#")]
        line = line.strip()
        return line

    with open(filepath, "r") as f:
        lines = f.read().split("\n")
    lines = list(filter(contains_hex, lines))
    lines = list(map(remove_comments, lines))
    return lines


def main():
    codes = read_parameter_codes(parameter_codes)

    values = {}
    for code in codes:
        print(f"reading code {code}")
        try:
            response = requests.get(
                f"http://{args.ip}/adi/data.json?inst={int(code, 16)}&count=1"
            )
            values[code] = response.json()
        except Exception as e:
            print(f"error reading code {code}: {e}")

    with open(outfile, "w") as f:
        json.dump(values, f, indent=4)


if __name__ == "__main__":
    main()
