#!/usr/bin/env python3
import requests  # type: ignore
import json
import argparse

parser = argparse.ArgumentParser()
parser.add_argument(
    "ip", help="The IP address of the gripper on the network.", type=str
)
args = parser.parse_args()

parameter_codes = "./system_parameter_codes"
parameters = "./data.json"


def read_parameter_codes(filepath: str) -> list[str]:
    def contains_hex(line: str) -> bool:
        return True if "0x" in line and line[0] != "#" else False

    with open(filepath, "r") as f:
        lines = f.read().split("\n")
    lines = list(filter(contains_hex, lines))
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

    with open(parameters, "w") as f:
        json.dump(values, f, indent=4)


if __name__ == "__main__":
    main()
