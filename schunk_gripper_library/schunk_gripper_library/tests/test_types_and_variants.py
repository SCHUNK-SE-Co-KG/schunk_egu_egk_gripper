# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

from schunk_gripper_library.driver import Driver


def test_gripper_types():
    driver = Driver()

    invalid_combinations = [
        {"args": {"module_type": "EGU_50_", "fieldbus": "EC"}},
        {"args": {"module_type": "EGU_50_M_B", "fieldbus": "AA"}},
        {"args": {"module_type": "0x?|^$%", "fieldbus": "PN"}},
    ]
    for entry in invalid_combinations:
        gripper_type = driver.compose_gripper_type(**entry["args"])
        assert gripper_type == ""

    valid_combinations = [
        {
            "args": {"module_type": "EGU_50_M_B", "fieldbus": "EC"},
            "expected": "EGU_50_EC_M_B",
        },
        {
            "args": {"module_type": "EGU_80_N_SD", "fieldbus": "EI"},
            "expected": "EGU_80_EI_N_SD",
        },
        {
            "args": {"module_type": "EZU_35_N_B", "fieldbus": "MB"},
            "expected": "EZU_35_MB_N_B",
        },
        {
            "args": {"module_type": "EZU_40_N_SD", "fieldbus": "MB"},
            "expected": "EZU_40_MB_N_SD",
        },
        {
            "args": {"module_type": "EGK_25_M_B", "fieldbus": "MB"},
            "expected": "EGK_25_MB_M_B",
        },
        {
            "args": {"module_type": "EGK_50_N_B", "fieldbus": "PN"},
            "expected": "EGK_50_PN_N_B",
        },
    ]
    for entry in valid_combinations:
        gripper_type = driver.compose_gripper_type(**entry["args"])
        assert gripper_type == entry["expected"]


def test_variants():
    driver = Driver()
    assert driver.get_variant() == ""  # when unconnected

    for module in driver.valid_module_types.values():
        driver.module_type = module
        expected = module.split("_")[0]
        assert driver.get_variant() == expected

    unknown_types = [
        "XYZ_99_PN_M",
        "UG4_DIO_80",
        "0x0048",
        "???",
        "_EGU_40_M_B",
        "_EGU_40_M_B",
        " EGK_123",
        "EGu_5",
        "EGU_90_M_B",
        "EGK_100_E_Z",
        "EZU_42_M_SD",
        "",
    ]

    for idx, type_str in enumerate(unknown_types):
        driver.module_type = type_str
        assert driver.get_variant() == "", f"wrong type at index: {idx}"
