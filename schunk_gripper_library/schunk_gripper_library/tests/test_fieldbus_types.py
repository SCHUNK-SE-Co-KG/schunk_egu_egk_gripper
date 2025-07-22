from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper


@skip_without_gripper
def test_driver_supports_profinet_grippers():
    driver = Driver()
    assert driver.connect(host="192.168.0.42", port=80)

    assert driver.acknowledge()

    max_pos = driver.module_parameters["max_pos"]
    min_pos = driver.module_parameters["min_pos"]
    half = int(0.5 * (max_pos - min_pos))
    max_vel = driver.module_parameters["max_vel"]

    # Move the gripper.
    # This proves that reading and writing parameters works.
    assert driver.move_to_absolute_position(position=max_pos, velocity=max_vel)
    assert driver.move_to_absolute_position(position=min_pos, velocity=max_vel)
    assert driver.move_to_absolute_position(position=half, velocity=max_vel)
    driver.disconnect()
