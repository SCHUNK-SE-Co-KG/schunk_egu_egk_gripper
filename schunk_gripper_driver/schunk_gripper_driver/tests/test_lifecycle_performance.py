from schunk_gripper_driver.driver import Driver
from time import perf_counter
import numpy as np


def test_duration_of_lifecycles(ros2):
    driver = Driver("driver")

    assert driver.load_previous_configuration()

    def run_cycle():
        assert driver.on_configure(state=None)
        assert driver.on_activate(state=None)
        assert driver.on_deactivate(state=None)
        assert driver.on_cleanup(state=None)

    # Collect data on complete lifecycle changes
    n_runs = 100
    durations = []

    print()
    for i in range(n_runs):
        print(f"Running {i+1}/{n_runs}", end="\r")
        start = perf_counter()
        run_cycle()
        end = perf_counter()
        durations.append(end - start)

    # Statistics evaluation
    durations = np.array(durations)
    print()
    print(f"For {n_runs} runs:")
    print(f"Mean duration: {durations.mean():.6f} seconds")
    print(f"Standard deviation: {durations.std():.6f} seconds")
    print(f"Min duration: {durations.min():.6f} seconds")
    print(f"Max duration: {durations.max():.6f} seconds")
