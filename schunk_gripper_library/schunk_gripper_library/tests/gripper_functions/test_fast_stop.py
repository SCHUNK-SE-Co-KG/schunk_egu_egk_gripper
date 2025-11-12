from schunk_gripper_library.tests.utils import skip_if_no_drivers


def test_fast_stop(drivers, scheduler, executor):
    skip_if_no_drivers(drivers)

    futures = []
    for driver in drivers:
        def do_test():
            assert driver.fast_stop(scheduler=scheduler), \
            f"Failed to fast stop driver. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"

        futures.append(executor.submit(do_test))

    for future in futures:
        future.result()
