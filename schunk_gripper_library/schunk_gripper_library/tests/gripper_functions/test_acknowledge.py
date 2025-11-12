from schunk_gripper_library.tests.utils import skip_if_no_drivers


def test_acknowledge(drivers, scheduler, executor):
    skip_if_no_drivers(drivers)

    futures = []
    for driver in drivers:
        def do_test():
            assert driver.acknowledge(scheduler), \
                f"Failed to acknowledge driver. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"

        futures.append(executor.submit(do_test))

    for future in futures:
        future.result()
