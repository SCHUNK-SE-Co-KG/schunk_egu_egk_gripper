from schunk_gripper_library.tests.utils import skip_if_no_drivers


def test_stop(drivers, scheduler, executor):
    skip_if_no_drivers(drivers)

    futures = []
    for driver in drivers:
        def do_test():
            assert driver.stop(scheduler=scheduler, use_gpe=False), \
            f"Failed to stop driver without GPE. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
            if driver.gpe_available():
                assert driver.stop(scheduler=scheduler, use_gpe=True), \
                f"Failed to stop driver with GPE. Driver: {driver.addr_str}, Status: {driver.get_status_diagnostics()}"
        
        futures.append(executor.submit(do_test))
    
    for future in futures:
        future.result()