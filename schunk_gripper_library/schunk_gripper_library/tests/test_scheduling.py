from schunk_gripper_library.utility import Task, Scheduler
import threading
from queue import PriorityQueue
from functools import partial
import time


def YES() -> bool:
    return True


def NO() -> bool:
    return False


def test_tasks_have_expected_initial_values():
    task = Task()
    assert task.func is None
    assert task.future is None
    assert task.stamp < time.time()


def test_tasks_have_boolean_meaning():
    task = Task()
    assert not task
    task.func = "some function"
    assert task


def test_tasks_support_less_than_and_greater_than_operators():
    task1 = Task()
    task2 = Task()
    assert task1 < task2
    assert task2 > task1


def test_scheduler_runs_internal_worker_thread_when_started():
    before = threading.active_count()
    scheduler = Scheduler()
    for _ in range(3):
        scheduler.start()
        assert threading.active_count() == before + 1
        scheduler.stop()
        assert threading.active_count() == before


def test_schedular_supports_multiple_starts_and_stops():
    scheduler = Scheduler()
    before = threading.active_count()
    for _ in range(3):
        scheduler.start()
    assert threading.active_count() == before + 1
    for _ in range(3):
        scheduler.stop()
    assert threading.active_count() == before


def test_scheduler_uses_a_priority_queue_for_tasks():
    scheduler = Scheduler()
    assert isinstance(scheduler.tasks, PriorityQueue)
    assert scheduler.tasks.empty()  # on startup


def test_scheduler_clears_tasks_on_stop():
    scheduler = Scheduler()
    task = Task()
    scheduler.tasks.put((1, task))
    scheduler.stop()
    assert scheduler.tasks.empty()


def test_scheduler_can_execute_tasks():
    scheduler = Scheduler()
    scheduler.start()
    assert scheduler.execute(func=partial(YES)).result()
    assert not scheduler.execute(func=partial(NO)).result()
    scheduler.stop()


def test_scheduler_rejects_tasks_when_not_connected():
    scheduler = Scheduler()
    assert not scheduler.execute(func=partial(YES)).result()
    scheduler.start()
    assert scheduler.execute(func=partial(YES)).result()
    scheduler.stop()


def test_scheduler_rejects_tasks_with_invalid_priority():
    scheduler = Scheduler()
    invalid_priorities = [0, -1, "asap", 3]
    scheduler.start()
    for priority in invalid_priorities:
        assert not scheduler.execute(func=partial(YES), priority=priority).result()


def test_scheduler_supports_task_execution_from_several_threads():
    scheduler = Scheduler()
    scheduler.start()
    threads = []

    def repeatedly_execute():
        for _ in range(5):
            assert scheduler.execute(func=partial(YES)).result()
            time.sleep(0.01)  # make sure threads overlap a little

    for _ in range(10):
        thread = threading.Thread(target=repeatedly_execute)
        thread.start()
        threads.append(thread)
    for thread in threads:
        thread.join()
    scheduler.stop()


def test_scheduler_executes_tasks_according_to_their_priorities():
    scheduler = Scheduler()
    scheduler.start()
    global low_priority_finished
    global high_priority_finished

    def fill_scheduler_queue():
        def wait():
            time.sleep(1)
            return True

        for _ in range(3):
            scheduler.execute(func=partial(wait))

    def low_priority():
        assert scheduler.execute(func=partial(YES), priority=2).result()
        global low_priority_finished
        low_priority_finished = time.time()

    def high_priority():
        assert scheduler.execute(func=partial(YES), priority=1).result()
        global high_priority_finished
        high_priority_finished = time.time()

    # Block the scheduler's task queue for some time so that
    # new tasks can be re-arranged.
    thread_fill_scheduler = threading.Thread(target=fill_scheduler_queue, daemon=True)
    thread_fill_scheduler.start()

    thread_low_priority = threading.Thread(target=low_priority, daemon=True)
    thread_low_priority.start()

    thread_high_priority = threading.Thread(target=high_priority, daemon=True)
    thread_high_priority.start()  # should finish first

    thread_fill_scheduler.join()
    thread_low_priority.join()
    thread_high_priority.join()
    scheduler.stop()
    assert high_priority_finished < low_priority_finished


def test_scheduler_can_execute_cyclic_tasks():
    scheduler = Scheduler()
    scheduler.start()
    global counter
    counter = 0
    runs = 10

    def func():
        global counter
        counter += 1

    cycle_time = 0.1
    assert scheduler.cyclic_execute(func=partial(func), cycle_time=cycle_time)
    time.sleep(runs * cycle_time)
    scheduler.stop()
    assert runs - 1 <= counter <= runs + 1  # account for timing issues in a non-RTOS


def test_scheduler_rejects_cyclic_execution_when_not_started():
    scheduler = Scheduler()
    for _ in range(3):
        assert not scheduler.cyclic_execute(func=partial(YES), cycle_time=1.0)
    scheduler.start()
    assert scheduler.cyclic_execute(func=partial(YES), cycle_time=1.0)
    scheduler.stop()
    for _ in range(3):
        assert not scheduler.cyclic_execute(func=partial(YES), cycle_time=1.0)


def test_scheduler_rejects_invalid_arguments_for_cyclic_execution():
    scheduler = Scheduler()
    scheduler.start()

    invalid_priorities = [0, -1, "asap", 3]
    for priority in invalid_priorities:
        assert not scheduler.cyclic_execute(
            func=partial(YES), priority=priority, cycle_time=1.0
        )

    invalid_cycle_times = [0, -1, 0.0, None]
    for cycle_time in invalid_cycle_times:
        assert not scheduler.cyclic_execute(func=partial(YES), cycle_time=cycle_time)

    scheduler.stop()


def test_scheduler_manages_cyclic_execution_from_various_threads():
    scheduler = Scheduler()
    scheduler.start()

    def run():
        assert scheduler.cyclic_execute(func=partial(YES), cycle_time=0.01)

    threads = []
    for _ in range(10):
        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        threads.append(thread)

    time.sleep(1)  # run a little
    scheduler.stop()
    for thread in threads:
        thread.join()
