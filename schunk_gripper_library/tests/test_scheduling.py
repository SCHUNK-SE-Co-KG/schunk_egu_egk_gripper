from ..schunk_gripper_library.utility import Task, Scheduler
import threading
from queue import PriorityQueue
from functools import partial
from concurrent.futures import Future


def YES() -> bool:
    return True


def NO() -> bool:
    return False


def test_tasks_have_expected_initial_values():
    task1 = Task()
    assert task1.func is None
    assert task1.future is None
    task2 = Task(func=partial(YES), future=Future())
    assert task2


def test_tasks_have_boolean_meaning():
    task = Task()
    assert not task
    task.func = "some function"
    assert task


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


def test_scheduler_executes_tasks():
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
        assert not scheduler.execute(func=YES, priority=priority).result()
