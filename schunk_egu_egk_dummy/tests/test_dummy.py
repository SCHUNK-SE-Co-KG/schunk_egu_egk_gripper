from src.dummy import Dummy


def test_dummy_starts_a_background_thread():
    dummy = Dummy()
    assert not dummy.running
    dummy.start()
    assert dummy.running
    dummy.stop()
    assert not dummy.running


def test_dummy_survives_repeated_starts_and_stops():
    dummy = Dummy()
    for _ in range(3):
        dummy.start()
    assert dummy.running

    for _ in range(3):
        dummy.stop()
    assert not dummy.running


def test_dummy_processes_messages():
    dummy = Dummy()
    msg = "Hello simulator!"
    assert dummy.process(msg)
