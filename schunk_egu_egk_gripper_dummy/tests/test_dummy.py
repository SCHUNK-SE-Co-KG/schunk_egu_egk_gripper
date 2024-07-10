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


def test_dummy_responds_correctly_to_info_requests():
    dummy = Dummy()
    path = "info.json"
    query = ""
    expected = {}
    assert dummy.process_get(path, query) == expected


def test_dummy_responds_correctly_to_enum_requests():
    dummy = Dummy()
    path = "enum.json"
    query = ""
    expected = []
    assert dummy.process_get(path, query) == expected


def test_dummy_responds_correctly_to_data_requests():
    dummy = Dummy()
    path = "data.json"
    query = ""
    expected = []
    assert dummy.process_get(path, query) == expected
