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


def test_dummy_reads_configuration_on_startup():
    dummy = Dummy()
    assert dummy.enum is not None
    assert dummy.data is not None
    assert dummy.metadata is not None


def test_dummy_responds_correctly_to_info_requests():
    dummy = Dummy()
    path = "info.json"
    query = ""
    expected = {"dataformat": 0}
    assert dummy.get(path, query) == expected


def test_dummy_responds_correctly_to_enum_requests():
    dummy = Dummy()
    path = "enum.json"
    inst = "0x0118"
    value = 0
    query = {"inst": inst, "value": value}
    expected = [dummy.enum[inst][value]]
    assert dummy.get(path, query) == expected


def test_dummy_survives_invalid_enum_requests():
    dummy = Dummy()
    path = "enum.json"
    invalid_inst = "0x0"
    query = {"inst": invalid_inst, "value": 0}
    expected = []
    assert dummy.get(path, query) == expected


def test_dummy_responds_correctly_to_data_requests():
    dummy = Dummy()
    path = "data.json"
    query = {"offset": 15, "count": 3}
    expected = [dummy.metadata[15], dummy.metadata[16], dummy.metadata[17]]
    assert dummy.get(path, query) == expected


def test_dummy_survives_invalid_data_requests():
    dummy = Dummy()
    path = "data.json"
    query = {"offset": 1000, "count": "2"}
    expected = []
    assert dummy.get(path, query) == expected
    query = {"offset": 100, "count": "90"}
    expected = []
    assert dummy.get(path, query) == expected
    query = {"offset": 1000, "count": "-1"}
    expected = []
    assert dummy.get(path, query) == expected
    query = {"offset": -1, "count": "1000"}
    expected = []
    assert dummy.get(path, query) == expected
