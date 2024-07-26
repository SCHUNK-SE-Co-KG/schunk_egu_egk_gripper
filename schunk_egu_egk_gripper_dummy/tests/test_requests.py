from src.dummy import Dummy


def test_dummy_responds_correctly_to_info_requests():
    dummy = Dummy()
    query = ""
    expected = {"dataformat": 0}
    assert dummy.get_info(query) == expected


def test_dummy_responds_correctly_to_enum_requests():
    dummy = Dummy()
    inst = "0x0118"
    value = 0
    query = {"inst": inst, "value": value}
    expected = [dummy.enum[inst][value]]
    assert dummy.get_enum(query) == expected


def test_dummy_survives_invalid_enum_requests():
    dummy = Dummy()
    invalid_inst = "0x0"
    query = {"inst": invalid_inst, "value": 0}
    expected = []
    assert dummy.get_enum(query) == expected


def test_dummy_responds_correctly_to_data_offset_requests():
    dummy = Dummy()
    query = {"offset": 15, "count": 3}
    expected = [dummy.metadata[15], dummy.metadata[16], dummy.metadata[17]]
    assert dummy.get_data(query) == expected


def test_dummy_responds_correctly_to_data_instance_requests():
    dummy = Dummy()
    inst = "0x0040"
    query = {"inst": inst, "count": 1}
    expected = dummy.data[inst]
    assert dummy.get_data(query) == expected


def test_dummy_survives_invalid_data_requests():
    dummy = Dummy()
    query = {"offset": 1000, "count": "2"}
    expected = []
    assert dummy.get_data(query) == expected
    query = {"offset": 100, "count": "90"}
    expected = []
    assert dummy.get_data(query) == expected
    query = {"offset": 1000, "count": "-1"}
    expected = []
    assert dummy.get_data(query) == expected
    query = {"offset": -1, "count": "1000"}
    expected = []
    assert dummy.get_data(query) == expected
    query = {"inst": "0x0040", "count": "0"}
    expected = []
    assert dummy.get_data(query) == expected
    query = {"inst": "0x0040", "count": "2"}
    expected = []
    assert dummy.get_data(query) == expected


def test_dummy_responds_correctly_to_post_requests():
    dummy = Dummy()
    inst = "0x0048"
    data = {"inst": inst, "value": "01"}
    expected = {"result": 0}
    assert dummy.post(data) == expected
