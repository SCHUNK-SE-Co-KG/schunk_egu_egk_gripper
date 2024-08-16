from schunk_egu_egk_gripper_dummy.main import server, dummy
from fastapi.testclient import TestClient


def test_info_route_is_available():
    client = TestClient(server)
    assert client.get("/adi/info.json").is_success
    dummy.stop()


def test_enum_route_is_available():
    client = TestClient(server)
    assert client.get("/adi/enum.json").is_success
    dummy.stop()


def test_data_route_is_available():
    client = TestClient(server)
    assert client.get("/adi/data.json").is_success
    dummy.stop()


def test_update_route_is_available():
    client = TestClient(server)
    data = {"inst": 0, "value": "0"}
    assert client.post("/adi/update.json", data=data).is_success
    dummy.stop()
