from schunk_egu_egk_gripper_dummy.main import server
from fastapi.testclient import TestClient


def test_info_route_is_available():
    client = TestClient(server)
    assert client.get("/adi/info.json").is_success


def test_enum_route_is_available():
    client = TestClient(server)
    assert client.get("/adi/enum.json").is_success


def test_data_route_is_available():
    client = TestClient(server)
    assert client.get("/adi/data.json").is_success
