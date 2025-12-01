# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

from schunk_gripper_dummy.main import create_webserver
from schunk_gripper_dummy.dummy import Dummy
from fastapi.testclient import TestClient


def test_data_route_is_available():
    dummy = Dummy()
    dummy.start()
    server = create_webserver(dummy)
    client = TestClient(server)

    response = client.get("/adi/data.json")
    assert response.is_success

    dummy.stop()


def test_update_route_is_available():
    dummy = Dummy()
    dummy.start()
    server = create_webserver(dummy)
    client = TestClient(server)

    data = {"inst": "0", "value": "0"}
    response = client.post("/adi/update.json", data=data)
    assert response.is_success

    dummy.stop()


def test_events_route_is_available():
    dummy = Dummy()
    dummy.start()
    server = create_webserver(dummy)
    client = TestClient(server)

    events = {}
    response = client.post("/adi/events.json", json=events)
    assert response.is_success

    dummy.stop()
