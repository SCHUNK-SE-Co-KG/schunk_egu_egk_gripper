import pytest
from schunk_gripper_library.tests.etc.pseudo_terminals import Connection


@pytest.fixture(scope="module")
def pseudo_terminals():

    connection = Connection()
    pt1, pt2 = connection.open()
    print(f"Opening two pseudo terminals:\n{pt1}\n{pt2}")
    yield (pt1, pt2)
    connection.close()
    print("Closing both pseudo terminals")
