import pytest
import rclpy

# We avoid black's F811, F401 linting warnings
# by using pytest's special conftest.py file.
# See documentation here:
# https://docs.pytest.org/en/7.1.x/reference/fixtures.html#conftest-py-sharing-fixtures-across-multiple-files  # noqa: E501


@pytest.fixture()
def isolated():
    rclpy.init()
    yield
    rclpy.shutdown()
