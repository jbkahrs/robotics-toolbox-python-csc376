import pytest

from helpers import make_station


@pytest.fixture(scope="session")
def native():
    return pytest.importorskip(
        "csc376_franky_server._control", reason="the robot server is not built"
    )


@pytest.fixture
def station(native):
    station, close = make_station(native)
    yield station
    close()
