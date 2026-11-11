import math

import pytest

from csc376_franky import protocol
from csc376_franky.protocol import ErrorCodes


def test_old_error_codes_keep_their_values():
    # The values of the old csc376_bind_franky.ErrorCodes.
    assert ErrorCodes.Success == 1
    assert ErrorCodes.JointSizeNotSeven == -1
    assert ErrorCodes.JointDistancesAboveMaxJointDifference == -2
    assert ErrorCodes.JointVelocitiesAboveMaximumAllowed == -3
    assert ErrorCodes.DtIsNegative == -4
    assert ErrorCodes.DtIsSmallerThanAllowed == -5
    assert ErrorCodes.Interrupted == -6
    assert ErrorCodes.JointTrajectoryDoesNotStartAtCurrentPosition == -7
    assert ErrorCodes.TrajectorySizeLessThanTwo == -8


def test_error_codes_match_the_controller(native):
    for name, member in native.ErrorCodes.__members__.items():
        assert ErrorCodes[name] == int(member), name


def test_messages_are_single_lines():
    line = protocol.encode({"op": "execute", "trajectory": [[0.1] * 7] * 2, "note": "a\nb"})
    assert line.endswith(b"\n") and line.count(b"\n") == 1
    assert protocol.decode(line)["note"] == "a\nb"


def test_non_finite_numbers_are_refused():
    with pytest.raises(ValueError):
        protocol.encode({"dt": math.nan})
    with pytest.raises(ValueError):
        protocol.decode(b"[1, 2]\n")
