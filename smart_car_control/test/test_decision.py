import math

import pytest

from smart_car_control.decision import Clearance, State, choose_motion, scan_clearance


def read_scan(ranges, increment=math.pi / 360):
    return scan_clearance(ranges, -math.pi, math.pi, increment, 0.1, 8.0)


def motion(values, turn=0):
    return choose_motion(values, turn, 0.60, 0.82, 0.30)[:2]


def test_scan_splits_three_sectors():
    ranges = [math.inf] * 721
    ranges[360] = 0.5
    ranges[460] = 1.2
    ranges[260] = 2.2
    assert read_scan(ranges) == Clearance(0.5, 1.2, 2.2)


def test_infinite_returns_distinguish_open_space_and_too_close():
    ranges = [math.inf] * 721
    assert read_scan(ranges) == Clearance(8.0, 8.0, 8.0)
    ranges[360] = -math.inf
    assert read_scan(ranges) == Clearance(0.1, 8.0, 8.0)


@pytest.mark.parametrize("invalid", [math.nan, 0.0, 9.0])
def test_invalid_front_sector_stops(invalid):
    ranges = [math.inf] * 721
    ranges[310:410] = [invalid] * 100
    assert motion(read_scan(ranges)) == (State.STOP, 0)


def test_malformed_angle_metadata_is_rejected():
    assert read_scan([math.inf] * 721, increment=0.001) is None


def test_turn_hysteresis_latches_until_front_is_clear():
    assert motion(Clearance(0.5, 2.0, 1.0)) == (State.AVOID, 1)
    assert motion(Clearance(0.7, 1.0, 2.0), turn=1) == (State.AVOID, 1)
    assert motion(Clearance(0.9, 1.0, 2.0), turn=1) == (State.CRUISE, 0)


def test_blocked_turn_switches_only_when_other_side_is_safe():
    assert motion(Clearance(0.3, 0.2, 0.4), turn=1) == (State.AVOID, -1)
    assert motion(Clearance(0.3, 0.2, 0.2), turn=1) == (State.STOP, 0)
