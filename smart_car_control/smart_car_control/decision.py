"""Reduce a planar scan to sector distances and select the next motion."""

import math
from dataclasses import dataclass
from enum import Enum

FRONT_ANGLE = math.radians(27.0)
SIDE_ANGLE = math.radians(105.0)
MIN_VALID_FRACTION = 0.8


class State(Enum):
    CRUISE = "CRUISE"
    AVOID = "AVOID"
    STOP = "STOP"


@dataclass(frozen=True)
class Clearance:
    front: float
    left: float
    right: float


def scan_clearance(ranges, angle_min, angle_max, angle_increment, range_min, range_max):
    metadata = (angle_min, angle_max, angle_increment, range_min, range_max)
    if not all(map(math.isfinite, metadata)):
        return None
    if not (
        0.0 < angle_increment <= math.radians(5.0)
        and angle_min <= -SIDE_ANGLE
        and angle_max >= SIDE_ANGLE
        and 0.0 < range_min < range_max
        and ranges
    ):
        return None
    last_angle = angle_min + (len(ranges) - 1) * angle_increment
    if not math.isclose(last_angle, angle_max, abs_tol=angle_increment / 2.0):
        return None

    distances = [range_max] * 3
    counts = [0] * 3
    valid = [0] * 3
    for index, distance in enumerate(ranges):
        angle = angle_min + index * angle_increment
        if abs(angle) > SIDE_ANGLE:
            continue
        if abs(angle) <= FRONT_ANGLE:
            sector = 0
        else:
            sector = 1 if angle > 0.0 else 2

        counts[sector] += 1
        if distance == math.inf:
            distance = range_max
        elif distance == -math.inf:
            distance = range_min  # A return closer than the minimum is unsafe.
        elif not range_min <= distance <= range_max:
            continue
        valid[sector] += 1
        distances[sector] = min(distances[sector], distance)

    if any(total == 0 or count < total * MIN_VALID_FRACTION
           for count, total in zip(valid, counts)):
        return None
    return Clearance(*distances)


def choose_motion(clearance, previous_turn, enter_distance, exit_distance, side_minimum):
    """Return the state, turn direction (+1 left, -1 right), and decision reason."""
    if clearance is None:
        return State.STOP, 0, "no_valid_scan"

    front_threshold = exit_distance if previous_turn else enter_distance
    if clearance.front >= front_threshold:
        return State.CRUISE, 0, "front_clear"

    # Hold the selected direction until clear to avoid alternating between sides.
    if previous_turn == 1 and clearance.left >= side_minimum:
        return State.AVOID, 1, "turn_left"
    if previous_turn == -1 and clearance.right >= side_minimum:
        return State.AVOID, -1, "turn_right"
    if max(clearance.left, clearance.right) < side_minimum:
        return State.STOP, 0, "sides_blocked"
    if clearance.left >= clearance.right:
        return State.AVOID, 1, "turn_left"
    return State.AVOID, -1, "turn_right"
