"""Validation for the planner-to-controller Float32 overtake override contract."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Sequence


MAX_OVERTAKE_OVERRIDE_POINTS = 1000
MAX_OVERTAKE_OVERRIDE_GENERATION = 16777215


@dataclass(frozen=True)
class OvertakeReferenceOverride:
    mode_id: int
    point_count: int
    lateral_offsets: tuple[float, ...]
    speed_caps: tuple[float, ...]
    generation: int


def _integer_float_in_range(value: object, minimum: int, maximum: int) -> int | None:
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    parsed = float(value)
    if (not math.isfinite(parsed) or not parsed.is_integer() or
            parsed < minimum or parsed > maximum):
        return None
    return int(parsed)


def parse_overtake_reference_override(
    data: Sequence[object],
) -> OvertakeReferenceOverride | None:
    """Accept only an exact v0/v1 payload; malformed input has no effect."""
    if len(data) < 3:
        return None
    valid = _integer_float_in_range(data[0], 1, 1)
    mode_id = _integer_float_in_range(data[1], 0, 255)
    point_count = _integer_float_in_range(
        data[2], 0, MAX_OVERTAKE_OVERRIDE_POINTS)
    if valid is None or mode_id is None or point_count is None:
        return None

    expected = 3 + 2 * point_count
    if len(data) not in {expected, expected + 2}:
        return None
    payload = data[3:expected]
    if any(
            isinstance(value, bool) or not isinstance(value, Real) or
            not math.isfinite(float(value))
            for value in payload):
        return None

    generation = 0
    if len(data) == expected + 2:
        contract_version = _integer_float_in_range(data[expected], 1, 1)
        generation = _integer_float_in_range(
            data[expected + 1], 1, MAX_OVERTAKE_OVERRIDE_GENERATION)
        if contract_version is None or generation is None:
            return None

    return OvertakeReferenceOverride(
        mode_id=mode_id,
        point_count=point_count,
        lateral_offsets=tuple(float(value) for value in data[3:3 + point_count]),
        speed_caps=tuple(float(value) for value in data[3 + point_count:expected]),
        generation=generation,
    )
