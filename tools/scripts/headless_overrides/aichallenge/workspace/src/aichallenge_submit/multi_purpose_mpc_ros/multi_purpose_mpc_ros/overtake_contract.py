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
    speed_only: bool = False
    solver_horizon_authorized: bool = False
    mandatory_lateral_avoidance: bool = False
    longitudinal_offsets_m: tuple[float, ...] = ()


@dataclass
class OvertakeSpeedOnlyFailClosedLatch:
    """Retain only the last validated v2 cap across invalid transport input.

    A v1 lateral override and explicit inactive intentionally clear this latch.
    This makes a missing or malformed v2 fail closed longitudinally without
    retaining an old lateral trajectory.
    """

    _last_speed_only: OvertakeReferenceOverride | None = None

    def observe_valid(self, override: OvertakeReferenceOverride) -> None:
        is_valid_speed_only = (
            override.speed_only and override.mode_id > 0 and
            override.point_count == 0 and not override.lateral_offsets and
            len(override.speed_caps) == 1 and
            math.isfinite(override.speed_caps[0]) and
            override.speed_caps[0] > 0.0)
        if is_valid_speed_only:
            self._last_speed_only = override
        else:
            self.clear()

    def retained(self) -> OvertakeReferenceOverride | None:
        return self._last_speed_only

    def clear(self) -> None:
        self._last_speed_only = None


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
    """Accept exact legacy/v1, v2 speed-only, v3, or spatial v4 payloads.

    Malformed messages return ``None``. Callers keep a previously validated v2
    speed-only cap, if any, but clear v1 lateral overrides as before.
    v2 is deliberately narrow: it has no lateral points and exactly one positive
    speed cap, which must apply to every MPC horizon point. v3 appends a
    generation-bound horizon intent to a lateral payload. v4 additionally
    carries the planner's physical-distance axis for Pure Pursuit; MPC keeps
    consuming the original time-indexed lateral/speed arrays. Legacy v1 is
    accepted for transport compatibility but never authorizes a solver horizon.
    """
    if len(data) < 3:
        return None
    valid = _integer_float_in_range(data[0], 1, 1)
    mode_id = _integer_float_in_range(data[1], 0, 255)
    point_count = _integer_float_in_range(
        data[2], 0, MAX_OVERTAKE_OVERRIDE_POINTS)
    if valid is None or mode_id is None or point_count is None:
        return None

    if point_count == 0:
        # Explicit inactive: legacy/v1 n=0 only permits mode 0.  It clears
        # downstream state and is intentionally not interpreted as speed-only.
        if mode_id == 0:
            if len(data) == 3:
                return OvertakeReferenceOverride(0, 0, (), (), 0)
            if len(data) != 5:
                return None
            contract_version = _integer_float_in_range(data[3], 1, 1)
            generation = _integer_float_in_range(
                data[4], 1, MAX_OVERTAKE_OVERRIDE_GENERATION)
            if contract_version is None or generation is None:
                return None
            return OvertakeReferenceOverride(0, 0, (), (), generation)

        # v2 speed-only: [1, mode!=0, 0, 2, generation, speed_cap_mps].
        if len(data) != 6:
            return None
        contract_version = _integer_float_in_range(data[3], 2, 2)
        generation = _integer_float_in_range(
            data[4], 1, MAX_OVERTAKE_OVERRIDE_GENERATION)
        speed_cap = data[5]
        if (contract_version is None or generation is None or
                isinstance(speed_cap, bool) or not isinstance(speed_cap, Real) or
                not math.isfinite(float(speed_cap)) or float(speed_cap) <= 0.0):
            return None
        return OvertakeReferenceOverride(
            mode_id=mode_id,
            point_count=0,
            lateral_offsets=(),
            speed_caps=(float(speed_cap),),
            generation=generation,
            speed_only=True,
        )

    # v1/v3/v4 lateral + speed override. The no-trailer v1 form remains accepted
    # for old publishers, while the current Node emits v4 when a spatial axis is
    # available and v3 otherwise.
    if mode_id == 0:
        return None
    expected = 3 + 2 * point_count
    spatial_expected = expected + point_count + 3
    if len(data) not in {expected, expected + 2, expected + 3,
                         spatial_expected}:
        return None
    payload = data[3:expected]
    if any(
            isinstance(value, bool) or not isinstance(value, Real) or
            not math.isfinite(float(value))
            for value in payload):
        return None
    speed_payload = data[3 + point_count:expected]
    if any(float(value) <= 0.0 for value in speed_payload):
        return None

    generation = 0
    solver_horizon_authorized = False
    mandatory_lateral_avoidance = False
    if len(data) == expected + 2:
        contract_version = _integer_float_in_range(data[expected], 1, 1)
        generation = _integer_float_in_range(
            data[expected + 1], 1, MAX_OVERTAKE_OVERRIDE_GENERATION)
        if contract_version is None or generation is None:
            return None
    elif len(data) == expected + 3:
        contract_version = _integer_float_in_range(data[expected], 3, 3)
        generation = _integer_float_in_range(
            data[expected + 1], 1, MAX_OVERTAKE_OVERRIDE_GENERATION)
        horizon_intent = _integer_float_in_range(data[expected + 2], 0, 2)
        if (contract_version is None or generation is None or
                horizon_intent is None):
            return None
        solver_horizon_authorized = horizon_intent in {1, 2}
        mandatory_lateral_avoidance = horizon_intent == 2

    longitudinal_offsets_m: tuple[float, ...] = ()
    if len(data) == spatial_expected:
        distance_payload = data[expected:expected + point_count]
        if any(
                isinstance(value, bool) or not isinstance(value, Real) or
                not math.isfinite(float(value)) or float(value) < 0.0
                for value in distance_payload):
            return None
        longitudinal_offsets_m = tuple(float(value) for value in distance_payload)
        if (abs(longitudinal_offsets_m[0]) > 1.0e-5 or
                longitudinal_offsets_m[-1] <= 1.0e-6 or
                any(next_distance + 1.0e-6 < previous_distance
                    for previous_distance, next_distance in
                    zip(longitudinal_offsets_m, longitudinal_offsets_m[1:]))):
            return None
        trailer_index = expected + point_count
        contract_version = _integer_float_in_range(data[trailer_index], 4, 4)
        generation = _integer_float_in_range(
            data[trailer_index + 1], 1, MAX_OVERTAKE_OVERRIDE_GENERATION)
        horizon_intent = _integer_float_in_range(
            data[trailer_index + 2], 0, 2)
        if (contract_version is None or generation is None or
                horizon_intent is None):
            return None
        solver_horizon_authorized = horizon_intent in {1, 2}
        mandatory_lateral_avoidance = horizon_intent == 2

    return OvertakeReferenceOverride(
        mode_id=mode_id,
        point_count=point_count,
        lateral_offsets=tuple(float(value) for value in data[3:3 + point_count]),
        speed_caps=tuple(float(value) for value in data[3 + point_count:expected]),
        generation=generation,
        solver_horizon_authorized=solver_horizon_authorized,
        mandatory_lateral_avoidance=mandatory_lateral_avoidance,
        longitudinal_offsets_m=longitudinal_offsets_m,
    )
