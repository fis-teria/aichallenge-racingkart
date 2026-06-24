import pytest

from multi_purpose_mpc_ros.speed_profile import (
    apply_combined_speed_profile,
    combine_speed_profile,
)


class ReferencePathStub:
    def __init__(self):
        self.v_ref = []

    def set_v_ref(self, values):
        self.v_ref = list(values)


def test_combine_speed_profile_uses_lowest_active_limit():
    caps = {0: 8.0, 1: 8.0}

    profile = combine_speed_profile(
        [12.0, 7.0, 10.0],
        global_cap_mps=9.0,
        section_cap_provider=lambda wp_id: caps.get(wp_id),
    )

    assert [point.target_speed_mps for point in profile] == pytest.approx([8.0, 7.0, 9.0])
    assert [point.source for point in profile] == ["section_cap", "curvature", "global_v_max"]


def test_apply_combined_speed_profile_sets_waypoint_targets():
    reference_path = ReferencePathStub()
    profile = combine_speed_profile([6.0, 7.0], global_cap_mps=10.0)

    apply_combined_speed_profile(reference_path, profile)

    assert reference_path.v_ref == pytest.approx([6.0, 7.0])
