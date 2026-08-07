#include "overtake_planner/state_lattice_shadow_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace overtake_planner {
StateLatticeShadowAdapterResult StateLatticeShadowAdapter::adapt(
    const state_lattice_overtake_planner::ShadowGeometryResult &geometry,
    const CandidateTrajectory &current_longitudinal_profile, CandidateType type,
    const std::string &target_id, int pass_side, double target_d_m) const {
  StateLatticeShadowAdapterResult result;
  if (!geometry.valid || geometry.dense.size() < 2U || frame_.empty() ||
      target_id.empty() || pass_side == 0 || !std::isfinite(target_d_m) ||
      !std::isfinite(geometry.raw_cost)) {
    result.reason = "invalid_shadow_geometry_or_identity";
    return result;
  }
  const std::size_t point_count = geometry.dense.size();
  if (current_longitudinal_profile.t.size() != point_count ||
      current_longitudinal_profile.longitudinal_offsets_m.size() !=
          point_count ||
      current_longitudinal_profile.predicted_speed_mps.size() != point_count ||
      current_longitudinal_profile.v_ref.size() != point_count) {
    result.reason = "shadow_longitudinal_profile_size_mismatch";
    return result;
  }
  for (std::size_t index = 0U; index < point_count; ++index) {
    if (!std::isfinite(current_longitudinal_profile.t[index]) ||
        !std::isfinite(
            current_longitudinal_profile.longitudinal_offsets_m[index]) ||
        !std::isfinite(
            current_longitudinal_profile.predicted_speed_mps[index]) ||
        !std::isfinite(current_longitudinal_profile.v_ref[index]) ||
        (index > 0U && current_longitudinal_profile.t[index] <=
                           current_longitudinal_profile.t[index - 1U]) ||
        (index > 0U &&
         current_longitudinal_profile.longitudinal_offsets_m[index] <
             current_longitudinal_profile.longitudinal_offsets_m[index - 1U])) {
      result.reason = "invalid_shadow_longitudinal_profile";
      return result;
    }
  }
  CandidateTrajectory candidate;
  candidate.type = type;
  candidate.feasible = false;
  candidate.safety_evaluated = false;
  // Reprojection alone is not a PP/trackability proof for new Cartesian
  // geometry.  Keep these false until the shared CandidateBuilder facade is
  // available; shadow records must not imply a live-authorizable trajectory.
  candidate.controller_tracking_profile_valid = false;
  candidate.desired_path_trackable = false;
  candidate.pure_pursuit_command_trackable = false;
  candidate.controller_spatial_horizon_proof_valid = false;
  candidate.planned_target_d_m = target_d_m;
  candidate.score = geometry.raw_cost;
  double previous_unwrapped_s = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t index = 0U; index < geometry.dense.size(); ++index) {
    const auto &point = geometry.dense[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.yaw)) {
      result.reason = "nonfinite_shadow_cartesian_point";
      return result;
    }
    const FrenetPose projected =
        frame_.cartesianToFrenet(point.x, point.y, point.yaw);
    if (!std::isfinite(projected.s) || !std::isfinite(projected.d) ||
        (index > 0U &&
         frame_.deltaS(previous_unwrapped_s, projected.s) <= 1.0e-6)) {
      result.reason = "shadow_frenet_projection_not_monotonic";
      return result;
    }
    // The Cartesian lattice is lateral geometry only.  Reuse the selected
    // current candidate's longitudinal s(t) profile exactly. Cartesian arc is
    // independently recomputed by CartesianTrackabilityEvaluator.
    candidate.t.push_back(current_longitudinal_profile.t[index]);
    candidate.longitudinal_offsets_m.push_back(
        current_longitudinal_profile.longitudinal_offsets_m[index]);
    candidate.s.push_back(projected.s);
    candidate.d.push_back(projected.d);
    candidate.x.push_back(point.x);
    candidate.y.push_back(point.y);
    candidate.yaw.push_back(point.yaw);
    candidate.predicted_speed_mps.push_back(
        current_longitudinal_profile.predicted_speed_mps[index]);
    candidate.v_ref.push_back(current_longitudinal_profile.v_ref[index]);
    previous_unwrapped_s = projected.s;
  }
  // Parametric quinticはcurrent smootherstepと同形である必要はない。adapterは
  // 座標・時刻契約だけを検証し、幾何の可追従性は直後のCartesian exact evaluator
  // が判定する。ここでfeasible/safety_evaluatedを立てない境界は維持する。
  result.valid = true;
  result.reason = "ok_unevaluated_shadow_candidate";
  result.target_id = target_id;
  result.pass_side = pass_side;
  result.target_d_m = target_d_m;
  result.candidate = std::move(candidate);
  result.raw_cost = geometry.raw_cost;
  return result;
}

} // namespace overtake_planner
