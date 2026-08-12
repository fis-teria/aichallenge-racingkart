#include "state_lattice_overtake_planner/reference_override_contract.hpp"

#include <algorithm>
#include <cmath>

namespace state_lattice_overtake_planner {
namespace {
constexpr std::uint32_t kMaxGeneration = 16777215U;
}

PlannerOutput prepareWireOutputForPublication(
    const PlannerOutput &output, bool v2_base_identity_available) {
  PlannerOutput wire_output = output;
  if (v2_base_identity_available) {
    wire_output.spatial_profile_shadow_only = false;
  }
  return wire_output;
}

WirePayload makeWirePayload(const PlannerOutput &output,
                            std::uint32_t generation,
                            WirePublicationPolicy policy) {
  const auto valid_generation = std::clamp(generation, 1U, kMaxGeneration);
  const int mode = static_cast<int>(output.mode);
  const bool valid_lateral =
      output.active && output.safe_lateral && !output.emergency_stop &&
      output.mode != BehaviorMode::SAFE_STOP &&
      output.intent != SolverHorizonIntent::NONE && mode > 0 &&
      !output.lateral_offsets_m.empty() &&
      output.lateral_offsets_m.size() == output.speed_caps_mps.size() &&
      output.lateral_offsets_m.size() <= 1000U &&
      std::all_of(output.lateral_offsets_m.begin(),
                  output.lateral_offsets_m.end(),
                  [](double value) { return std::isfinite(value); }) &&
      std::all_of(
          output.speed_caps_mps.begin(), output.speed_caps_mps.end(),
          [](double value) { return std::isfinite(value) && value > 0.0; });
  const bool valid_spatial =
      output.longitudinal_offsets_m.size() == output.lateral_offsets_m.size() &&
      !output.longitudinal_offsets_m.empty() &&
      std::abs(output.longitudinal_offsets_m.front()) <= 1.0e-5 &&
      output.longitudinal_offsets_m.back() > 1.0e-6 &&
      std::all_of(
          output.longitudinal_offsets_m.begin(),
          output.longitudinal_offsets_m.end(),
          [](double value) { return std::isfinite(value) && value >= 0.0; }) &&
      std::is_sorted(output.longitudinal_offsets_m.begin(),
                     output.longitudinal_offsets_m.end());
  const bool lateral_authority_eligible =
      policy.exact_spatial_generation_enabled &&
      policy.lateral_live_publish_enabled &&
      !output.spatial_profile_shadow_only;
  if (valid_lateral && valid_spatial && lateral_authority_eligible) {
    WirePayload payload;
    payload.kind = WireKind::SPATIAL_LATERAL_AND_SPEED_V4;
    payload.data = {1.0F, static_cast<float>(mode),
                    static_cast<float>(output.lateral_offsets_m.size())};
    for (const double value : output.lateral_offsets_m) {
      payload.data.push_back(static_cast<float>(value));
    }
    for (const double value : output.speed_caps_mps) {
      payload.data.push_back(static_cast<float>(value));
    }
    for (const double value : output.longitudinal_offsets_m) {
      payload.data.push_back(static_cast<float>(value));
    }
    payload.data.push_back(4.0F);
    payload.data.push_back(static_cast<float>(valid_generation));
    payload.data.push_back(static_cast<float>(static_cast<int>(output.intent)));
    return payload;
  }
  // There is intentionally no V3 downgrade.  Missing exact-spatial binding or
  // either disabled gate falls through to the speed-only fail-closed path.
  if (output.active && mode > 0 && std::isfinite(output.speed_cap_mps) &&
      output.speed_cap_mps > 0.0) {
    return {WireKind::SPEED_ONLY_V2,
            {1.0F, static_cast<float>(mode), 0.0F, 2.0F,
             static_cast<float>(valid_generation),
             static_cast<float>(output.speed_cap_mps)}};
  }
  return {WireKind::INACTIVE,
          {1.0F, 0.0F, 0.0F, 1.0F, static_cast<float>(valid_generation)}};
}

bool semanticallyEqual(const WirePayload &lhs, const WirePayload &rhs) {
  if (lhs.kind != rhs.kind || lhs.data.size() != rhs.data.size() ||
      lhs.data.empty()) {
    return false;
  }
  const std::size_t generation_index =
      lhs.kind == WireKind::SPEED_ONLY_V2 ? 4U
      : (lhs.kind == WireKind::LATERAL_AND_SPEED_V3 ||
         lhs.kind == WireKind::SPATIAL_LATERAL_AND_SPEED_V4)
          ? lhs.data.size() - 2U
          : lhs.data.size() - 1U;
  for (std::size_t i = 0; i < lhs.data.size(); ++i) {
    if (i != generation_index && lhs.data[i] != rhs.data[i]) {
      return false;
    }
  }
  return true;
}

bool shouldAdvanceWireGeneration(
    const PlannerOutput &output,
    const std::optional<WirePayload> &last_payload,
    const WirePayload &prospective_payload) {
  return output.execution_geometry_kind ==
             ExecutionGeometryKind::EXACT_CARTESIAN ||
         !last_payload.has_value() ||
         !semanticallyEqual(last_payload.value(), prospective_payload);
}

std::uint32_t nextGeneration(std::uint32_t current) {
  return current >= kMaxGeneration || current == 0U ? 1U : current + 1U;
}

bool deadlinePreviousOutputReusable(bool previous_fresh,
                                    bool previous_horizon_safe,
                                    const PlannerOutput &previous_output) {
  return previous_fresh && previous_horizon_safe &&
         !previous_output.preventive_side_role_active;
}

PlannerOutput makePlanningDeadlineStop(double safe_stop_speed_mps) {
  PlannerOutput output;
  output.active = true;
  output.mode = BehaviorMode::SAFE_STOP;
  output.intent = SolverHorizonIntent::NONE;
  output.emergency_stop = true;
  output.safe_lateral = false;
  output.lateral_offsets_m.clear();
  output.speed_caps_mps.clear();
  output.longitudinal_offsets_m.clear();
  output.speed_cap_mps = safe_stop_speed_mps;
  output.reason = "planning_deadline";
  return output;
}

} // namespace state_lattice_overtake_planner
