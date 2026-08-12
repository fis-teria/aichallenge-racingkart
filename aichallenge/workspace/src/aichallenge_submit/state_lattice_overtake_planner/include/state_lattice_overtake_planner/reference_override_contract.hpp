#pragma once

#include "state_lattice_overtake_planner/types.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace state_lattice_overtake_planner {

enum class WireKind {
  INACTIVE,
  SPEED_ONLY_V2,
  LATERAL_AND_SPEED_V3,
  SPATIAL_LATERAL_AND_SPEED_V4
};

struct WirePayload {
  WireKind kind{WireKind::INACTIVE};
  std::vector<float> data;
};

struct WirePublicationPolicy {
  // Generation and authority are deliberately separate.  A lateral payload is
  // authoritative only when both gates are explicitly enabled.
  bool exact_spatial_generation_enabled{false};
  bool lateral_live_publish_enabled{false};
};

PlannerOutput prepareWireOutputForPublication(
    const PlannerOutput &output, bool v2_base_identity_available);
WirePayload makeWirePayload(const PlannerOutput &output,
                            std::uint32_t generation,
                            WirePublicationPolicy policy = {});
bool semanticallyEqual(const WirePayload &lhs, const WirePayload &rhs);
bool shouldAdvanceWireGeneration(
    const PlannerOutput &output,
    const std::optional<WirePayload> &last_payload,
    const WirePayload &prospective_payload);
std::uint32_t nextGeneration(std::uint32_t current);
bool deadlinePreviousOutputReusable(bool previous_fresh,
                                    bool previous_horizon_safe,
                                    const PlannerOutput &previous_output);
PlannerOutput makePlanningDeadlineStop(double safe_stop_speed_mps);

} // namespace state_lattice_overtake_planner
