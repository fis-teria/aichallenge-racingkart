#pragma once

#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/types.hpp"
#include "state_lattice_overtake_planner/shadow_geometry_generator.hpp"

#include <limits>
#include <string>

namespace overtake_planner
{

// Untrusted lattice geometry is projected into an unevaluated current-planner
// candidate.  It intentionally does not call SafetyEvaluator or alter Core.
struct StateLatticeShadowAdapterResult
{
  bool valid{false};
  std::string reason{"not_adapted"};
  std::string target_id{};
  int pass_side{0};
  double target_d_m{std::numeric_limits<double>::quiet_NaN()};
  CandidateTrajectory candidate{};
  double raw_cost{0.0};
};

class StateLatticeShadowAdapter
{
public:
  explicit StateLatticeShadowAdapter(const FrenetFrame & frame) : frame_(frame) {}
  StateLatticeShadowAdapterResult adapt(
    const state_lattice_overtake_planner::ShadowGeometryResult & geometry,
    const CandidateTrajectory & current_longitudinal_profile,
    CandidateType type, const std::string & target_id, int pass_side,
    double target_d_m) const;

private:
  const FrenetFrame & frame_;
};

}  // namespace overtake_planner
