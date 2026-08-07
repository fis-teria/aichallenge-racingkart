#pragma once

#include "overtake_planner/types.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace overtake_planner {

struct SafetyConstraintCommand {
  bool valid{false};
  bool stop_requested{true};
  bool release_authorized{false};
  double speed_limit_mps{0.0};
  double required_brake_decel_mps2{0.0};
  std::string reason{"uninitialized"};
};

bool safetyConstraintSemanticallyEqual(const SafetyConstraintCommand &lhs,
                                       const SafetyConstraintCommand &rhs);

SafetyConstraintCommand makeSafetyConstraint(
    const PlannerOutput &output, const EgoState &ego,
    const ReentryInputStatus &inputs, double normal_speed_limit_mps,
    double maximum_brake_decel_mps2);

class SafetyConstraintReleaseGate {
public:
  explicit SafetyConstraintReleaseGate(
      int required_safe_cycles = 3,
      bool allow_conservative_target_progress = false);

  SafetyConstraintCommand filter(const SafetyConstraintCommand &candidate);
  int safeCycles() const { return safe_cycles_; }

private:
  int required_safe_cycles_{3};
  bool allow_conservative_target_progress_{false};
  int safe_cycles_{0};
  std::optional<SafetyConstraintCommand> pending_release_target_;
  std::optional<SafetyConstraintCommand> last_filtered_;
};

} // namespace overtake_planner
