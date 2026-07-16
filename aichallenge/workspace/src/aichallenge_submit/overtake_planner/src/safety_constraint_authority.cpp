#include "overtake_planner/safety_constraint_authority.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace overtake_planner {

namespace {

double minimumPositiveSpeedCap(const PlannerOutput &output,
                               double fallback_mps) {
  double cap_mps = fallback_mps;
  if (std::isfinite(output.applied_speed_cap_mps) &&
      output.applied_speed_cap_mps > 0.0) {
    cap_mps = std::min(cap_mps, output.applied_speed_cap_mps);
  }
  if (output.active_override || output.longitudinal_speed_cap_active) {
    for (const double candidate_cap_mps : output.speed_caps) {
      if (std::isfinite(candidate_cap_mps) && candidate_cap_mps > 0.0) {
        cap_mps = std::min(cap_mps, candidate_cap_mps);
      }
    }
  }
  return cap_mps;
}

constexpr double kConstraintTolerance = 1.0e-6;

bool sameReleaseTarget(const SafetyConstraintCommand &lhs,
                       const SafetyConstraintCommand &rhs) {
  return lhs.valid == rhs.valid &&
         lhs.stop_requested == rhs.stop_requested &&
         std::abs(lhs.speed_limit_mps - rhs.speed_limit_mps) <=
             kConstraintTolerance &&
         std::abs(lhs.required_brake_decel_mps2 -
                  rhs.required_brake_decel_mps2) <= kConstraintTolerance;
}

bool relaxesConstraint(const SafetyConstraintCommand &candidate,
                       const SafetyConstraintCommand &previous) {
  return (!previous.valid && candidate.valid) ||
         (previous.stop_requested && !candidate.stop_requested) ||
         candidate.speed_limit_mps >
             previous.speed_limit_mps + kConstraintTolerance ||
         candidate.required_brake_decel_mps2 + kConstraintTolerance <
             previous.required_brake_decel_mps2;
}

SafetyConstraintCommand holdPreviousRelaxations(
    const SafetyConstraintCommand &candidate,
    const SafetyConstraintCommand &previous) {
  SafetyConstraintCommand filtered = candidate;
  filtered.valid = candidate.valid && previous.valid;
  filtered.stop_requested =
      candidate.stop_requested || previous.stop_requested;
  filtered.speed_limit_mps =
      std::min(candidate.speed_limit_mps, previous.speed_limit_mps);
  filtered.required_brake_decel_mps2 =
      std::max(candidate.required_brake_decel_mps2,
               previous.required_brake_decel_mps2);
  filtered.release_authorized = false;
  filtered.reason = "release_pending_safe_cycles";
  return filtered;
}

} // namespace

bool safetyConstraintSemanticallyEqual(const SafetyConstraintCommand &lhs,
                                       const SafetyConstraintCommand &rhs) {
  return lhs.valid == rhs.valid &&
         lhs.stop_requested == rhs.stop_requested &&
         lhs.release_authorized == rhs.release_authorized &&
         std::abs(lhs.speed_limit_mps - rhs.speed_limit_mps) <=
             kConstraintTolerance &&
         std::abs(lhs.required_brake_decel_mps2 -
                  rhs.required_brake_decel_mps2) <= kConstraintTolerance &&
         lhs.reason == rhs.reason;
}

SafetyConstraintCommand makeSafetyConstraint(
    const PlannerOutput &output, const EgoState &ego,
    const ReentryInputStatus &inputs, double normal_speed_limit_mps,
    double maximum_brake_decel_mps2) {
  SafetyConstraintCommand command;
  const double normal_limit_mps =
      std::isfinite(normal_speed_limit_mps) && normal_speed_limit_mps > 0.0
          ? normal_speed_limit_mps
          : 1.0e-3;
  const double brake_decel_mps2 =
      std::isfinite(maximum_brake_decel_mps2) &&
              maximum_brake_decel_mps2 > 0.0
          ? maximum_brake_decel_mps2
          : 0.0;
  const bool safety_inputs_complete =
      inputs.ego_fresh && inputs.v2x_snapshot_fresh &&
      inputs.all_observed_opponents_fresh &&
      inputs.all_observed_opponents_included && inputs.reference_valid;
  command.valid = true;
  command.speed_limit_mps = minimumPositiveSpeedCap(output, normal_limit_mps);
  command.stop_requested = !safety_inputs_complete || !ego.valid ||
                           output.safe_stop_triggered ||
                           output.mode == BehaviorMode::SAFE_STOP;
  if (!std::isfinite(command.speed_limit_mps) ||
      command.speed_limit_mps <= 0.0) {
    command.valid = false;
    command.stop_requested = true;
    command.speed_limit_mps = 1.0e-3;
    command.reason = "invalid_speed_limit";
  } else if (!safety_inputs_complete || !ego.valid) {
    command.reason = "safety_input_incomplete";
  } else if (output.safe_stop_triggered ||
             output.mode == BehaviorMode::SAFE_STOP) {
    command.reason = output.safe_stop_reason.empty() ? "safe_stop"
                                                     : output.safe_stop_reason;
  } else if (!output.speed_cap_reason.empty()) {
    command.reason = output.speed_cap_reason;
  } else if (!output.reason.empty()) {
    command.reason = output.reason;
  } else {
    command.reason = "normal_limit";
  }

  const bool braking_required =
      command.stop_requested ||
      (ego.valid && std::isfinite(ego.v) &&
       ego.v > command.speed_limit_mps + 1.0e-3);
  command.required_brake_decel_mps2 =
      braking_required ? brake_decel_mps2 : 0.0;

  const bool guard_active =
      output.speed_only_fallback_active || output.wall_risk_speed_guard_active ||
      output.mpc_health_speed_guard_active ||
      output.recovery_speed_guard_active || output.reentry_gate.requested ||
      output.published_lateral_safety_rejected;
  // MPC health alone does not prove that the mux-selected PP/recovery tracker
  // is unusable.  The final mux independently stops on source timeout.  Until
  // ControllerTrackingStatus is wired in Phase 3, this transitional authority
  // may keep/tighten a cap but must not disable every fallback solely because
  // the MPC horizon is unhealthy.
  const bool safe_brake_release = !braking_required;
  command.release_authorized = command.valid && !command.stop_requested &&
                               safety_inputs_complete &&
                               (!guard_active || safe_brake_release);
  return command;
}

SafetyConstraintReleaseGate::SafetyConstraintReleaseGate(
    int required_safe_cycles)
    : required_safe_cycles_(std::max(1, required_safe_cycles)) {}

SafetyConstraintCommand SafetyConstraintReleaseGate::filter(
    const SafetyConstraintCommand &candidate) {
  const auto reset_pending_release = [this]() {
    safe_cycles_ = 0;
    pending_release_target_.reset();
  };

  if (!last_filtered_.has_value()) {
    SafetyConstraintCommand filtered = candidate;
    filtered.release_authorized = false;
    if (candidate.release_authorized) {
      pending_release_target_ = candidate;
      safe_cycles_ = 1;
      if (safe_cycles_ >= required_safe_cycles_) {
        filtered.release_authorized = true;
      } else {
        filtered.reason = "release_pending_safe_cycles";
      }
    }
    last_filtered_ = filtered;
    return filtered;
  }

  const auto previous = *last_filtered_;
  if (!candidate.release_authorized) {
    reset_pending_release();
    auto filtered = candidate;
    filtered.release_authorized = false;
    if (relaxesConstraint(candidate, previous)) {
      filtered = holdPreviousRelaxations(candidate, previous);
    }
    last_filtered_ = filtered;
    return filtered;
  }

  if (!pending_release_target_.has_value() ||
      !sameReleaseTarget(candidate, *pending_release_target_)) {
    pending_release_target_ = candidate;
    safe_cycles_ = 0;
  }
  safe_cycles_ = std::min(required_safe_cycles_, safe_cycles_ + 1);
  SafetyConstraintCommand filtered = candidate;
  if (safe_cycles_ < required_safe_cycles_) {
    filtered.release_authorized = false;
    if (relaxesConstraint(candidate, previous)) {
      filtered = holdPreviousRelaxations(candidate, previous);
    } else {
      filtered.reason = "release_pending_safe_cycles";
    }
  }
  last_filtered_ = filtered;
  return filtered;
}

} // namespace overtake_planner
