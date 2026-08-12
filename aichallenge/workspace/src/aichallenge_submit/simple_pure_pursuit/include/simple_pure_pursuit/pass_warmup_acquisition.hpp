#pragma once

#include <cmath>

namespace simple_pure_pursuit {

struct PassWarmupSteeringAcquisition {
  bool evidence_valid{false};
  bool acquisition_required{false};
  bool measured_steering_converged{false};
  bool motion_ready{false};
};

// Separates zero-speed steering acquisition from permission to move.  A
// rate-limited command may be applied during PASS_WARMUP, but motion is ready
// only after the requested command is no longer limited and fresh measured
// steering has reached that final command.
inline PassWarmupSteeringAcquisition evaluatePassWarmupSteeringAcquisition(
    double requested_steering_rad, double bounded_steering_rad,
    double tracking_target_steering_rad, double measured_steering_rad,
    double measured_steering_age_sec, double max_measured_steering_age_sec,
    double command_tolerance_rad, double measured_tolerance_rad) noexcept {
  PassWarmupSteeringAcquisition result;
  if (!std::isfinite(requested_steering_rad) ||
      !std::isfinite(bounded_steering_rad) ||
      !std::isfinite(tracking_target_steering_rad) ||
      !std::isfinite(measured_steering_rad) ||
      !std::isfinite(measured_steering_age_sec) ||
      !std::isfinite(max_measured_steering_age_sec) ||
      !std::isfinite(command_tolerance_rad) ||
      !std::isfinite(measured_tolerance_rad) ||
      measured_steering_age_sec < 0.0 ||
      max_measured_steering_age_sec <= 0.0 || command_tolerance_rad < 0.0 ||
      measured_tolerance_rad < 0.0 ||
      measured_steering_age_sec > max_measured_steering_age_sec) {
    return result;
  }
  result.evidence_valid = true;
  result.acquisition_required =
      std::abs(requested_steering_rad - bounded_steering_rad) >
      command_tolerance_rad;
  // Output gain compensates the actuator/plant; it is not the physical tire
  // angle the vehicle is expected to report.  Convergence therefore follows
  // the controller's raw tire-angle target while limiter completion remains
  // defined by the requested-vs-bounded actuator command above.
  result.measured_steering_converged =
      std::abs(measured_steering_rad - tracking_target_steering_rad) <=
      measured_tolerance_rad;
  result.motion_ready =
      !result.acquisition_required && result.measured_steering_converged;
  return result;
}

} // namespace simple_pure_pursuit
