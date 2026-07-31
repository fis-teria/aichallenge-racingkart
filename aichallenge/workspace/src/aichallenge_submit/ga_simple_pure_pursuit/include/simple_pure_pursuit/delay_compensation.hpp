#ifndef SIMPLE_PURE_PURSUIT_DELAY_COMPENSATION_HPP_
#define SIMPLE_PURE_PURSUIT_DELAY_COMPENSATION_HPP_

#include <algorithm>
#include <cmath>

namespace simple_pure_pursuit {

struct EgoControlState {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double velocity_mps{0.0};
};

struct DelayedPosePrediction {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double applied_steering_rad{0.0};
  int prediction_steps{0};
  bool shifted{false};
};

inline double normalizeDelayAngle(double angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

inline double predictLaggedSteering(double current_steering_rad,
                                    double target_steering_rad, double dt_sec,
                                    double time_constant_sec) {
  if (!std::isfinite(current_steering_rad)) {
    current_steering_rad = 0.0;
  }
  if (!std::isfinite(target_steering_rad)) {
    target_steering_rad = current_steering_rad;
  }
  if (!std::isfinite(dt_sec) || dt_sec <= 0.0 ||
      !std::isfinite(time_constant_sec) || time_constant_sec <= 1.0e-6) {
    return target_steering_rad;
  }

  const double alpha = 1.0 - std::exp(-dt_sec / time_constant_sec);
  return current_steering_rad +
         std::clamp(alpha, 0.0, 1.0) *
             (target_steering_rad - current_steering_rad);
}

inline DelayedPosePrediction
predictDelayedPose(const EgoControlState &state, double current_steering_rad,
                   double target_steering_rad, double delay_sec,
                   double prediction_dt_sec, double steering_time_constant_sec,
                   double wheel_base_m) {
  DelayedPosePrediction result;
  result.x = std::isfinite(state.x) ? state.x : 0.0;
  result.y = std::isfinite(state.y) ? state.y : 0.0;
  result.yaw = std::isfinite(state.yaw) ? state.yaw : 0.0;
  result.applied_steering_rad =
      std::isfinite(current_steering_rad) ? current_steering_rad : 0.0;

  const double delay =
      std::isfinite(delay_sec) ? std::max(0.0, delay_sec) : 0.0;
  const double dt = std::isfinite(prediction_dt_sec)
                        ? std::max(1.0e-3, prediction_dt_sec)
                        : 0.02;
  const double wheel_base =
      std::isfinite(wheel_base_m) ? std::max(1.0e-3, wheel_base_m) : 1.0e-3;
  const double velocity =
      std::isfinite(state.velocity_mps) ? state.velocity_mps : 0.0;

  if (delay <= 0.0) {
    return result;
  }

  double elapsed = 0.0;
  double steering = result.applied_steering_rad;
  while (elapsed < delay - 1.0e-12) {
    const double step_dt = std::min(dt, delay - elapsed);
    steering = predictLaggedSteering(steering, target_steering_rad, step_dt,
                                     steering_time_constant_sec);
    result.x += velocity * std::cos(result.yaw) * step_dt;
    result.y += velocity * std::sin(result.yaw) * step_dt;
    result.yaw = normalizeDelayAngle(
        result.yaw + velocity / wheel_base * std::tan(steering) * step_dt);
    result.applied_steering_rad = steering;
    elapsed += step_dt;
    ++result.prediction_steps;
  }
  result.shifted = result.prediction_steps > 0;
  return result;
}

} // namespace simple_pure_pursuit

#endif // SIMPLE_PURE_PURSUIT_DELAY_COMPENSATION_HPP_
