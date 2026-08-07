#include "overtake_planner/cartesian_trackability_evaluator.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace overtake_planner {
namespace {

struct Sample {
  double x_m{0.0};
  double y_m{0.0};
  double time_sec{0.0};
  double arc_m{0.0};
  double longitudinal_offset_m{0.0};
  double predicted_speed_mps{0.0};
  double v_ref_mps{0.0};
};

const char *invalidConfigReason(const CartesianTrackabilityConfig &config) {
  constexpr double kHalfPi = 1.57079632679489661923;
  if (!std::isfinite(config.wheelbase_m) || config.wheelbase_m <= 0.0) {
    return "invalid_wheelbase_config";
  }
  if (!std::isfinite(config.steering_gain) || config.steering_gain <= 0.0) {
    return "invalid_steering_gain_config";
  }
  if (!std::isfinite(config.max_steering_angle_rad) ||
      config.max_steering_angle_rad <= 0.0 ||
      config.max_steering_angle_rad >= kHalfPi) {
    return "invalid_steering_angle_limit_config";
  }
  if (!std::isfinite(config.max_steering_rate_radps) ||
      config.max_steering_rate_radps <= 0.0) {
    return "invalid_steering_rate_limit_config";
  }
  if (!std::isfinite(config.max_yaw_tangent_error_rad) ||
      config.max_yaw_tangent_error_rad <= 0.0 ||
      config.max_yaw_tangent_error_rad > 3.14159265358979323846) {
    return "invalid_yaw_tangent_error_config";
  }
  if (!std::isfinite(config.speed_consistency_abs_tolerance_mps) ||
      config.speed_consistency_abs_tolerance_mps < 0.0 ||
      !std::isfinite(config.speed_consistency_relative_tolerance) ||
      config.speed_consistency_relative_tolerance < 0.0 ||
      config.speed_consistency_relative_tolerance > 1.0 ||
      !std::isfinite(config.speed_reference_envelope_tolerance_mps) ||
      config.speed_reference_envelope_tolerance_mps < 0.0) {
    return "invalid_speed_consistency_config";
  }
  if (!std::isfinite(config.pure_pursuit_required_arc_m) ||
      config.pure_pursuit_required_arc_m <= 0.0) {
    return "invalid_pure_pursuit_arc_config";
  }
  if (!std::isfinite(config.target_d_m)) {
    return "invalid_target_d_config";
  }
  if (!std::isfinite(config.target_d_deadline_arc_m) ||
      config.target_d_deadline_arc_m < 0.0) {
    return "invalid_target_d_deadline_config";
  }
  if (!std::isfinite(config.target_d_tolerance_m) ||
      config.target_d_tolerance_m <= 0.0) {
    return "invalid_target_d_tolerance_config";
  }
  return nullptr;
}

double signedCurvature(const Sample &a, const Sample &b, const Sample &c,
                       bool *valid) {
  const double ab_m = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
  const double bc_m = std::hypot(c.x_m - b.x_m, c.y_m - b.y_m);
  const double ca_m = std::hypot(a.x_m - c.x_m, a.y_m - c.y_m);
  const double denominator = ab_m * bc_m * ca_m;
  if (!std::isfinite(denominator) || denominator <= 1.0e-15) {
    *valid = false;
    return 0.0;
  }
  const double twice_signed_area =
      (b.x_m - a.x_m) * (c.y_m - a.y_m) - (b.y_m - a.y_m) * (c.x_m - a.x_m);
  const double curvature_m_inv = 2.0 * twice_signed_area / denominator;
  *valid = std::isfinite(curvature_m_inv);
  return curvature_m_inv;
}

double sampleYaw(
    const std::array<Sample, CartesianTrackabilityEvaluator::kMaxPointCount>
        &samples,
    std::size_t sample_count, std::size_t index) {
  if (index + 1U < sample_count) {
    return std::atan2(samples[index + 1U].y_m - samples[index].y_m,
                      samples[index + 1U].x_m - samples[index].x_m);
  }
  return std::atan2(samples[index].y_m - samples[index - 1U].y_m,
                    samples[index].x_m - samples[index - 1U].x_m);
}

} // namespace

CartesianTrackabilityResult CartesianTrackabilityEvaluator::evaluate(
    const CandidateTrajectory &candidate,
    const CartesianTrackabilityInput &input,
    const CartesianTrackabilityConfig &config) const {
  CartesianTrackabilityResult result;
  if (frame_.empty() || frame_.length() <= 0.0) {
    result.reason = "invalid_frame";
    return result;
  }
  if (const char *config_reason = invalidConfigReason(config)) {
    result.reason = config_reason;
    return result;
  }

  const std::size_t source_count = candidate.x.size();
  if (source_count < 3U || source_count > kMaxPointCount ||
      candidate.y.size() != source_count ||
      candidate.yaw.size() != source_count ||
      candidate.t.size() != source_count ||
      candidate.longitudinal_offsets_m.size() != source_count ||
      candidate.predicted_speed_mps.size() != source_count ||
      candidate.v_ref.size() != source_count) {
    result.reason = "invalid_cartesian_candidate_size";
    return result;
  }

  std::size_t resampled_count = 1U;
  for (std::size_t index = 0U; index < source_count; ++index) {
    if (!std::isfinite(candidate.x[index]) ||
        !std::isfinite(candidate.y[index]) ||
        !std::isfinite(candidate.yaw[index]) ||
        !std::isfinite(candidate.t[index]) ||
        !std::isfinite(candidate.longitudinal_offsets_m[index]) ||
        !std::isfinite(candidate.predicted_speed_mps[index]) ||
        candidate.predicted_speed_mps[index] < 0.0 ||
        !std::isfinite(candidate.v_ref[index]) ||
        candidate.v_ref[index] < 0.0 ||
        (index > 0U && candidate.t[index] <= candidate.t[index - 1U]) ||
        (index > 0U && candidate.longitudinal_offsets_m[index] <
                           candidate.longitudinal_offsets_m[index - 1U])) {
      result.reason = "nonfinite_or_nonmonotonic_cartesian_candidate";
      return result;
    }
    if (index == 0U) {
      continue;
    }
    const double segment_m =
        std::hypot(candidate.x[index] - candidate.x[index - 1U],
                   candidate.y[index] - candidate.y[index - 1U]);
    if (!std::isfinite(segment_m) || segment_m <= 0.0) {
      result.reason = "zero_or_invalid_cartesian_segment";
      return result;
    }
    const double subdivisions_real =
        std::ceil(segment_m / kMaxResampleSpacingM);
    if (!std::isfinite(subdivisions_real) || subdivisions_real < 1.0 ||
        subdivisions_real > static_cast<double>(kMaxPointCount) ||
        resampled_count >
            kMaxPointCount - static_cast<std::size_t>(subdivisions_real)) {
      result.reason = "resampled_point_limit_exceeded";
      return result;
    }
    resampled_count += static_cast<std::size_t>(subdivisions_real);
  }

  for (std::size_t index = 0U; index < source_count; ++index) {
    const std::size_t tangent_from_index =
        index + 1U < source_count ? index : index - 1U;
    const std::size_t tangent_to_index =
        index + 1U < source_count ? index + 1U : index;
    const double tangent_yaw_rad = std::atan2(
        candidate.y[tangent_to_index] - candidate.y[tangent_from_index],
        candidate.x[tangent_to_index] - candidate.x[tangent_from_index]);
    const double yaw_error_rad =
        std::atan2(std::sin(candidate.yaw[index] - tangent_yaw_rad),
                   std::cos(candidate.yaw[index] - tangent_yaw_rad));
    if (!std::isfinite(tangent_yaw_rad) || !std::isfinite(yaw_error_rad) ||
        std::abs(yaw_error_rad) > config.max_yaw_tangent_error_rad) {
      result.reason = "cartesian_yaw_tangent_mismatch";
      return result;
    }
  }

  std::array<Sample, kMaxPointCount> samples{};
  std::size_t sample_count = 1U;
  samples[0U] = {candidate.x.front(),
                 candidate.y.front(),
                 candidate.t.front(),
                 0.0,
                 candidate.longitudinal_offsets_m.front(),
                 candidate.predicted_speed_mps.front(),
                 candidate.v_ref.front()};
  double total_arc_m = 0.0;
  for (std::size_t index = 1U; index < source_count; ++index) {
    const double dx_m = candidate.x[index] - candidate.x[index - 1U];
    const double dy_m = candidate.y[index] - candidate.y[index - 1U];
    const double dt_sec = candidate.t[index] - candidate.t[index - 1U];
    const double segment_m = std::hypot(dx_m, dy_m);
    const std::size_t subdivisions =
        static_cast<std::size_t>(std::ceil(segment_m / kMaxResampleSpacingM));
    for (std::size_t subdivision = 1U; subdivision <= subdivisions;
         ++subdivision) {
      const double ratio =
          static_cast<double>(subdivision) / static_cast<double>(subdivisions);
      const double arc_m = total_arc_m + segment_m *
                                             static_cast<double>(subdivision) /
                                             static_cast<double>(subdivisions);
      samples[sample_count++] = {
          candidate.x[index - 1U] + ratio * dx_m,
          candidate.y[index - 1U] + ratio * dy_m,
          candidate.t[index - 1U] + ratio * dt_sec,
          arc_m,
          candidate.longitudinal_offsets_m[index - 1U] +
              ratio * (candidate.longitudinal_offsets_m[index] -
                       candidate.longitudinal_offsets_m[index - 1U]),
          candidate.predicted_speed_mps[index - 1U] +
              ratio * (candidate.predicted_speed_mps[index] -
                       candidate.predicted_speed_mps[index - 1U]),
          candidate.v_ref[index - 1U] +
              ratio * (candidate.v_ref[index] - candidate.v_ref[index - 1U])};
    }
    total_arc_m += segment_m;
  }
  if (sample_count != resampled_count || !std::isfinite(total_arc_m)) {
    result.reason = "invalid_resampling_result";
    return result;
  }
  result.resampled_point_count = sample_count;
  result.total_arc_m = total_arc_m;
  result.max_abs_speed_consistency_error_mps = 0.0;
  const double initial_predicted_speed_mps =
      samples.front().predicted_speed_mps;
  for (std::size_t index = 0U; index < sample_count; ++index) {
    const double lower_speed_mps =
        std::min(initial_predicted_speed_mps, samples[index].v_ref_mps) -
        config.speed_reference_envelope_tolerance_mps;
    const double upper_speed_mps =
        std::max(initial_predicted_speed_mps, samples[index].v_ref_mps) +
        config.speed_reference_envelope_tolerance_mps;
    // v_refは即時指令、predicted_speedは応答遅れを含む実行予測なので、
    // braking中のpredicted > v_refは正しい。両者のどちらよりも外側へ
    // 発散する列だけを不整合として拒否する。
    if (samples[index].predicted_speed_mps < lower_speed_mps ||
        samples[index].predicted_speed_mps > upper_speed_mps) {
      result.reason = "predicted_speed_outside_v_ref_envelope";
      return result;
    }
  }
  for (std::size_t index = 1U; index < sample_count; ++index) {
    const double dt_sec =
        samples[index].time_sec - samples[index - 1U].time_sec;
    const double segment_longitudinal_m =
        samples[index].longitudinal_offset_m -
        samples[index - 1U].longitudinal_offset_m;
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0 ||
        !std::isfinite(segment_longitudinal_m) ||
        segment_longitudinal_m < 0.0) {
      result.reason = "invalid_resampled_time_or_longitudinal_axis";
      return result;
    }
    // predicted_speed_mpsはCandidateBuilderのs(t)速度。横移動を含む
    // Cartesian arc速度ではないため、time軸の改ざん検出は同じ縦軸ds/dtへ
    // 束縛し、実Cartesian arcは曲率・操舵・PP spatial horizonへ使う。
    const double execution_speed_mps = segment_longitudinal_m / dt_sec;
    const double predicted_speed_mps =
        0.5 * (samples[index - 1U].predicted_speed_mps +
               samples[index].predicted_speed_mps);
    const double speed_error_mps =
        std::abs(execution_speed_mps - predicted_speed_mps);
    const double allowed_error_mps =
        config.speed_consistency_abs_tolerance_mps +
        config.speed_consistency_relative_tolerance *
            std::max(execution_speed_mps, predicted_speed_mps);
    if (!std::isfinite(execution_speed_mps) ||
        !std::isfinite(predicted_speed_mps) ||
        !std::isfinite(speed_error_mps) || !std::isfinite(allowed_error_mps)) {
      result.reason = "invalid_execution_speed_consistency";
      return result;
    }
    result.max_abs_speed_consistency_error_mps =
        std::max(result.max_abs_speed_consistency_error_mps, speed_error_mps);
    if (speed_error_mps > allowed_error_mps) {
      result.reason = "execution_speed_inconsistent_with_prediction";
      return result;
    }
  }

  std::array<double, kMaxPointCount> steering_angles_rad{};
  result.max_abs_curvature_m_inv = 0.0;
  result.max_abs_steering_angle_rad = 0.0;
  for (std::size_t index = 0U; index < sample_count; ++index) {
    const std::size_t middle =
        std::clamp(index, std::size_t{1U}, sample_count - 2U);
    bool curvature_valid = false;
    const double curvature_m_inv =
        signedCurvature(samples[middle - 1U], samples[middle],
                        samples[middle + 1U], &curvature_valid);
    if (!curvature_valid) {
      result.reason = "invalid_cartesian_curvature";
      return result;
    }
    const double steering_angle_rad =
        config.steering_gain * std::atan(config.wheelbase_m * curvature_m_inv);
    if (!std::isfinite(steering_angle_rad)) {
      result.reason = "invalid_cartesian_steering";
      return result;
    }
    result.max_abs_curvature_m_inv =
        std::max(result.max_abs_curvature_m_inv, std::abs(curvature_m_inv));
    result.max_abs_steering_angle_rad = std::max(
        result.max_abs_steering_angle_rad, std::abs(steering_angle_rad));
    steering_angles_rad[index] = steering_angle_rad;
  }

  result.max_abs_steering_rate_radps = 0.0;
  for (std::size_t index = 1U; index < sample_count; ++index) {
    const double dt_sec =
        samples[index].time_sec - samples[index - 1U].time_sec;
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
      result.reason = "invalid_resampled_time";
      return result;
    }
    const double steering_rate_radps =
        std::abs(steering_angles_rad[index] - steering_angles_rad[index - 1U]) /
        dt_sec;
    if (!std::isfinite(steering_rate_radps)) {
      result.reason = "invalid_cartesian_steering_rate";
      return result;
    }
    result.max_abs_steering_rate_radps =
        std::max(result.max_abs_steering_rate_radps, steering_rate_radps);
  }

  const double first_yaw = sampleYaw(samples, sample_count, 0U);
  const FrenetPose first_frenet =
      frame_.cartesianToFrenet(samples[0U].x_m, samples[0U].y_m, first_yaw);
  if (!std::isfinite(first_frenet.s) || !std::isfinite(first_frenet.d)) {
    result.reason = "invalid_frenet_projection";
    return result;
  }
  double previous_s_m = first_frenet.s;
  double previous_target_delta_m = first_frenet.d - config.target_d_m;
  const int initial_target_side =
      previous_target_delta_m > config.target_d_tolerance_m    ? 1
      : previous_target_delta_m < -config.target_d_tolerance_m ? -1
                                                               : 0;
  bool target_reached =
      std::abs(previous_target_delta_m) <= config.target_d_tolerance_m;
  if (target_reached) {
    result.target_d_reach_arc_m = 0.0;
  }
  for (std::size_t index = 0U; index < sample_count; ++index) {
    const double yaw_rad = sampleYaw(samples, sample_count, index);
    const FrenetPose projected = frame_.cartesianToFrenet(
        samples[index].x_m, samples[index].y_m, yaw_rad);
    if (!std::isfinite(projected.s) || !std::isfinite(projected.d)) {
      result.reason = "invalid_frenet_projection";
      return result;
    }
    if (index > 0U) {
      double delta_s_m = projected.s - previous_s_m;
      if (delta_s_m < -0.5 * frame_.length()) {
        delta_s_m += frame_.length();
      } else if (delta_s_m > 0.5 * frame_.length()) {
        delta_s_m -= frame_.length();
      }
      if (!std::isfinite(delta_s_m) || delta_s_m <= 0.0) {
        result.reason = "nonmonotonic_frenet_projection";
        return result;
      }
      previous_s_m = projected.s;
    }
    const double target_delta_m = projected.d - config.target_d_m;
    if (!target_reached && index > 0U &&
        (std::abs(target_delta_m) <= config.target_d_tolerance_m ||
         (previous_target_delta_m < 0.0 && target_delta_m > 0.0) ||
         (previous_target_delta_m > 0.0 && target_delta_m < 0.0))) {
      const double entry_boundary_delta_m =
          previous_target_delta_m > config.target_d_tolerance_m
              ? config.target_d_tolerance_m
              : -config.target_d_tolerance_m;
      const double delta_change_m = target_delta_m - previous_target_delta_m;
      if (!std::isfinite(delta_change_m) ||
          std::abs(delta_change_m) <= std::numeric_limits<double>::epsilon()) {
        result.target_d_reach_arc_m = samples[index].arc_m;
      } else {
        const double crossing_ratio = std::clamp(
            (entry_boundary_delta_m - previous_target_delta_m) / delta_change_m,
            0.0, 1.0);
        result.target_d_reach_arc_m =
            samples[index - 1U].arc_m +
            crossing_ratio * (samples[index].arc_m - samples[index - 1U].arc_m);
      }
      target_reached = true;
    }
    if (target_reached &&
        ((initial_target_side == 0 &&
          std::abs(target_delta_m) > config.target_d_tolerance_m) ||
         (initial_target_side != 0 &&
          static_cast<double>(initial_target_side) * target_delta_m >
              config.target_d_tolerance_m))) {
      result.reason = "target_d_cross_back";
      return result;
    }
    result.terminal_projected_d_m = projected.d;
    previous_target_delta_m = target_delta_m;
  }

  result.valid = true;
  if (result.max_abs_steering_angle_rad > config.max_steering_angle_rad) {
    result.desired_path_reason = "steering_angle_limit_exceeded";
    result.reason = "steering_angle_limit_exceeded";
    return result;
  }
  if (result.max_abs_steering_rate_radps > config.max_steering_rate_radps) {
    result.desired_path_reason = "steering_rate_limit_exceeded";
    result.reason = "steering_rate_limit_exceeded";
    return result;
  }
  if (total_arc_m < config.pure_pursuit_required_arc_m) {
    result.desired_path_reason = "insufficient_pure_pursuit_arc";
    result.reason = "insufficient_pure_pursuit_arc";
    return result;
  }
  if (!target_reached) {
    result.desired_path_reason = "target_d_not_reached";
    result.reason = "target_d_not_reached";
    return result;
  }
  if (result.target_d_reach_arc_m > config.target_d_deadline_arc_m) {
    result.desired_path_reason = "target_d_deadline_exceeded";
    result.reason = "target_d_deadline_exceeded";
    return result;
  }
  if (!std::isfinite(result.terminal_projected_d_m) ||
      std::abs(result.terminal_projected_d_m - config.target_d_m) >
          config.target_d_tolerance_m) {
    result.desired_path_reason = "target_d_wrong_terminal";
    result.reason = "target_d_wrong_terminal";
    return result;
  }

  result.desired_path_trackable = true;
  result.desired_path_reason = "ok";
  const bool pp_input_valid =
      input.ego.valid && std::isfinite(input.ego.x) &&
      std::isfinite(input.ego.y) && std::isfinite(input.ego.yaw) &&
      std::isfinite(input.ego.v) && input.ego.v >= 0.0 &&
      input.active_lookahead_valid &&
      std::isfinite(input.active_lookahead_distance_m) &&
      input.active_lookahead_distance_m > 0.0 &&
      input.nearest_source_index_valid &&
      input.nearest_source_index < source_count &&
      input.curvature_feedforward_valid &&
      std::isfinite(input.curvature_feedforward_steering_rad) &&
      input.steering_reference_valid &&
      std::isfinite(input.steering_reference_angle_rad) &&
      std::isfinite(input.steering_command_dt_sec) &&
      input.steering_command_dt_sec > 0.0;
  if (!pp_input_valid) {
    result.pure_pursuit_command_reason =
        "pure_pursuit_command_snapshot_unavailable";
    result.reason = result.pure_pursuit_command_reason;
    return result;
  }

  const double rear_x_m =
      input.ego.x - 0.5 * config.wheelbase_m * std::cos(input.ego.yaw);
  const double rear_y_m =
      input.ego.y - 0.5 * config.wheelbase_m * std::sin(input.ego.yaw);
  const double endpoint_chord_m =
      std::hypot(candidate.x.back() - rear_x_m, candidate.y.back() - rear_y_m);
  if (!std::isfinite(endpoint_chord_m) || endpoint_chord_m <= 1.0e-3) {
    result.pure_pursuit_command_reason = "invalid_pure_pursuit_endpoint_chord";
    result.reason = result.pure_pursuit_command_reason;
    return result;
  }
  result.pure_pursuit_lookahead_distance_m =
      std::min(input.active_lookahead_distance_m, endpoint_chord_m);

  std::size_t target_index = source_count;
  for (std::size_t index = input.nearest_source_index; index < source_count;
       ++index) {
    const double chord_m = std::hypot(candidate.x[index] - rear_x_m,
                                      candidate.y[index] - rear_y_m);
    if (chord_m >= result.pure_pursuit_lookahead_distance_m) {
      target_index = index;
      break;
    }
  }
  if (target_index == source_count) {
    result.pure_pursuit_command_reason =
        "pure_pursuit_lookahead_endpoint_fallback";
    result.reason = result.pure_pursuit_command_reason;
    return result;
  }
  result.pure_pursuit_target_source_index = target_index;
  const double alpha_rad =
      std::atan2(std::sin(std::atan2(candidate.y[target_index] - rear_y_m,
                                     candidate.x[target_index] - rear_x_m) -
                          input.ego.yaw),
                 std::cos(std::atan2(candidate.y[target_index] - rear_y_m,
                                     candidate.x[target_index] - rear_x_m) -
                          input.ego.yaw));
  result.pure_pursuit_geometric_steering_angle_rad =
      std::atan2(2.0 * config.wheelbase_m * std::sin(alpha_rad),
                 result.pure_pursuit_lookahead_distance_m);
  result.curvature_feedforward_steering_angle_rad =
      input.curvature_feedforward_steering_rad;
  result.pure_pursuit_raw_steering_angle_rad =
      result.pure_pursuit_geometric_steering_angle_rad +
      result.curvature_feedforward_steering_angle_rad;
  result.pure_pursuit_requested_steering_angle_rad =
      config.steering_gain * result.pure_pursuit_raw_steering_angle_rad;
  const double bounded_reference_rad =
      std::clamp(input.steering_reference_angle_rad,
                 -config.max_steering_angle_rad, config.max_steering_angle_rad);
  const double angle_clamped_request =
      std::clamp(result.pure_pursuit_requested_steering_angle_rad,
                 -config.max_steering_angle_rad, config.max_steering_angle_rad);
  const double max_delta_rad =
      config.max_steering_rate_radps * input.steering_command_dt_sec;
  if (!std::isfinite(max_delta_rad)) {
    result.pure_pursuit_command_reason = "invalid_pure_pursuit_rate_allowance";
    result.reason = result.pure_pursuit_command_reason;
    return result;
  }
  const double rate_clamped_request =
      std::clamp(angle_clamped_request, bounded_reference_rad - max_delta_rad,
                 bounded_reference_rad + max_delta_rad);
  result.pure_pursuit_bounded_steering_angle_rad =
      std::clamp(rate_clamped_request, -config.max_steering_angle_rad,
                 config.max_steering_angle_rad);
  result.pure_pursuit_requested_steering_rate_radps =
      (result.pure_pursuit_requested_steering_angle_rad -
       bounded_reference_rad) /
      input.steering_command_dt_sec;
  result.pure_pursuit_bounded_steering_rate_radps =
      (result.pure_pursuit_bounded_steering_angle_rad - bounded_reference_rad) /
      input.steering_command_dt_sec;
  result.pure_pursuit_command_evaluated = true;
  const bool angle_limited =
      std::abs(result.pure_pursuit_requested_steering_angle_rad -
               angle_clamped_request) > 1.0e-12;
  const bool rate_limited =
      std::abs(angle_clamped_request -
               result.pure_pursuit_bounded_steering_angle_rad) > 1.0e-12;
  result.pure_pursuit_angle_limited = angle_limited;
  result.pure_pursuit_rate_limited = rate_limited;
  if (angle_limited) {
    result.pure_pursuit_command_reason = "pure_pursuit_command_angle_limited";
    result.reason = result.pure_pursuit_command_reason;
    return result;
  }
  if (rate_limited) {
    result.pure_pursuit_command_reason = "pure_pursuit_command_rate_limited";
    result.reason = result.pure_pursuit_command_reason;
    return result;
  }

  result.pure_pursuit_command_trackable = true;
  result.pure_pursuit_command_reason = "ok";
  result.trackable =
      result.desired_path_trackable && result.pure_pursuit_command_trackable;
  result.reason = result.trackable ? "ok" : "not_trackable";
  return result;
}

} // namespace overtake_planner
