#include "state_lattice_overtake_planner/cost_model.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace state_lattice_overtake_planner {
namespace {
constexpr double kPi = 3.14159265358979323846;

Pose2d transformPoint(const Pose2d &origin, double longitudinal,
                      double lateral) {
  return {origin.x + std::cos(origin.yaw) * longitudinal -
              std::sin(origin.yaw) * lateral,
          origin.y + std::sin(origin.yaw) * longitudinal +
              std::cos(origin.yaw) * lateral,
          origin.yaw};
}

double cross(const Pose2d &a, const Pose2d &b, const Pose2d &c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool segmentsIntersect(const Pose2d &a, const Pose2d &b, const Pose2d &c,
                       const Pose2d &d) {
  const double ab_c = cross(a, b, c);
  const double ab_d = cross(a, b, d);
  const double cd_a = cross(c, d, a);
  const double cd_b = cross(c, d, b);
  constexpr double epsilon = 1.0e-12;
  const auto on_segment = [](const Pose2d &p, const Pose2d &q,
                             const Pose2d &r) {
    constexpr double epsilon = 1.0e-12;
    return q.x >= std::min(p.x, r.x) - epsilon &&
           q.x <= std::max(p.x, r.x) + epsilon &&
           q.y >= std::min(p.y, r.y) - epsilon &&
           q.y <= std::max(p.y, r.y) + epsilon;
  };
  if (((ab_c > epsilon && ab_d < -epsilon) ||
       (ab_c < -epsilon && ab_d > epsilon)) &&
      ((cd_a > epsilon && cd_b < -epsilon) ||
       (cd_a < -epsilon && cd_b > epsilon))) {
    return true;
  }
  if (std::abs(ab_c) <= epsilon && on_segment(a, c, b)) {
    return true;
  }
  if (std::abs(ab_d) <= epsilon && on_segment(a, d, b)) {
    return true;
  }
  if (std::abs(cd_a) <= epsilon && on_segment(c, a, d)) {
    return true;
  }
  if (std::abs(cd_b) <= epsilon && on_segment(c, b, d)) {
    return true;
  }
  return false;
}

double rectangleDistance(const std::array<Pose2d, 4> &lhs,
                         const std::array<Pose2d, 4> &rhs) {
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    const std::size_t inext = (i + 1U) % lhs.size();
    for (std::size_t j = 0; j < rhs.size(); ++j) {
      const std::size_t jnext = (j + 1U) % rhs.size();
      if (segmentsIntersect(lhs[i], lhs[inext], rhs[j], rhs[jnext])) {
        return 0.0;
      }
      distance = std::min(distance, pointToSegmentDistance(
                                        lhs[i].x, lhs[i].y, rhs[j].x, rhs[j].y,
                                        rhs[jnext].x, rhs[jnext].y));
      distance = std::min(distance, pointToSegmentDistance(
                                        rhs[j].x, rhs[j].y, lhs[i].x, lhs[i].y,
                                        lhs[inext].x, lhs[inext].y));
    }
  }
  return distance;
}
} // namespace

int wallCostLevel(double distance_m, const PlannerConfig &c) {
  if (!std::isfinite(distance_m) ||
      distance_m < c.wall_distance_thresholds_m[0]) {
    return 9;
  }
  if (distance_m < c.wall_distance_thresholds_m[1]) {
    return 7;
  }
  if (distance_m < c.wall_distance_thresholds_m[2]) {
    return 6;
  }
  return 0;
}

int objectCostLevel(double distance_m, const PlannerConfig &c) {
  if (!std::isfinite(distance_m) ||
      distance_m < c.object_distance_thresholds_m[0]) {
    return 9;
  }
  if (distance_m < c.object_distance_thresholds_m[1]) {
    return 8;
  }
  if (distance_m < c.object_distance_thresholds_m[2]) {
    return 7;
  }
  if (distance_m < c.object_distance_thresholds_m[3]) {
    return 6;
  }
  if (distance_m < c.object_distance_thresholds_m[4]) {
    return 5;
  }
  return 0;
}

int referenceCostLevel(double distance_m, const PlannerConfig &c) {
  if (!std::isfinite(distance_m)) {
    return 9;
  }
  const double d = std::abs(distance_m);
  if (d < c.reference_distance_thresholds_m[0]) {
    return 0;
  }
  if (d < c.reference_distance_thresholds_m[1]) {
    return 1;
  }
  if (d < c.reference_distance_thresholds_m[2]) {
    return 2;
  }
  if (d < c.reference_distance_thresholds_m[3]) {
    return 3;
  }
  if (d < 1.7) {
    return 4;
  }
  return std::min(9, 5 + static_cast<int>(std::floor(
                             (d - 1.7 + 1.0e-9) / c.reference_extra_step_m)));
}

int mergeCostLevels(int lhs, int rhs) {
  lhs = std::clamp(lhs, 0, 9);
  rhs = std::clamp(rhs, 0, 9);
  if (lhs >= 7 && rhs >= 7) {
    return 9;
  }
  if (lhs >= 5 && rhs >= 5) {
    return std::min(9, std::max(lhs, rhs) + 1);
  }
  return std::max(lhs, rhs);
}

int trajectoryCost(const std::vector<int> &pose_costs) {
  return std::accumulate(pose_costs.begin(), pose_costs.end(), 0);
}

double forwardTerminalDistance(double speed_mps, const PlannerConfig &c) {
  if (!std::isfinite(speed_mps)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double ratio =
      std::clamp(std::max(0.0, speed_mps) / c.max_distance_speed_mps, 0.0, 1.0);
  return c.base_distance_m + (c.max_distance_m - c.base_distance_m) * ratio;
}

double targetSpeedForCost(int cost, const PlannerConfig &c) {
  if (cost <= c.free_run_return_cost) {
    return c.normal_speed_mps;
  }
  if (cost >= c.stop_cost) {
    return 0.0;
  }
  return c.normal_speed_mps * static_cast<double>(c.stop_cost - cost) /
         static_cast<double>(c.stop_cost - c.free_run_return_cost);
}

double curvatureForSteering(double steering_rad, double wheel_base_m) {
  return std::tan(steering_rad) / wheel_base_m;
}

double curveSpeedLimit(double curvature, const PlannerConfig &c) {
  if (!std::isfinite(curvature)) {
    return 0.0;
  }
  return std::min(c.normal_speed_mps,
                  std::sqrt(c.lateral_acceleration_limit_mps2 /
                            std::max(std::abs(curvature), 1.0e-6)));
}

double jerkLimitedAcceleration(double target_speed_mps,
                               double previous_speed_mps,
                               double previous_acceleration_mps2, double dt_sec,
                               bool emergency, const PlannerConfig &c,
                               bool *jerk_exceeded) {
  double target = std::clamp((target_speed_mps - previous_speed_mps) / dt_sec,
                             c.min_acceleration_mps2, c.max_acceleration_mps2);
  const double low =
      previous_acceleration_mps2 - c.max_deceleration_jerk_mps3 * dt_sec;
  const double high =
      previous_acceleration_mps2 + c.max_acceleration_jerk_mps3 * dt_sec;
  const bool exceeds = target < low || target > high;
  if (jerk_exceeded != nullptr) {
    *jerk_exceeded = emergency && exceeds;
  }
  if (!emergency) {
    target = std::clamp(target, low, high);
  }
  return std::clamp(target, c.min_acceleration_mps2, c.max_acceleration_mps2);
}

UncertaintyMargin uncertaintyMargin(double sigma_x_m, double sigma_y_m,
                                    const PlannerConfig &c) {
  UncertaintyMargin out;
  if (!std::isfinite(sigma_x_m) || !std::isfinite(sigma_y_m) ||
      sigma_x_m < 0.0 || sigma_y_m < 0.0) {
    out.reason = "invalid_sigma";
    return out;
  }
  out.x_m = std::max(c.sigma_min_margin_m, c.sigma_multiplier * sigma_x_m);
  out.y_m = std::max(c.sigma_min_margin_m, c.sigma_multiplier * sigma_y_m);
  if (out.x_m > c.sigma_max_margin_m || out.y_m > c.sigma_max_margin_m) {
    out.reason = "sigma_margin_exceeds_limit";
    return out;
  }
  out.valid = true;
  return out;
}

Footprint nominalFootprint(const PlannerConfig &c) {
  return {c.wheel_base_m + c.front_overhang_m, c.rear_overhang_m,
          c.left_extent_m, c.right_extent_m};
}

Footprint wallFootprint(const PlannerConfig &c, bool tracking) {
  const double longitudinal =
      c.wall_hard_margin_m +
      (tracking ? c.longitudinal_tracking_margin_m : 0.0);
  const double lateral =
      c.wall_hard_margin_m + (tracking ? c.lateral_tracking_margin_m : 0.0);
  const auto base = nominalFootprint(c);
  return {base.front_m + longitudinal, base.rear_m + longitudinal,
          base.left_m + lateral, base.right_m + lateral};
}

std::array<Pose2d, 4> footprintCorners(const Pose2d &pose,
                                       const Footprint &fp) {
  return {transformPoint(pose, fp.front_m, fp.left_m),
          transformPoint(pose, fp.front_m, -fp.right_m),
          transformPoint(pose, -fp.rear_m, -fp.right_m),
          transformPoint(pose, -fp.rear_m, fp.left_m)};
}

bool rectanglesWithinClearance(const Pose2d &a, const Footprint &af,
                               const Pose2d &b, const Footprint &bf,
                               double clearance_m) {
  return rectangleClearance(a, af, b, bf) < clearance_m;
}

double rectangleClearance(const Pose2d &a, const Footprint &af, const Pose2d &b,
                          const Footprint &bf) {
  return rectangleDistance(footprintCorners(a, af), footprintCorners(b, bf));
}

double pointToSegmentDistance(double px, double py, double ax, double ay,
                              double bx, double by) {
  const double dx = bx - ax;
  const double dy = by - ay;
  const double denom = dx * dx + dy * dy;
  const double t =
      denom > 1.0e-12
          ? std::clamp(((px - ax) * dx + (py - ay) * dy) / denom, 0.0, 1.0)
          : 0.0;
  return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

const char *toString(BehaviorMode mode) {
  static constexpr std::array<const char *, 12> names{"FREE_RUN",
                                                      "FOLLOW_BLOCKED",
                                                      "PREPARE_OVERTAKE_LEFT",
                                                      "PREPARE_OVERTAKE_RIGHT",
                                                      "OVERTAKE_LEFT",
                                                      "OVERTAKE_RIGHT",
                                                      "MERGE_BACK",
                                                      "ABORT_RECOVERY",
                                                      "SIDE_BY_SIDE_KEEP",
                                                      "YIELD_BEHIND",
                                                      "SAFE_STOP",
                                                      "SPEED_GUARD"};
  const int index = static_cast<int>(mode);
  return index >= 0 && index < static_cast<int>(names.size())
             ? names[static_cast<std::size_t>(index)]
             : "UNKNOWN";
}

const char *toString(CollisionKind kind) {
  static constexpr std::array<const char *, 3> names{"none", "wall",
                                                     "opponent"};
  const int index = static_cast<int>(kind);
  return index >= 0 && index < static_cast<int>(names.size())
             ? names[static_cast<std::size_t>(index)]
             : "unknown";
}

const char *toString(CurrentPoseOpponentRelation relation) {
  static constexpr std::array<const char *, 4> names{
      "unknown", "front_or_overlap", "side_overlap", "rear_only"};
  const int index = static_cast<int>(relation);
  return index >= 0 && index < static_cast<int>(names.size())
             ? names[static_cast<std::size_t>(index)]
             : "unknown";
}

const char *toString(OutputHorizonFailure failure) {
  static constexpr std::array<const char *, 13> names{
      "none",
      "invalid_input",
      "invalid_contract",
      "non_positive_dt",
      "steering_rate",
      "invalid_point",
      "curvature",
      "waypoint_wall_collision",
      "waypoint_opponent_collision",
      "interpolated_wall_collision",
      "interpolated_opponent_collision",
      "waypoint_side_role_separation",
      "interpolated_side_role_separation",
  };
  const int index = static_cast<int>(failure);
  return index >= 0 && index < static_cast<int>(names.size())
             ? names[static_cast<std::size_t>(index)]
             : "unknown";
}

const char *toString(FrontTargetClass target_class) {
  switch (target_class) {
  case FrontTargetClass::NONE:
    return "none";
  case FrontTargetClass::FORWARD:
    return "forward";
  case FrontTargetClass::PARALLEL:
    return "parallel";
  }
  return "unknown";
}

const char *toString(FrontDetectionFailure failure) {
  switch (failure) {
  case FrontDetectionFailure::NONE:
    return "none";
  case FrontDetectionFailure::INVALID_OPPONENT:
    return "invalid_opponent";
  case FrontDetectionFailure::LONGITUDINAL_TARGET:
    return "longitudinal_target";
  case FrontDetectionFailure::TRANSITION_ENVELOPE:
    return "transition_envelope";
  case FrontDetectionFailure::SELECTION_ARBITRATION:
    return "selection_arbitration";
  case FrontDetectionFailure::ENTER_HYSTERESIS:
    return "enter_hysteresis";
  }
  return "unknown";
}

const char *toString(EarlyAwareState state) {
  switch (state) {
  case EarlyAwareState::INACTIVE:
    return "inactive";
  case EarlyAwareState::CANDIDATE:
    return "candidate";
  case EarlyAwareState::ACTIVE:
    return "active";
  }
  return "unknown";
}

const char *toString(PreventiveSideRoleEligibilityFailure failure) {
  switch (failure) {
  case PreventiveSideRoleEligibilityFailure::NONE:
    return "none";
  case PreventiveSideRoleEligibilityFailure::INVALID_EGO_OBSERVATION:
    return "invalid_ego_observation";
  case PreventiveSideRoleEligibilityFailure::INVALID_PEER_OBSERVATION:
    return "invalid_peer_observation";
  case PreventiveSideRoleEligibilityFailure::DUPLICATE_PEER_ID:
    return "duplicate_peer_id";
  case PreventiveSideRoleEligibilityFailure::INVALID_VEHICLE_ID:
    return "invalid_vehicle_id";
  case PreventiveSideRoleEligibilityFailure::DELTA_S:
    return "delta_s";
  case PreventiveSideRoleEligibilityFailure::EGO_TANGENT_PROGRESS:
    return "ego_tangent_progress";
  case PreventiveSideRoleEligibilityFailure::PEER_TANGENT_PROGRESS:
    return "peer_tangent_progress";
  case PreventiveSideRoleEligibilityFailure::EGO_TRACK_HEADING:
    return "ego_track_heading";
  case PreventiveSideRoleEligibilityFailure::PEER_TRACK_HEADING:
    return "peer_track_heading";
  case PreventiveSideRoleEligibilityFailure::RELATIVE_HEADING:
    return "relative_heading";
  case PreventiveSideRoleEligibilityFailure::MUTUAL_SIDE_GEOMETRY:
    return "mutual_side_geometry";
  case PreventiveSideRoleEligibilityFailure::LATERAL_SEPARATION:
    return "lateral_separation";
  case PreventiveSideRoleEligibilityFailure::HARD_CLEARANCE:
    return "hard_clearance";
  case PreventiveSideRoleEligibilityFailure::MULTIPLE_PEERS:
    return "multiple_peers";
  case PreventiveSideRoleEligibilityFailure::PEER_DROPOUT:
    return "peer_dropout";
  case PreventiveSideRoleEligibilityFailure::PEER_CHANGED:
    return "peer_changed";
  case PreventiveSideRoleEligibilityFailure::ROLE_CHANGED:
    return "role_changed";
  case PreventiveSideRoleEligibilityFailure::RELEASE_UNVERIFIED:
    return "release_unverified";
  case PreventiveSideRoleEligibilityFailure::ENTRY_ENVELOPE:
    return "entry_envelope";
  case PreventiveSideRoleEligibilityFailure::PREDICTED_HARD_CLEARANCE:
    return "predicted_hard_clearance";
  case PreventiveSideRoleEligibilityFailure::SYNCHRONIZATION:
    return "synchronization";
  case PreventiveSideRoleEligibilityFailure::DECISION_DEADLINE:
    return "decision_deadline";
  case PreventiveSideRoleEligibilityFailure::POSITION_JUMP:
    return "position_jump";
  }
  return "unknown";
}

const char *toString(PreventiveSideRoleCandidateFailure failure) {
  switch (failure) {
  case PreventiveSideRoleCandidateFailure::NONE:
    return "none";
  case PreventiveSideRoleCandidateFailure::NO_GENERATED_CANDIDATE:
    return "no_generated_candidate";
  case PreventiveSideRoleCandidateFailure::NO_BASE_FEASIBLE_CANDIDATE:
    return "no_base_feasible_candidate";
  case PreventiveSideRoleCandidateFailure::DENSE_PEER_SEPARATION:
    return "dense_peer_separation";
  case PreventiveSideRoleCandidateFailure::OUTPUT_HORIZON:
    return "output_horizon";
  case PreventiveSideRoleCandidateFailure::RESAMPLED_PEER_SEPARATION:
    return "resampled_peer_separation";
  case PreventiveSideRoleCandidateFailure::DIRECTION:
    return "direction";
  case PreventiveSideRoleCandidateFailure::DEADLINE:
    return "deadline";
  case PreventiveSideRoleCandidateFailure::PROFILE_LIMIT:
    return "profile_limit";
  }
  return "unknown";
}

const char *toString(PreventiveSideRolePhase phase) {
  switch (phase) {
  case PreventiveSideRolePhase::NONE:
    return "none";
  case PreventiveSideRolePhase::FOLLOWER_YIELD:
    return "follower_yield";
  case PreventiveSideRolePhase::LEADER_WAIT:
    return "leader_wait";
  case PreventiveSideRolePhase::LEADER_ESCAPE:
    return "leader_escape";
  case PreventiveSideRolePhase::NEUTRAL_HOLD:
    return "neutral_hold";
  case PreventiveSideRolePhase::LEADER_PROCEED:
    return "leader_proceed";
  }
  return "unknown";
}

const char *toString(PreventiveSideRoleAction action) {
  switch (action) {
  case PreventiveSideRoleAction::NONE:
    return "none";
  case PreventiveSideRoleAction::NEUTRAL_STRAIGHT_HOLD:
    return "neutral_straight_hold";
  case PreventiveSideRoleAction::STRAIGHT_CURRENT_CORRIDOR:
    return "straight_current_corridor";
  case PreventiveSideRoleAction::LEFT_ESCAPE:
    return "left_escape";
  case PreventiveSideRoleAction::RIGHT_ESCAPE:
    return "right_escape";
  case PreventiveSideRoleAction::STRAIGHT_YIELD:
    return "straight_yield";
  case PreventiveSideRoleAction::FOLLOW_BEHIND:
    return "follow_behind";
  case PreventiveSideRoleAction::SAFE_STOP:
    return "safe_stop";
  }
  return "unknown";
}

double normalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

} // namespace state_lattice_overtake_planner
