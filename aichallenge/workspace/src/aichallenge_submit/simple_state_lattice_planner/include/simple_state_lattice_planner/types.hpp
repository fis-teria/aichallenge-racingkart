#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace simple_state_lattice_planner {

enum class PlannerState { FREE_RUN, OVERTAKE };

enum class CandidateSide : int { CENTER = 0, LEFT = 1, RIGHT = -1 };

enum class AdmissionFailure {
  NONE,
  INVALID_FRAME,
  INVALID_TIMESTAMP,
  STALE_INPUT,
  FUTURE_INPUT,
  NON_FINITE_INPUT,
  REVERSE_SPEED,
  INVALID_REFERENCE,
  INVALID_CONFIG,
};

enum class RejectReason {
  NONE,
  TOO_FEW_POINTS,
  NON_FINITE_POINT,
  REVERSE_S,
  COSTMAP_OCCUPIED,
  CURVATURE_LIMIT,
  STEERING_ANGLE_LIMIT,
  STEERING_RATE_LIMIT,
  NEGATIVE_SPEED,
  SNAPSHOT_MISMATCH,
  INVALID_COSTMAP,
  INVALID_EVALUATOR_CONFIG,
  COST_OVERFLOW,
};

struct ReferencePoint {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double s_m{0.0};
};

struct ReferenceWindow {
  std::uint64_t snapshot_id{0U};
  std::string frame_id{"map"};
  double stamp_sec{0.0};
  std::vector<ReferencePoint> points;
};

struct EgoState {
  std::uint64_t snapshot_id{0U};
  std::string frame_id{"map"};
  double stamp_sec{0.0};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double speed_mps{0.0};
};

struct TrajectoryPoint {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double curvature_radpm{0.0};
  double s_m{0.0};
  double d_m{0.0};
  double speed_mps{0.0};
};

struct Candidate {
  std::uint64_t snapshot_id{0U};
  std::uint32_t id{0U};
  CandidateSide side{CandidateSide::CENTER};
  double transition_distance_m{0.0};
  double remaining_transition_distance_m{0.0};
  std::vector<TrajectoryPoint> points;
  bool valid{false};
  RejectReason reject_reason{RejectReason::NONE};
  double total_cost{0.0};
};

struct LatticeConfig {
  std::string frame_id{"map"};
  double horizon_m{20.0};
  double sample_spacing_m{0.05};
  double pass_offset_m{1.8};
  std::array<double, 3U> transition_distances_m{{6.0, 10.0, 14.0}};
  double target_speed_mps{2.0};
  double maximum_input_age_sec{0.20};
  double maximum_future_offset_sec{0.05};
  double maximum_input_skew_sec{0.05};
  std::size_t maximum_sample_count{4096U};
};

struct LatticeResult {
  std::uint64_t snapshot_id{0U};
  AdmissionFailure failure{AdmissionFailure::NONE};
  std::string reason{"valid"};
  std::vector<Candidate> candidates;

  bool valid() const {
    return failure == AdmissionFailure::NONE && !candidates.empty();
  }
};

} // namespace simple_state_lattice_planner
