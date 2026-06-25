#pragma once

#include <limits>
#include <string>
#include <vector>

namespace overtake_planner
{

enum class BehaviorMode {
  FREE_RUN = 0,
  FOLLOW_BLOCKED = 1,
  PREPARE_OVERTAKE_LEFT = 2,
  PREPARE_OVERTAKE_RIGHT = 3,
  OVERTAKE_LEFT = 4,
  OVERTAKE_RIGHT = 5,
  MERGE_BACK = 6,
  ABORT_RECOVERY = 7,
};

enum class CandidateType {
  FASTEST = 0,
  FOLLOW = 1,
  PASS_LEFT = 2,
  PASS_RIGHT = 3,
  RECOVERY = 4,
};

struct ReferencePoint
{
  double s{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double kappa{0.0};
  double v_ref{0.0};
};

struct FrenetPose
{
  double s{0.0};
  double d{0.0};
  double yaw_error{0.0};
  std::size_t index{0};
};

struct EgoState
{
  double stamp_sec{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
  FrenetPose frenet{};
  bool valid{false};
};

struct OpponentState
{
  std::string id{};
  double stamp_sec{0.0};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double v{0.0};
  FrenetPose frenet{};
  bool valid{false};
};

struct PredictedOpponent
{
  std::string id{};
  std::vector<double> t;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> d;
};

struct CandidateTrajectory
{
  CandidateType type{CandidateType::FASTEST};
  std::vector<double> t;
  std::vector<double> s;
  std::vector<double> d;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> v_ref;
  bool feasible{true};
  double score{0.0};
  std::string reject_reason{};
};

struct BlockedInfo
{
  bool blocked{false};
  bool side_by_side{false};
  int nearest_index{-1};
  double front_delta_s{std::numeric_limits<double>::infinity()};
  double front_delta_d{0.0};
  double front_rel_v{0.0};
};

struct PlannerConfig
{
  bool enabled{true};
  std::size_t horizon_points{30};
  double horizon_dt_sec{0.025};
  double pass_safe_required_cycles{5.0};
  double lookahead_s_m{35.0};
  double follow_trigger_s_m{12.0};
  double same_corridor_width_m{0.90};
  double dv_block_threshold_mps{0.20};
  double opponent_stale_time_sec{0.50};
  double side_by_side_s_m{4.0};
  double side_margin_m{1.2};
  double left_offset_m{0.80};
  double right_offset_m{-0.80};
  double prepare_distance_m{8.0};
  double pass_distance_m{20.0};
  double merge_distance_m{12.0};
  double follow_speed_margin_mps{0.20};
  double max_overtake_v_bonus_mps{0.30};
  double recovery_v_max_mps{5.0};
  double v_passthrough_mps{50.0};
  double d_min_m{-1.35};
  double d_max_m{1.35};
  double min_wall_margin_m{0.25};
  double safety_ellipse_a_m{3.0};
  double safety_ellipse_b_m{1.2};
  double min_ellipse_h{0.20};
  double merge_front_gap_m{6.0};
  double abort_timeout_sec{5.0};
  double min_mode_hold_time_sec{0.60};
  double keep_mode_bonus{25.0};
};

struct PlannerOutput
{
  BehaviorMode mode{BehaviorMode::FREE_RUN};
  CandidateType selected{CandidateType::FASTEST};
  std::vector<double> lateral_offsets;
  std::vector<double> speed_caps;
  BlockedInfo blocked_info{};
  std::string reason{};
  bool active_override{false};
};

const char * toString(BehaviorMode mode);
const char * toString(CandidateType type);
bool isPassMode(BehaviorMode mode);

}  // namespace overtake_planner
