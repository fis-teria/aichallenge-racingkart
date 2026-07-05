#pragma once

#include <limits>
#include <string>
#include <vector>

namespace overtake_planner {

enum class BehaviorMode {
  // 状態機械の現在モード。MPC overrideを出すか、追従するか、復帰するかを表す。
  FREE_RUN = 0,
  FOLLOW_BLOCKED = 1,
  PREPARE_OVERTAKE_LEFT = 2,
  PREPARE_OVERTAKE_RIGHT = 3,
  OVERTAKE_LEFT = 4,
  OVERTAKE_RIGHT = 5,
  MERGE_BACK = 6,
  ABORT_RECOVERY = 7,
  SIDE_BY_SIDE_KEEP = 8,
  YIELD_BEHIND = 9,
  SAFE_STOP = 10,
  SPEED_GUARD = 11,
};

enum class CandidateType {
  // 各周期で評価する候補軌道。最終的に一番低スコアの候補が状態機械へ渡される。
  FASTEST = 0,
  FOLLOW = 1,
  PASS_LEFT = 2,
  PASS_RIGHT = 3,
  RECOVERY = 4,
  SIDE_BY_SIDE_KEEP = 5,
  YIELD_BEHIND = 6,
  SAFE_STOP = 7,
};

struct ReferencePoint {
  // 参照CSV上の中心線サンプル。Frenet座標系の基準になる。
  double s{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double kappa{0.0};
  double v_ref{0.0};
};

struct FrenetPose {
  // 参照線に対する縦方向sと横方向d。追い越し判断は主にこの座標系で行う。
  double s{0.0};
  double d{0.0};
  double yaw_error{0.0};
  std::size_t index{0};
};

struct EgoState {
  // 自車の現在状態。odomと参照線から周期ごとに作り直す。
  double stamp_sec{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
  FrenetPose frenet{};
  bool valid{false};
};

struct OpponentState {
  // V2Xから見える他車状態。速度は位置差分から推定する。
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

struct PredictedOpponent {
  // 安全評価用に、他車を短い時間 horizon で等速予測した軌道。
  std::string id{};
  std::vector<double> t;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> d;
};

struct CandidateTrajectory {
  // MPCへ渡す横オフセット列と速度上限列。安全評価とスコアもここへ保持する。
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
  double min_safety_margin{std::numeric_limits<double>::infinity()};
  double cbf_slack{0.0};
  int active_safety_constraint_count{0};
  std::string reject_reason{};
};

struct BlockedInfo {
  // 前方の遅い車両や横並び状態をまとめた、追い越し開始/継続判断の入力。
  bool blocked{false};
  bool side_by_side{false};
  bool corner_side_by_side{false};
  int nearest_index{-1};
  std::string nearest_id{};
  double front_delta_s{std::numeric_limits<double>::infinity()};
  double front_delta_d{0.0};
  double front_rel_v{0.0};
  int side_index{-1};
  std::string side_id{};
  double side_delta_s{std::numeric_limits<double>::infinity()};
  double side_delta_d{0.0};
  double side_rel_v{0.0};
  double front_s_dot_mps{0.0};
  double side_s_dot_mps{0.0};
  bool front_direction_known{false};
  bool side_direction_known{false};
  bool front_same_direction{true};
  bool side_same_direction{true};
  int ignored_opposite_direction_count{0};
  bool parallel_side_candidate{false};
  int parallel_side_index{-1};
  std::string parallel_side_id{};
  double parallel_side_delta_s{std::numeric_limits<double>::infinity()};
  double parallel_side_delta_d{0.0};
  double parallel_side_rel_v{0.0};
  double parallel_side_s_dot_mps{0.0};
  bool parallel_side_direction_known{false};
  bool parallel_side_same_direction{true};
  bool future_side_by_side{false};
  bool future_corner_side_by_side{false};
  bool future_outer_wall_risk{false};
  bool future_yield_required{false};
  double future_delta_s{std::numeric_limits<double>::infinity()};
  double future_delta_d{0.0};
  double future_wall_clearance_m{std::numeric_limits<double>::infinity()};
  double future_abs_curvature{0.0};
  double predicted_opponent_s{std::numeric_limits<double>::quiet_NaN()};
  double predicted_opponent_d{std::numeric_limits<double>::quiet_NaN()};
  double future_prediction_time_sec{0.0};
  std::string yield_reason{};
  double ego_wall_clearance_m{std::numeric_limits<double>::infinity()};
  double left_pass_gap_m{std::numeric_limits<double>::infinity()};
  double right_pass_gap_m{std::numeric_limits<double>::infinity()};
  bool can_pass_left{false};
  bool can_pass_right{false};
  double pass_gap_required_m{0.0};
  double corner_abs_curvature{0.0};
  bool straight_overtake_start_allowed{true};
  double overtake_start_abs_curvature{0.0};
  std::string overtake_start_gate_reason{};
  std::string pass_gap_reason{};
};

struct SafeStopContext {
  // SAFE_STOPの遷移/解除に必要な、通常候補選択だけでは表せない補助入力。
  bool requested{false};
  bool candidate_feasible{false};
  bool release_ready{false};
  double ego_speed_mps{std::numeric_limits<double>::infinity()};
  double lateral_error_m{std::numeric_limits<double>::infinity()};
  int trigger_count{0};
  std::string reason{};
};

struct MpcHealthStatus {
  bool valid{false};
  int infeasible_count{0};
  double solve_time_ms{std::numeric_limits<double>::quiet_NaN()};
  double age_sec{std::numeric_limits<double>::infinity()};
};

struct SectionSafetyRule {
  std::string name{};
  double s_start_m{0.0};
  double s_end_m{0.0};
  std::string profile{"default"};
  std::string role_policy{"default"};
};

struct ActiveSectionSafety {
  bool active{false};
  std::string name{};
  std::string profile{"default"};
  std::string role_policy{"default"};
  double wall_margin_scale{1.0};
  double speed_cap_scale{1.0};
  bool force_outer_yield{false};
};

struct PlannerConfig {
  // 追い越し候補生成、安全マージン、状態遷移をまとめて調整するパラメータ群。
  bool enabled{true};
  std::size_t horizon_points{20};
  double horizon_dt_sec{0.025};
  double pass_safe_required_cycles{5.0};
  double lookahead_s_m{10.0};
  double follow_trigger_s_m{12.0};
  double same_corridor_width_m{0.90};
  bool same_direction_filter_enabled{true};
  double same_direction_min_speed_mps{0.30};
  double same_direction_min_s_dot_mps{0.05};
  bool future_side_prediction_enabled{true};
  double future_side_prediction_horizon_sec{1.2};
  double future_side_prediction_dt_sec{0.3};
  double future_side_yield_wall_clearance_m{0.35};
  double dv_block_threshold_mps{0.20};
  double opponent_stale_time_sec{0.50};
  double side_by_side_s_m{4.0};
  double side_margin_m{1.2};
  bool parallel_side_detection_enabled{true};
  double parallel_side_s_m{12.0};
  double parallel_side_margin_m{4.0};
  double side_yield_s_m{0.30};
  double side_by_side_target_gap_m{0.75};
  double side_by_side_shift_distance_m{7.0};
  double side_by_side_speed_cap_mps{7.5};
  double corner_side_yield_curvature_m_inv{0.05};
  double corner_side_yield_lookahead_m{10.0};
  double corner_side_yield_wall_clearance_m{0.55};
  double corner_yield_target_d_m{0.0};
  double corner_yield_rejoin_gap_m{5.5};
  double yield_rejoin_wall_clearance_m{0.25};
  double corner_follow_speed_margin_mps{0.20};
  double corner_yield_v_max_mps{3.0};
  bool straight_only_overtake_enabled{true};
  double straight_overtake_max_curvature_m_inv{0.025};
  double straight_overtake_lookahead_m{12.0};
  double straight_overtake_release_hysteresis_m_inv{0.005};
  double large_lateral_error_threshold_m{0.60};
  double large_lateral_error_v_max_mps{2.5};
  double min_pass_gap_m{1.80};
  double pass_gap_hysteresis_m{0.15};
  double yield_speed_margin_mps{0.60};
  double yield_rejoin_gap_m{3.0};
  double left_offset_m{0.80};
  double right_offset_m{-0.80};
  double prepare_distance_m{8.0};
  double pass_distance_m{20.0};
  double merge_distance_m{12.0};
  double follow_speed_margin_mps{0.20};
  double max_overtake_v_bonus_mps{0.30};
  double recovery_v_max_mps{8.5};
  double wall_margin_recovery_v_max_mps{8.5};
  double v_passthrough_mps{50.0};
  double d_min_m{-1.35};
  double d_max_m{1.35};
  double min_wall_margin_m{0.50};
  double safety_ellipse_a_m{3.0};
  double safety_ellipse_b_m{1.8};
  double min_ellipse_h{0.20};
  double merge_front_gap_m{6.0};
  double abort_timeout_sec{5.0};
  double min_mode_hold_time_sec{0.60};
  double keep_mode_bonus{25.0};
  bool speed_only_fallback_enabled{true};
  double speed_only_fallback_v_max_mps{3.0};
  bool wall_risk_speed_guard_enabled{true};
  double wall_soft_margin_m{0.25};
  double wall_risk_v_max_mps{5.0};
  bool mpc_health_speed_guard_enabled{true};
  int mpc_health_infeasible_count_threshold{1};
  double mpc_health_solve_time_warn_ms{80.0};
  double mpc_health_v_max_mps{3.0};
  double mpc_health_stale_time_sec{0.60};
  bool section_safety_profile_enabled{true};
  std::vector<SectionSafetyRule> section_safety_rules;
  bool safe_stop_enabled{true};
  double safe_stop_v_mps{0.20};
  int safe_stop_trigger_cycles{1};
  int safe_stop_release_cycles{5};
  double safe_stop_release_front_gap_m{5.0};
  double safe_stop_release_wall_clearance_m{0.20};
  double safe_stop_lateral_error_threshold_m{0.40};
  double safe_stop_release_speed_mps{0.50};
};

struct PlannerOutput {
  // ROSノードへ返す最終結果。override配列、debug指標、選択理由を含める。
  BehaviorMode mode{BehaviorMode::FREE_RUN};
  CandidateType selected{CandidateType::FASTEST};
  std::vector<double> lateral_offsets;
  std::vector<double> speed_caps;
  BlockedInfo blocked_info{};
  std::string reason{};
  bool active_override{false};
  double target_lateral_offset_m{0.0};
  double min_cbf_h{std::numeric_limits<double>::quiet_NaN()};
  double cbf_slack{0.0};
  int active_cbf_constraint_count{0};
  bool safe_stop_triggered{false};
  bool safe_stop_release_ready{false};
  std::string safe_stop_reason{};
  std::string safe_stop_reject_reason{};
  double safe_stop_v_mps{0.0};
  int safe_stop_trigger_count{0};
  int safe_stop_hold_count{0};
  int safe_stop_release_count{0};
  bool speed_only_fallback_active{false};
  bool wall_risk_speed_guard_active{false};
  bool mpc_health_speed_guard_active{false};
  double applied_speed_cap_mps{std::numeric_limits<double>::quiet_NaN()};
  std::string speed_cap_reason{};
  double wall_soft_margin_m{std::numeric_limits<double>::quiet_NaN()};
  ActiveSectionSafety active_section{};
  MpcHealthStatus mpc_health{};
};

const char *toString(BehaviorMode mode);
const char *toString(CandidateType type);
bool isPassMode(BehaviorMode mode);

} // namespace overtake_planner
