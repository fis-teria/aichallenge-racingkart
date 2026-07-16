#pragma once

#include <cstdint>
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
  // 安全評価用の到達可能速度。v_refとは別に、応答遅れと制動上限を含む。
  std::vector<double> predicted_speed_mps;
  // 下流MPC/PPへの即時速度上限。各周期で先頭要素が直ちに消費される。
  std::vector<double> v_ref;
  bool feasible{true};
  // PASS目標へ到達するまでのs依存安全回廊を事前に通過できるか。
  // SafetyEvaluatorはこのfalseをwall/CBF評価より先にrejectする。
  bool pass_target_corridor_valid{true};
  // PASS候補で実際に計画した最終横目標と、raw profile全区間の最小壁余裕。
  // debugでは選択候補ではなく左右候補そのものを追えるようにする。
  double planned_target_d_m{std::numeric_limits<double>::quiet_NaN()};
  double corridor_min_margin_m{std::numeric_limits<double>::quiet_NaN()};
  double score{0.0};
  double min_safety_margin{std::numeric_limits<double>::infinity()};
  double cbf_slack{0.0};
  int active_safety_constraint_count{0};
  std::string reject_reason{};
  std::string blocking_opponent_id{};
  double blocking_time_sec{std::numeric_limits<double>::quiet_NaN()};
  // s(t) に使った減速到達可能性。v_refの即時capとは独立して安全判定する。
  bool longitudinal_profile_valid{true};
  double assumed_brake_decel_mps2{0.0};
  double response_delay_sec{0.0};
  double required_brake_distance_m{std::numeric_limits<double>::infinity()};
  double available_brake_distance_m{std::numeric_limits<double>::infinity()};
};

struct LocalizedLateralProfile {
  // PASS候補の横オフセットを、相手車両のs位置に紐づいた局所プロファイルとして固定する。
  bool active{false};
  CandidateType pass_type{CandidateType::FASTEST};
  std::string target_id{};
  double created_time_sec{std::numeric_limits<double>::quiet_NaN()};
  double anchor_s_m{std::numeric_limits<double>::quiet_NaN()};
  double target_s_m{std::numeric_limits<double>::quiet_NaN()};
  double avoid_start_s_m{std::numeric_limits<double>::quiet_NaN()};
  double full_offset_start_s_m{std::numeric_limits<double>::quiet_NaN()};
  double full_offset_end_s_m{std::numeric_limits<double>::quiet_NaN()};
  double merge_end_s_m{std::numeric_limits<double>::quiet_NaN()};
  double start_d_m{0.0};
  double target_d_m{0.0};
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
  bool parallel_follow_candidate{false};
  bool parallel_follow_feasible{false};
  bool parallel_follow_hold_lateral{false};
  int parallel_follow_index{-1};
  std::string parallel_follow_id{};
  double parallel_follow_delta_s{std::numeric_limits<double>::infinity()};
  double parallel_follow_delta_d{0.0};
  double parallel_follow_rel_v{0.0};
  double parallel_follow_s_dot_mps{0.0};
  bool parallel_follow_direction_known{false};
  bool parallel_follow_same_direction{true};
  bool slow_obstacle_chain_active{false};
  std::string slow_obstacle_chain_id{};
  double slow_obstacle_chain_delta_s{std::numeric_limits<double>::infinity()};
  double slow_obstacle_chain_delta_d{0.0};
  double slow_obstacle_chain_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  // 同一コリドーには未進入だが、停止した前方parallel車をPASS候補だけ先行評価する文脈。
  // blocked/FOLLOWへは昇格させず、PASSがGate 2を通らない限り通常走行を置き換えない。
  bool early_stationary_parallel_pass_target{false};
  std::string early_stationary_parallel_pass_id{};
  int early_stationary_parallel_pass_count{0};
  // parallel観測とは分離した、前方で停止またはほぼ停止している対象の判定。
  bool stationary_front_obstacle{false};
  std::string stationary_front_id{};
  double stationary_front_ttc_sec{std::numeric_limits<double>::infinity()};
  bool stationary_front_brake_feasible{false};
  double stationary_front_required_brake_distance_m{
      std::numeric_limits<double>::infinity()};
  double stationary_front_available_brake_distance_m{
      std::numeric_limits<double>::infinity()};
  bool future_side_by_side{false};
  bool future_corner_side_by_side{false};
  bool future_outer_wall_risk{false};
  bool future_yield_required{false};
  // strictなparallel接近で後方へ譲る間は、相手dへ横切らず現在dを保持する。
  bool parallel_yield_hold_lateral{false};
  bool future_parallel_interaction{false};
  double future_delta_s{std::numeric_limits<double>::infinity()};
  double future_delta_d{0.0};
  double future_wall_clearance_m{std::numeric_limits<double>::infinity()};
  double future_abs_curvature{0.0};
  double predicted_opponent_s{std::numeric_limits<double>::quiet_NaN()};
  double predicted_opponent_d{std::numeric_limits<double>::quiet_NaN()};
  double future_prediction_time_sec{0.0};
  std::string yield_reason{};
  double ego_lateral_offset_m{0.0};
  double ego_speed_mps{0.0};
  double ego_wall_clearance_m{std::numeric_limits<double>::infinity()};
  double left_pass_gap_m{std::numeric_limits<double>::infinity()};
  double right_pass_gap_m{std::numeric_limits<double>::infinity()};
  bool can_pass_left{false};
  bool can_pass_right{false};
  // can_pass_* は静的gap診断値。実際の開始可否は候補安全評価の結果で判断する。
  bool pass_left_candidate_generated{false};
  bool pass_right_candidate_generated{false};
  bool pass_left_candidate_feasible{false};
  bool pass_right_candidate_feasible{false};
  double pass_left_candidate_target_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_right_candidate_target_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_left_candidate_corridor_min_margin_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_right_candidate_corridor_min_margin_m{
      std::numeric_limits<double>::quiet_NaN()};
  std::string pass_left_candidate_reject_reason{};
  std::string pass_right_candidate_reject_reason{};
  // early stationary PASSがGate 2で不成立の間、中心線へ横断せず現在dを
  // SafetyEvaluator済みRECOVERY候補で保持する。reentry holdとは別文脈。
  bool early_stationary_parallel_pass_hold_lateral{false};
  // 停止障害物へ制動距離で接近したFOLLOWは、通常ラインへ横切らず現dを
  // SafetyEvaluatorへ渡す。成立しない時も同じ現d RECOVERYだけを再評価する。
  bool braking_follow_active{false};
  bool braking_follow_hold_lateral{false};
  bool braking_follow_feasible{false};
  int braking_follow_index{-1};
  std::string braking_follow_id{};
  double braking_follow_delta_s{std::numeric_limits<double>::infinity()};
  double braking_follow_required_distance_m{
      std::numeric_limits<double>::infinity()};
  double braking_follow_available_distance_m{
      std::numeric_limits<double>::infinity()};
  double braking_follow_speed_cap_mps{
      std::numeric_limits<double>::quiet_NaN()};
  // 追越禁止区間の停止parallel車に対する、限定PASS開始例外の最終承認。
  // permission CSV自体の診断値は変更せず、freshness・同一ID確認・曲率・
  // future/reentry除外・SafetyEvaluator通過を同じ周期に満たす場合だけtrue。
  bool confirmed_stationary_parallel_permission_exception{false};
  // current-section permissionを例外的に開始できる最終承認。direct slow-frontと
  // confirmed stationary-parallelのどちらでもtrueになるが、CSV診断値は変えない。
  bool permission_start_exception_active{false};
  // 復帰ゲートが閉じている間はRECOVERY候補を中心線へ動かさず、現在横位置を保持する。
  bool reentry_hold_active{false};
  // NaNなら従来のreentry_hold_v_max_mpsを使う。MPC
  // solve遅延だけの短時間holdは、
  // SafetyEvaluatorを通した現d保持候補に限って別の低速capを指定する。
  double reentry_hold_speed_cap_mps{std::numeric_limits<double>::quiet_NaN()};
  // ABORTの復帰が安全に完了した直後、高速カーブ中だけ現dを保持する。
  // この間はPASS候補を開始せず、Coreが毎周期RECOVERYをSafetyEvaluatorへ通す。
  bool post_abort_curve_hold_active{false};
  double pass_gap_required_m{0.0};
  double corner_abs_curvature{0.0};
  bool straight_overtake_start_allowed{true};
  double overtake_start_abs_curvature{0.0};
  std::string overtake_start_gate_reason{};
  // 通常のstraight gateが閉じた緩い曲線でだけ使う、制限済みPASS候補の診断。
  // trueでもSafetyEvaluatorを通るまでは開始gateを開かない。
  bool gentle_curve_safe_pass_eligible{false};
  bool gentle_curve_safe_pass_constraint_active{false};
  // 同周期に制限済みPASSがSafetyEvaluatorを通った時だけtrue。mode holdの
  // 例外にも使うため、曲率だけで開いたgateとは分離して保持する。
  bool gentle_curve_safe_pass_start_approved{false};
  // 制限済みPASSの横移動は開始dを基準にする。NaNなら候補生成時の現在dを使う。
  double gentle_curve_safe_pass_anchor_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  // 曲率から求めたPASS速度上限。SafetyEvaluatorが扱わない横加速度を
  // candidate生成とPREPARE/OVERTAKE継続中の両方で制限する。NaNなら例外PASS不可。
  double gentle_curve_safe_pass_speed_cap_mps{
      std::numeric_limits<double>::quiet_NaN()};
  // 禁止区間の停止障害物だけを対象にする制限PASS。通常のcurve gateや
  // permission CSVは変更せず、この候補が同周期のGate 2を通った時だけ
  // permission_start_exception_activeを立てる。
  bool stationary_no_pass_safe_pass_eligible{false};
  bool stationary_no_pass_safe_pass_constraint_active{false};
  bool stationary_no_pass_safe_pass_start_approved{false};
  double stationary_no_pass_safe_pass_anchor_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  double stationary_no_pass_safe_pass_speed_cap_mps{
      std::numeric_limits<double>::quiet_NaN()};
  // 直接前走車との大きなgapを詰めるFOLLOWだけでtrue。横並び、低速障害物、
  // stale/MPC不健全、future yieldでは常にfalseへ閉じる。
  bool follow_gap_closing_allowed{false};
  // PASSで速度上限を現在速度より上げてよいのは、FOLLOW gap-closingと同等の
  // freshness/MPC health条件を満たす周期だけ。falseならPASS候補も現速度で予測する。
  bool pass_acceleration_allowed{false};
  // falseはPREPARE/OVERTAKE開始済みの同側PASS。開始時に確認済みの目標到達性を
  // 相手dの一時変動で再判定せず、現在horizonのSafetyEvaluatorだけを継続する。
  bool pass_target_corridor_preflight_required{true};
  bool overtake_permission_allowed{true};
  std::string overtake_permission_section_name{};
  std::string overtake_permission_reason{"default_allowed"};
  bool front_vehicle_low_speed{false};
  bool slow_front_exception_active{false};
  int slow_front_exception_count{0};
  double front_vehicle_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  bool pass_decision_frozen{false};
  std::string pass_decision_freeze_reason{};
  std::string pass_gap_reason{};
  bool leader_priority_active{false};
  bool leader_priority_latched{false};
  std::string leader_priority_id{};
  double leader_priority_delta_s{std::numeric_limits<double>::infinity()};
  std::string leader_priority_reason{};
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
  // /mpc/speed_profile_debugの新規受信ごとにNodeが増やす。planner
  // timer周期ではない。
  std::uint64_t sample_sequence{0U};
};

enum class ReentryMpcHealthState {
  HEALTHY = 0,
  TRANSIENT_LATENCY = 1,
  UNHEALTHY = 2,
  STALE = 3,
};

struct ReentryInputStatus {
  // Node側で検証した復帰判断の入力完全性。falseは「安全」とは解釈しない。
  bool ego_fresh{false};
  bool v2x_snapshot_fresh{false};
  bool all_observed_opponents_fresh{false};
  // collectOpponents()から除外された近接他車が無いこと。falseは未評価車両あり。
  bool all_observed_opponents_included{false};
  bool reference_valid{false};
  // mpc_healthyは通常ライン復帰を直ちに許せる健康状態。latency warningだけは
  // sample単位のhysteresisでTRANSIENT_LATENCYへ分離し、CBF/stale/infeasibleは
  // hard fail-safeとして扱う。
  bool mpc_healthy{false};
  bool mpc_health_fresh{false};
  bool mpc_hard_failure{false};
  bool mpc_latency_warning{false};
  std::uint64_t mpc_health_sample_sequence{0U};
};

struct ReentryGateResult {
  // 通常ラインへ戻る候補を全相手車両に対して評価した結果。
  bool requested{false};
  bool permitted{false};
  bool input_complete{false};
  int clear_cycles{0};
  int evaluated_opponent_count{0};
  std::string reason{};
  std::string blocking_vehicle_id{};
  double min_safety_margin{std::numeric_limits<double>::infinity()};
  double cbf_slack{0.0};
  double blocking_time_sec{std::numeric_limits<double>::quiet_NaN()};
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

struct OvertakePermissionRule {
  std::string name{};
  double s_start_m{0.0};
  double s_end_m{0.0};
  bool allow_overtake{true};
};

struct ActiveOvertakePermission {
  bool active{false};
  std::string name{};
  bool allow_overtake{true};
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
  bool parallel_follow_enabled{false};
  double parallel_follow_s_m{12.0};
  double parallel_follow_lateral_width_m{1.20};
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
  double recovery_release_lateral_error_m{0.60};
  double yield_release_lateral_error_m{0.60};
  double corner_follow_speed_margin_mps{0.20};
  double corner_yield_v_max_mps{3.0};
  bool straight_only_overtake_enabled{true};
  double straight_overtake_max_curvature_m_inv{0.025};
  double straight_overtake_lookahead_m{12.0};
  double straight_overtake_release_hysteresis_m_inv{0.005};
  // 直線限定gateを緩めるのではなく、緩い曲線だけで小さく遅いPASS候補を
  // SafetyEvaluatorへ先に通すための明示opt-in。0以下の上限は無効扱い。
  bool gentle_curve_safe_pass_enabled{false};
  double gentle_curve_safe_pass_max_curvature_m_inv{0.0};
  double gentle_curve_safe_pass_v_max_mps{0.0};
  double gentle_curve_safe_pass_max_lateral_displacement_m{0.0};
  // 曲線PASSだけに課す横加速度上限[m/s^2]。有限の正値でなければ例外PASSは
  // fail-closedにする。候補速度は sqrt(a_lat_max / abs(kappa)) 以下へ制限する。
  double gentle_curve_safe_pass_max_lateral_accel_mps2{0.0};
  double gentle_curve_safe_pass_max_cbf_slack{0.0};
  // trueでも時間だけでgateは延長しない。同周期のSafetyEvaluator承認済みPASSが
  // pass_safe_required_cyclesに達した時だけFOLLOWの最低holdを例外化する。
  bool gentle_curve_safe_pass_bypass_mode_hold_enabled{false};
  bool overtake_permission_profile_enabled{true};
  bool default_overtake_allowed{true};
  double overtake_permission_lookahead_m{8.0};
  bool slow_front_exception_enabled{true};
  // falseなら停止/低速障害物でも禁止permissionを例外許可しない。
  // 有効時もSafetyEvaluator通過済みPASSだけを開始対象にする。
  bool slow_front_permission_exception_enabled{false};
  double slow_front_exception_speed_mps{1.0};
  double slow_front_exception_distance_m{8.0};
  int slow_front_exception_required_cycles{3};
  // 0以下ならslow front例外で曲率gateを開かない。有限の正値を設定しても
  // 壁/CBFと候補SafetyEvaluatorは迂回しない。permission例外は別の明示設定が
  // 有効な時だけ、低速連続判定済みかつSafetyEvaluatorを通ったPASSに限る。
  double slow_front_exception_max_start_curvature_m_inv{0.0};
  bool slow_obstacle_chain_enabled{true};
  double slow_obstacle_chain_distance_m{12.0};
  // 停止した前方parallel車だけを、同一コリドーに入る前からPASSのSafetyEvaluatorへ載せる。
  // 通常のwide parallel/FOLLOW/permission例外には使わない明示opt-in。
  bool early_stationary_parallel_pass_enabled{false};
  // 停止parallel車のcurrent-section permission例外。early PASS probeとは別opt-inで、
  // falseならCSVの追越禁止をそのまま維持する。
  bool early_stationary_parallel_permission_exception_enabled{false};
  double early_stationary_parallel_pass_distance_m{8.0};
  double early_stationary_parallel_pass_lateral_width_m{1.5};
  // 現在の追越禁止区間で停止障害物を回避するための、通常PASSとは分離した
  // Gate 2限定の低速PASS設定。0以下/falseは経路全体を閉じる。
  bool stationary_no_pass_safe_pass_enabled{false};
  double stationary_no_pass_safe_pass_max_curvature_m_inv{0.0};
  double stationary_no_pass_safe_pass_v_max_mps{0.0};
  double stationary_no_pass_safe_pass_max_lateral_displacement_m{0.0};
  double stationary_no_pass_safe_pass_max_lateral_accel_mps2{0.0};
  double stationary_no_pass_safe_pass_max_cbf_slack{0.0};
  // 停止/極低速車に対するFOLLOW開始距離は、固定距離ではなくこの余裕を
  // 加えた制動到達距離で決める。max_distanceは探索上限であり開始閾値ではない。
  bool braking_follow_enabled{false};
  double braking_follow_max_distance_m{0.0};
  double braking_follow_trigger_margin_m{0.0};
  double large_lateral_error_threshold_m{0.60};
  double large_lateral_error_v_max_mps{2.5};
  double min_pass_gap_m{1.80};
  double pass_gap_hysteresis_m{0.25};
  // falseなら旧来の静的gap gateだけを使う。評価fixture比較用で、実運用はtrue。
  bool dynamic_pass_candidate_enabled{true};
  double yield_speed_margin_mps{0.60};
  double yield_min_speed_cap_mps{0.50};
  double yield_rejoin_gap_m{3.0};
  // 実際の下流制御より強い制動は安全評価に使わない。現行mux clampは1.5 m/s^2。
  double max_brake_decel_mps2{1.0};
  double longitudinal_response_delay_sec{0.25};
  double stationary_obstacle_speed_threshold_mps{0.30};
  // 通常ライン復帰を複数車両に対してfail-closedで許可するための設定。
  bool reentry_gate_enabled{true};
  int reentry_safe_cycles{5};
  double reentry_min_safety_margin_h{0.30};
  double reentry_evaluation_horizon_sec{4.0};
  double reentry_v2x_snapshot_stale_time_sec{0.50};
  double reentry_hold_v_max_mps{0.50};
  // MPC
  // solve遅延だけの時は、SafetyEvaluatorを通した現d保持に限りこのcapを使う。
  // stale/infeasible/CBF制約は常にreentry_hold_v_max_mps以下へ閉じる。
  double reentry_mpc_degraded_hold_v_max_mps{3.0};
  // reentry許可・中心収束後の高速カーブで使う現d保持の上限。ABORTを
  // 継続させず、SafetyEvaluatorを通したSPEED_GUARDへ分離する。
  double post_abort_curve_hold_v_max_mps{4.0};
  // 新規MPC sampleでslow solveがこの回数続いた時だけ現d holdへ入る。
  // 1は従来互換、2以上なら単発latencyはspeed capだけに留める。
  int reentry_mpc_latency_degraded_enter_samples{1};
  int reentry_mpc_unhealthy_enter_samples{2};
  int reentry_mpc_healthy_release_samples{3};
  bool reentry_require_mpc_health{true};
  double left_offset_m{0.70};
  double right_offset_m{-0.70};
  // legacy_fixed_offsetは既存の固定±dを優先する互換モード。
  // minimum_clearanceは相手楕円間隔を満たす最小横移動だけを目標にする。
  std::string pass_target_policy{"legacy_fixed_offset"};
  std::string overtake_lateral_profile_mode{"legacy"};
  std::string pass_horizon_publish_mode{"prepare_and_overtake"};
  double localized_avoidance_start_before_target_m{6.0};
  double localized_avoidance_full_offset_before_target_m{2.0};
  double localized_avoidance_hold_after_target_m{5.0};
  double localized_avoidance_merge_distance_m{8.0};
  double maneuver_latch_min_hold_sec{1.0};
  double maneuver_latch_target_update_alpha{0.0};
  double prepare_distance_m{8.0};
  double pass_distance_m{20.0};
  double merge_distance_m{12.0};
  double follow_speed_margin_mps{0.20};
  // 通常FOLLOWの前走車より遅いcapは維持しつつ、十分大きな同一レーンgapだけを
  // 小さなbonusで詰める。加速を含むs(t)はSafetyEvaluatorへ渡す。
  bool follow_gap_closing_enabled{false};
  double follow_gap_closing_target_gap_m{5.0};
  double follow_gap_closing_engage_gap_m{6.0};
  double follow_gap_closing_speed_gain_per_m{0.10};
  double follow_gap_closing_max_speed_bonus_mps{0.30};
  // 下流の許容最大加速以上を使い、最遠到達距離を保守的に予測する。
  double follow_gap_closing_assumed_accel_mps2{3.0};
  // PASSは速度capだけを上げて安全予測を据え置かない。想定加速を含むs(t)で
  // SafetyEvaluatorを通した時だけ、下流へこの上限を出す。
  double pass_speed_cap_mps{10.0};
  double pass_assumed_accel_mps2{3.0};
  // 前走車中心から必要楕円間隔よりさらに確保する横方向余裕。
  double pass_target_lateral_margin_m{0.10};
  double max_overtake_v_bonus_mps{0.30};
  double recovery_v_max_mps{8.5};
  double wall_margin_recovery_v_max_mps{8.5};
  double outside_corridor_recovery_centering_time_sec{1.0};
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
  double lateral_target_max_step_m{0.25};
  bool high_speed_curve_lateral_hold_enabled{true};
  double high_speed_curve_lateral_hold_min_speed_mps{4.0};
  double high_speed_curve_lateral_hold_release_speed_mps{2.5};
  double high_speed_curve_lateral_hold_release_curvature_m_inv{0.025};
  bool speed_only_fallback_enabled{true};
  double speed_only_fallback_v_max_mps{1.0};
  // 追越不可区間で安全に通常ラインへ戻れる時だけ使うレース用cap。
  // SAFE_STOP、衝突、wall/MPC healthなどのfail-safe capとは分離する。
  double normal_recovery_speed_only_v_max_mps{10.0};
  double opponent_collision_fallback_v_max_mps{0.5};
  bool side_by_side_leader_priority_enabled{true};
  double side_by_side_leader_priority_enter_s_m{1.0};
  double side_by_side_leader_priority_release_s_m{0.3};
  double side_by_side_leader_priority_hold_sec{1.0};
  double side_by_side_leader_priority_v_max_mps{3.0};
  bool wall_risk_speed_guard_enabled{true};
  double wall_soft_margin_m{0.25};
  double wall_risk_v_max_mps{5.0};
  bool mpc_health_speed_guard_enabled{true};
  int mpc_health_infeasible_count_threshold{1};
  double mpc_health_solve_time_warn_ms{80.0};
  double mpc_health_v_max_mps{3.0};
  double mpc_health_stale_time_sec{0.60};
  bool recovery_speed_guard_enabled{true};
  double recovery_speed_guard_v_max_mps{3.0};
  bool section_safety_profile_enabled{true};
  std::vector<SectionSafetyRule> section_safety_rules;
  std::vector<OvertakePermissionRule> overtake_permission_rules;
  bool safe_stop_enabled{true};
  double safe_stop_v_mps{0.20};
  int safe_stop_trigger_cycles{1};
  bool start_grace_safe_stop_enabled{true};
  double start_grace_duration_sec{8.0};
  double start_grace_max_speed_mps{1.5};
  int safe_stop_release_cycles{5};
  double safe_stop_release_front_gap_m{5.0};
  double safe_stop_release_wall_clearance_m{0.20};
  double safe_stop_lateral_error_threshold_m{0.40};
  double safe_stop_release_speed_mps{0.50};
};

// 入力: PASS方向、現在の自車d、対象車d、必要横離隔。
// 出力: CandidateBuilderとCoreの局所profileが共通で使うPASS目標d。
// 処理概要: minimum_clearanceでは余計に壁側へ寄せず、相手楕円間隔を満たす
// 最小横移動を選ぶ。対象が無い呼出しでは使わず、従来offsetは別途fallbackする。
inline double passTargetOffset(const PlannerConfig &config,
                               CandidateType pass_type, double ego_d_m,
                               double opponent_d_m,
                               double required_gap_m) {
  if (pass_type == CandidateType::PASS_LEFT) {
    const double required_target_d = opponent_d_m + required_gap_m;
    if (config.pass_target_policy == "minimum_clearance") {
      return ego_d_m > required_target_d ? ego_d_m : required_target_d;
    }
    return config.left_offset_m > required_target_d ? config.left_offset_m
                                                     : required_target_d;
  }
  if (pass_type == CandidateType::PASS_RIGHT) {
    const double required_target_d = opponent_d_m - required_gap_m;
    if (config.pass_target_policy == "minimum_clearance") {
      return ego_d_m < required_target_d ? ego_d_m : required_target_d;
    }
    return config.right_offset_m < required_target_d ? config.right_offset_m
                                                      : required_target_d;
  }
  return ego_d_m;
}

struct PlannerOutput {
  // ROSノードへ返す最終結果。override配列、debug指標、選択理由を含める。
  BehaviorMode mode{BehaviorMode::FREE_RUN};
  CandidateType selected{CandidateType::FASTEST};
  std::vector<double> lateral_offsets;
  std::vector<double> speed_caps;
  BlockedInfo blocked_info{};
  std::string reason{};
  // 横軌道と縦速度capは別契約でpublishできる。active_overrideは横列が
  // SafetyEvaluatorを通った時だけtrueにし、横列を作れないfail-closed時も
  // longitudinal_speed_cap_activeで安全側の減速要求を下流へ届ける。
  bool active_override{false};
  bool longitudinal_speed_cap_active{false};
  // Solver prediction horizonをPure Pursuitへ渡してよい横マヌーバの意図。
  // NONEは横override自体を否定しない。通常復帰などは通常trajectoryへ
  // 明示offsetを重ねられるが、solverが逸脱したpredictionは採用しない。
  enum class SolverHorizonIntent {
    NONE = 0,
    MANEUVER_AUTHORIZED = 1,
    MANDATORY_AVOIDANCE = 2,
  };
  SolverHorizonIntent solver_horizon_intent{SolverHorizonIntent::NONE};
  double target_lateral_offset_m{0.0};
  double min_cbf_h{std::numeric_limits<double>::quiet_NaN()};
  double cbf_slack{0.0};
  int active_cbf_constraint_count{0};
  bool safe_stop_triggered{false};
  bool start_grace_active{false};
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
  bool recovery_speed_guard_active{false};
  bool lateral_target_hold_active{false};
  std::string lateral_target_hold_reason{};
  bool published_lateral_safety_rejected{false};
  ReentryGateResult reentry_gate{};
  std::string lateral_profile_mode{"legacy"};
  bool maneuver_latch_active{false};
  std::string maneuver_latch_target_id{};
  double maneuver_latch_target_s_m{std::numeric_limits<double>::quiet_NaN()};
  double maneuver_latch_avoid_start_s_m{
      std::numeric_limits<double>::quiet_NaN()};
  double maneuver_latch_full_offset_start_s_m{
      std::numeric_limits<double>::quiet_NaN()};
  double maneuver_latch_full_offset_end_s_m{
      std::numeric_limits<double>::quiet_NaN()};
  double maneuver_latch_merge_end_s_m{std::numeric_limits<double>::quiet_NaN()};
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
