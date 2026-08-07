#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace overtake_planner {

// vehicle_infoと共通化したvirtual tire angleのhard stop。
// Runtime Planner nodeはこの値を超える設定を拒否し、Muxは独立した
// 最終clampを維持する。
inline constexpr double kVehicleHardSteeringTireAngleRad = 0.64;
// 100 HzのPPで-0.64 radから+0.64 radまでを1周期で許容する有限上限。
// rate limiter自体は、実車・路面条件に応じて設定を下げられるよう維持する。
inline constexpr double kVehicleHardSteeringRateRadps = 128.0;

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

// 既存prediction horizon内で、将来PASS/FOLLOW対象になり得る相手を選ぶ
// shadow-only診断。Core/StateMachine/CandidateBuilderはこの値をauthorityや
// target latchへ使用せず、Nodeがplan/debugへ転送するだけとする。
struct PredictivePassTargetShadowDiagnostic {
  bool evaluated{false};
  bool inputs_complete{false};
  bool valid{false};
  int target_index{-1};
  std::string target_id{};
  std::string source{"none"};
  std::string reason{"not_evaluated"};
  double current_delta_s_m{std::numeric_limits<double>::quiet_NaN()};
  double current_delta_d_m{std::numeric_limits<double>::quiet_NaN()};
  double relative_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double earliest_blocking_time_sec{std::numeric_limits<double>::quiet_NaN()};
  double predicted_opponent_s_m{std::numeric_limits<double>::quiet_NaN()};
  double predicted_opponent_d_m{std::numeric_limits<double>::quiet_NaN()};
};

enum class PassTransitionDeadlineSource {
  NONE,
  PROFILE_MARKER,
  TARGET_CLEARANCE,
};

// PASS横遷移のdeadline判定を、認可条件とは独立して記録する診断。
// CandidateBuilderだけが値を生成し、Core/Nodeは左右候補の観測用に転送する。
// NaNは未評価値を表し、SafetyEvaluatorやStateMachineは本構造体を参照しない。
struct PassTransitionDeadlineDiagnostic {
  bool evaluated{false};
  bool input_valid{false};
  bool requires_new_lateral_transition{false};
  bool reachable{false};
  PassTransitionDeadlineSource source{PassTransitionDeadlineSource::NONE};
  double lateral_shift_m{std::numeric_limits<double>::quiet_NaN()};
  double transition_start_s_m{std::numeric_limits<double>::quiet_NaN()};
  double transition_end_s_m{std::numeric_limits<double>::quiet_NaN()};
  double required_transition_m{std::numeric_limits<double>::quiet_NaN()};
  double available_deadline_m{std::numeric_limits<double>::quiet_NaN()};
  double deadline_slack_m{std::numeric_limits<double>::quiet_NaN()};
  double deadline_speed_cap_mps{std::numeric_limits<double>::quiet_NaN()};
  double proposal_speed_cap_mps{std::numeric_limits<double>::quiet_NaN()};
  double evaluated_tracking_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double proposal_horizon_sec{std::numeric_limits<double>::quiet_NaN()};
  double proposal_endpoint_arc_m{std::numeric_limits<double>::quiet_NaN()};
  double proposal_end_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double proposal_time_to_required_transition_sec{
      std::numeric_limits<double>::quiet_NaN()};
  double pp_required_arc_m{std::numeric_limits<double>::quiet_NaN()};
};

// PASS開始がtracking契約で止まった周期だけの診断。候補選択、grant、probe、
// token、速度authorityには使わない。`first_false`は候補形状からNode入力までを
// 固定順で調べた最初の不成立であり、未評価deadlineはNOT_EVALUATEDとして明示する。
struct PassStartTrackingDiagnostic {
  bool evaluated{false};
  bool candidate_present{false};
  CandidateType candidate_type{CandidateType::FASTEST};
  bool actual_pose_start_evaluated{false};
  bool actual_pose_start{false};
  double endpoint_arc_m{std::numeric_limits<double>::quiet_NaN()};
  double required_arc_m{std::numeric_limits<double>::quiet_NaN()};
  bool transition_deadline_evaluated{false};
  bool transition_deadline_present{false};
  PassTransitionDeadlineDiagnostic transition_deadline{};
  bool controller_tracking_profile_valid{false};
  bool desired_path_trackable{false};
  bool pure_pursuit_command_trackable{false};
  bool controller_status_received{false};
  std::uint32_t controller_plan_generation{0U};
  std::uint32_t controller_expected_generation{0U};
  std::string controller_status_reason{"NOT_EVALUATED"};
  bool controller_mpc_horizon_usable{false};
  bool controller_continuity_usable{false};
  std::string first_false{"NOT_EVALUATED"};
};

struct CandidateTrajectory {
  // MPCへ渡す横オフセット列と速度上限列。安全評価とスコアもここへ保持する。
  CandidateType type{CandidateType::FASTEST};
  std::vector<double> t;
  // horizon先頭からの走行距離[m]。時間indexのd/v列を、下流の参照軌道へ
  // 点番号ではなく物理距離で再サンプルするために使う。
  std::vector<double> longitudinal_offsets_m;
  std::vector<double> s;
  std::vector<double> d;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  // 安全評価用の到達可能速度。v_refとは別に、応答遅れと制動上限を含む。
  // 生成時の実測ego速度は、ACK待ちsnapshotを現在egoへ張り直してよいかの
  // 判定に使う。predicted_speed先頭は実行遅れreserveを含むため代用しない。
  double longitudinal_initial_measured_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> predicted_speed_mps;
  // 下流MPC/PPへの即時速度上限。各周期で先頭要素が直ちに消費される。
  std::vector<double> v_ref;
  // SafetyEvaluatorを実際に通った証跡。生成直後のfeasible既定値だけでは
  // V2のtrajectory認可に使わない。
  bool safety_evaluated{false};
  bool feasible{true};
  // PASS目標へ到達するまでのs依存安全回廊を事前に通過できるか。
  // SafetyEvaluatorはこのfalseをwall/CBF評価より先にrejectする。
  bool pass_target_corridor_valid{true};
  // PASS横遷移が対象/安全deadlineより前に完了できるか。falseなら短い残距離へ
  // profileを圧縮せず、Coreは同targetのFOLLOW/current-d fallbackを選ぶ。
  bool pass_transition_deadline_reachable{true};
  double required_pass_transition_m{std::numeric_limits<double>::infinity()};
  double available_pass_transition_deadline_m{
      std::numeric_limits<double>::infinity()};
  PassTransitionDeadlineDiagnostic pass_transition_deadline{};
  // 生成したCartesian列の総曲率・総操舵速度がactive controller契約内か。
  // Frenet横profile単体が成立しても基準線曲率を含めて不成立ならrejectする。
  bool controller_tracking_profile_valid{true};
  // 横profileそのものの曲率・操舵速度と、active PurePursuitが同じprofileへ
  // 出すcommandを分離して記録する。最終認可は両方のANDだけを使う。
  bool desired_path_trackable{true};
  bool pure_pursuit_command_trackable{true};
  // 正速度のmoving targetを、最終速度capと加速契約でbounded時間内に
  // 抜き切れるか。停止対象には適用せず、falseはSafetyEvaluatorで明示reject。
  bool moving_target_relatively_reachable{true};
  // PASS候補で実際に計画した最終横目標と、raw profile全区間の最小壁余裕。
  // debugでは選択候補ではなく左右候補そのものを追えるようにする。
  double planned_target_d_m{std::numeric_limits<double>::quiet_NaN()};
  // ATTACK_FOLLOWではtransactionに保持する最終PASS目標と、この周期に
  // SafetyEvaluatorへ渡す実行目標を分離する。壁回廊が狭い周期に現在dを
  // 保持しても、元のtarget dを完了前に書き換えないための診断契約である。
  double committed_attack_follow_target_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  bool attack_follow_safe_lateral_hold{false};
  // commit済みATTACK_FOLLOWが同周期の全相手評価でopponent_collisionに
  // なった時だけ、元候補の縦profileを変えず実測current-dへ張り直した別候補。
  // corridor起因の一般safe holdとは異なり、再度SafetyEvaluator・壁・追従性を
  // 全て通過した候補だけがtransport identityを得る。
  bool attack_follow_opponent_collision_current_d_hold{false};
  // current-d holdがwall rejectの時に限り、実測dから境界内側へ
  // 0.15 m接続した独立FOLLOW。Coreが現周期の全安全・追従性を
  // 再評価した後だけtrueにし、shadow診断には与えない。
  bool attack_follow_opponent_collision_inward_connector{false};
  double corridor_min_margin_m{std::numeric_limits<double>::quiet_NaN()};
  // active controllerが横profileを受理するために必要な前方arc長[m]。
  // 実際に生成したlongitudinal_offsets_m.back()とdebugで直接比較する。
  double required_controller_spatial_horizon_m{0.0};
  // SafetyEvaluator後に、同じd/v/ds列がcontrollerの必要空間arcまで評価済みで
  // あることを示す証跡。STOP/HOLDの横authorityはこのproofなしに出さない。
  bool controller_spatial_horizon_proof_valid{false};
  double score{0.0};
  double min_safety_margin{std::numeric_limits<double>::infinity()};
  double cbf_slack{0.0};
  int active_safety_constraint_count{0};
  std::string reject_reason{};
  std::string blocking_opponent_id{};
  double blocking_time_sec{std::numeric_limits<double>::quiet_NaN()};
  // opponent_collisionを最初に成立させた同一補間sampleの位置。Cartesianは
  // map frame、Frenetはactive reference frame、timeは候補先頭からの相対秒。
  // 診断専用で、候補選択やmotion authorityには使用しない。
  double blocking_candidate_x_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_candidate_y_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_candidate_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double blocking_candidate_s_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_candidate_d_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_opponent_x_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_opponent_y_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_opponent_s_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_opponent_d_m{std::numeric_limits<double>::quiet_NaN()};
  // wall_footprint_marginを最初に成立させたsampleとcornerの診断。
  // corridor_min_margin_mとは分離し、候補選択やmotion authorityには使わない。
  bool blocking_wall_footprint_valid{false};
  int blocking_wall_segment_index{-1};
  double blocking_wall_segment_ratio{std::numeric_limits<double>::quiet_NaN()};
  int blocking_wall_corner_index{-1};
  double blocking_wall_time_sec{std::numeric_limits<double>::quiet_NaN()};
  double blocking_wall_candidate_x_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_wall_candidate_y_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_wall_candidate_yaw_rad{
      std::numeric_limits<double>::quiet_NaN()};
  double blocking_wall_candidate_s_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_wall_candidate_d_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_wall_corner_x_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_wall_corner_y_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_wall_corner_s_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_wall_corner_d_m{std::numeric_limits<double>::quiet_NaN()};
  double blocking_wall_corridor_d_min_m{
      std::numeric_limits<double>::quiet_NaN()};
  double blocking_wall_corridor_d_max_m{
      std::numeric_limits<double>::quiet_NaN()};
  double blocking_wall_physical_clearance_m{
      std::numeric_limits<double>::quiet_NaN()};
  double blocking_wall_effective_clearance_m{
      std::numeric_limits<double>::quiet_NaN()};
  // s(t) に使った減速到達可能性。v_refの即時capとは独立して安全判定する。
  bool longitudinal_profile_valid{true};
  double assumed_brake_decel_mps2{0.0};
  double response_delay_sec{0.0};
  double required_brake_distance_m{std::numeric_limits<double>::infinity()};
  double available_brake_distance_m{std::numeric_limits<double>::infinity()};
};

// STOP/HOLDを含む横追従authorityのPlanner側空間proof。SafetyEvaluator済みの
// 同一candidateのd/v/dsが、PPに必要なarcをちょうど満たす場合だけ許可する。
// 浮動小数のepsilonで必要arc未達を許さず、次表現可能値でもfail-closedにする。
inline bool
hasControllerSpatialHorizonProof(const CandidateTrajectory &candidate) {
  const std::size_t point_count = candidate.longitudinal_offsets_m.size();
  if (!candidate.safety_evaluated || !candidate.feasible ||
      !candidate.longitudinal_profile_valid ||
      !candidate.controller_tracking_profile_valid ||
      !candidate.desired_path_trackable ||
      !candidate.pure_pursuit_command_trackable ||
      !std::isfinite(candidate.required_controller_spatial_horizon_m) ||
      candidate.required_controller_spatial_horizon_m <= 0.0 ||
      point_count <= 1U || candidate.d.size() != point_count ||
      candidate.v_ref.size() != point_count) {
    return false;
  }
  if (std::abs(candidate.longitudinal_offsets_m.front()) > 1.0e-5 ||
      candidate.longitudinal_offsets_m.back() <= 1.0e-6) {
    return false;
  }
  for (std::size_t i = 0U; i < point_count; ++i) {
    if (!std::isfinite(candidate.longitudinal_offsets_m[i]) ||
        !std::isfinite(candidate.d[i]) || !std::isfinite(candidate.v_ref[i]) ||
        (i > 0U && candidate.longitudinal_offsets_m[i] <
                       candidate.longitudinal_offsets_m[i - 1U])) {
      return false;
    }
  }
  return candidate.longitudinal_offsets_m.back() >=
         candidate.required_controller_spatial_horizon_m;
}

// 内側connectorの開始と終端は、同じPASS側の中心reserve外に
// 残らなければならない。reserve内開始や境界から反対側への横断は
// SafetyEvaluatorと別のtransaction形状契約としてfail-closedにする。
inline bool inwardConnectorStaysOutsideCenterReserve(double start_d_m,
                                                     double terminal_d_m,
                                                     double center_reserve_m) {
  if (!std::isfinite(start_d_m) || !std::isfinite(terminal_d_m) ||
      !std::isfinite(center_reserve_m) || center_reserve_m < 0.0) {
    return false;
  }
  return (start_d_m >= center_reserve_m && terminal_d_m >= center_reserve_m) ||
         (start_d_m <= -center_reserve_m && terminal_d_m <= -center_reserve_m);
}

struct SupervisorV2CandidateDiagnostic {
  // V2へ実際に渡した候補ごとのSafetyEvaluator/追従可能性の証跡。
  // 選択候補だけのreject reasonでは左右PASS不成立原因を識別できないため、
  // plan generationへ影響しない観測専用情報として保持する。
  bool generated{false};
  bool safety_evaluated{false};
  bool feasible{false};
  bool controller_tracking_profile_valid{false};
  bool desired_path_trackable{false};
  bool pure_pursuit_command_trackable{false};
  bool moving_target_relatively_reachable{false};
  std::string reject_reason{};
  double endpoint_arc_m{std::numeric_limits<double>::quiet_NaN()};
  double required_arc_m{std::numeric_limits<double>::quiet_NaN()};
  double planned_target_d_m{std::numeric_limits<double>::quiet_NaN()};
  double min_safety_margin{std::numeric_limits<double>::quiet_NaN()};
  std::string blocking_opponent_id{};
  double blocking_time_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct AttackFollowInnerBandProbeDiagnostic {
  // C-002AIのshadow専用sample。実行候補・score・transport identityには
  // 接続せず、同じ縦profile上の滑らかな内側connectorを全安全評価した結果だけ。
  double inward_delta_m{std::numeric_limits<double>::quiet_NaN()};
  double terminal_d_m{std::numeric_limits<double>::quiet_NaN()};
  bool longitudinal_contract_unchanged{false};
  bool safety_evaluated{false};
  bool corridor_valid{false};
  bool controller_tracking_profile_valid{false};
  bool desired_path_trackable{false};
  bool pure_pursuit_command_trackable{false};
  bool all_checks_pass{false};
  std::string reject_reason{};
  double min_safety_margin{std::numeric_limits<double>::quiet_NaN()};
  double corridor_min_margin_m{std::numeric_limits<double>::quiet_NaN()};
};

struct AttackFollowInnerBandDiagnostic {
  static constexpr std::size_t kMaxProbeCount = 4U;

  bool evaluated{false};
  bool rate_limited{false};
  bool complete{false};
  std::string status_reason{};
  double source_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  std::string target_id{};
  CandidateType pass_type{CandidateType::FASTEST};
  double source_current_d_m{std::numeric_limits<double>::quiet_NaN()};
  double committed_target_d_m{std::numeric_limits<double>::quiet_NaN()};
  int inward_direction_sign{0};
  int evaluated_probe_count{0};
  int feasible_probe_count{0};
  int wall_reject_count{0};
  int opponent_reject_count{0};
  int other_reject_count{0};
  double outermost_feasible_d_m{std::numeric_limits<double>::quiet_NaN()};
  double innermost_feasible_d_m{std::numeric_limits<double>::quiet_NaN()};
  std::array<AttackFollowInnerBandProbeDiagnostic, kMaxProbeCount> probes{};
};

struct StateLatticeShadowCandidateMetrics {
  // current/latticeの同一snapshot比較だけに使う。候補選択、score、transport、
  // motion authorityへは接続しない。
  bool safety_evaluated{false};
  bool feasible{false};
  bool wall_rejected{false};
  double wall_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double cbf_min_margin{std::numeric_limits<double>::quiet_NaN()};
  double cbf_slack{std::numeric_limits<double>::quiet_NaN()};
  std::string cbf_blocking_opponent_id{};
  double cbf_blocking_time_sec{std::numeric_limits<double>::quiet_NaN()};
  double pure_pursuit_required_arc_m{std::numeric_limits<double>::quiet_NaN()};
  double pure_pursuit_available_arc_m{std::numeric_limits<double>::quiet_NaN()};
  double deadline_required_arc_m{std::numeric_limits<double>::quiet_NaN()};
  double deadline_available_arc_m{std::numeric_limits<double>::quiet_NaN()};
  double deadline_slack_m{std::numeric_limits<double>::quiet_NaN()};
  std::string first_reject_reason{};
};

struct StateLatticeShadowComparison {
  // C-002AZ shadow backendの観測record。生成物をPlannerOutputのlive trajectory
  // fieldsへコピーせず、Nodeはmetrics JSONへ直列化するだけとする。
  bool requested{false};
  bool snapshot_complete{false};
  bool geometry_generated{false};
  bool adapter_valid{false};
  bool cartesian_trackability_valid{false};
  bool cartesian_trackable{false};
  // Current applied candidateのPP exact tupleだけを観測する。lattice geometry
  // へは再利用せず、候補選択・SafetyConstraint・authorityにも接続しない。
  bool current_pp_exact_snapshot_valid{false};
  std::string current_pp_exact_snapshot_reason{"disabled"};
  std::uint64_t current_pp_command_sequence{0U};
  double current_pp_valid_until_sec{std::numeric_limits<double>::quiet_NaN()};
  double current_pp_resolved_lookahead_m{
      std::numeric_limits<double>::quiet_NaN()};
  double current_pp_requested_steering_rad{
      std::numeric_limits<double>::quiet_NaN()};
  double current_pp_bounded_steering_rad{
      std::numeric_limits<double>::quiet_NaN()};
  bool evaluated{false};
  std::string status_reason{"disabled"};
  std::string target_id{};
  CandidateType pass_type{CandidateType::FASTEST};
  int pass_side{0};
  std::uint64_t snapshot_cycle{0U};
  double snapshot_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  double ego_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  std::size_t opponent_count{0U};
  std::uint64_t snapshot_hash{0U};
  StateLatticeShadowCandidateMetrics current{};
  StateLatticeShadowCandidateMetrics lattice{};
};

struct LocalizedLateralWaypoint {
  // 同一低速車列を1本の横移動へ潰さず、各車の安全楕円へ入る前に必要な
  // 外側dへ段階的に到達するためのFrenet waypoint。認可後もtarget IDと
  // target dは固定し、freshな同一ID観測に対してsだけ前進追従させる。
  std::string target_id{};
  double target_s_m{std::numeric_limits<double>::quiet_NaN()};
  double target_d_m{std::numeric_limits<double>::quiet_NaN()};
  // ego相対の半周折返しでは、相手が半周以上先行した時に周回数を失う。
  // waypointごとの前周期観測から進捗を積算し、target_s_m更新の観測座標に使う。
  double observed_unwrapped_s_m{std::numeric_limits<double>::quiet_NaN()};
  double last_observed_wrapped_s_m{std::numeric_limits<double>::quiet_NaN()};
  double last_observed_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct LocalizedLateralProfile {
  // PASS候補の横オフセットを、相手車両のs位置に紐づいた局所プロファイルとして固定する。
  bool active{false};
  CandidateType pass_type{CandidateType::FASTEST};
  std::string target_id{};
  double created_time_sec{std::numeric_limits<double>::quiet_NaN()};
  double anchor_s_m{std::numeric_limits<double>::quiet_NaN()};
  // PASSが半周を越えて継続してもanchor基準の符号付き差分へ折り返さないよう、
  // freshな自車観測を周期間で積算した連続sを保持する。
  double ego_unwrapped_s_m{std::numeric_limits<double>::quiet_NaN()};
  double last_ego_wrapped_s_m{std::numeric_limits<double>::quiet_NaN()};
  double last_ego_stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  double target_s_m{std::numeric_limits<double>::quiet_NaN()};
  double avoid_start_s_m{std::numeric_limits<double>::quiet_NaN()};
  double full_offset_start_s_m{std::numeric_limits<double>::quiet_NaN()};
  // 初回認可時にactive controller契約から決めたmarker相対距離。targetが
  // 前進しても同じ横形状を平行移動し、途中で急峻化・緩和しない。
  double avoid_start_before_target_m{std::numeric_limits<double>::quiet_NaN()};
  double full_offset_before_target_m{std::numeric_limits<double>::quiet_NaN()};
  double full_offset_end_s_m{std::numeric_limits<double>::quiet_NaN()};
  double merge_end_s_m{std::numeric_limits<double>::quiet_NaN()};
  double start_d_m{0.0};
  double target_d_m{0.0};
  std::vector<LocalizedLateralWaypoint> chain_waypoints{};
  // target IDは最初の追越対象に固定したまま、同一コリドーのfreshな低速車列が
  // 続く場合だけ横位置保持と完了基準を列末尾まで延長する。
  std::string chain_tail_id{};
  double chain_tail_s_m{std::numeric_limits<double>::quiet_NaN()};
  double chain_tail_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  int chain_target_count{0};
  // Gate 2でこのprofile方向のlocalized candidateが一度でも認可された後はtrue。
  // falseの間だけ反対側候補を評価でき、true以降は対象通過完了までside/IDを固定する。
  bool pass_safety_approved_once{false};
  // Gate 2認可済みだがまだPASSをpublishしていない間、同一fresh targetの
  // classifier-only dropoutを再評価できる残り期間を数える。過去の安全結果を
  // 流用する値ではなく、毎周期の候補生成を許すbounded lifetimeである。
  int pass_start_target_continuity_cycles{0};
  // Gate 2で候補が成立しただけではなく、同じtarget/sideのPASS trajectoryが
  // authoritative outputとして実際にpublishされる周期へ到達した後だけtrue。
  // 未実行の候補評価を未完了transactionやABORT開始の根拠にしない。
  bool pass_execution_committed{false};
  // 同じtarget/sideのPASS payloadを一周期publishし、そのgenerationをfinal
  // Pure Pursuitが実際に追従可能と報告した後だけtrue。単なる1 generation遅れを
  // 初回PASS開始の根拠にせず、走行開始後のtransport skewだけを継続許可する。
  bool pass_tracking_continuity_armed{false};
  // 直前周期に同じtransactionのPASS trajectoryをpublishしたことを示す。
  // 次周期のgeneration一致proofと組み合わせて上のarmedを立てる一周期状態。
  bool pass_tracking_proof_published_last_cycle{false};
  // Gate 2認可後も実車が選択側へ一度も進まず、fresh対象だけが前方へ離れる
  // 状態を連続確認する。単発V2X揺れや一時加速ではtargetを解放しない。
  int unstarted_target_pulling_away_cycles{0};
  // ラッチ対象をFrenet相対s・相対速度・PASS候補安全余裕で抜き切ったと
  // 確認した後だけtrue。次の対象へ切り替える許可として使う。
  bool pass_complete_confirmed{false};
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
  // 通常のparallel FOLLOW帯外でも、PASSが同周期に不成立ならCoreが一度だけ
  // current-d FOLLOWとして再評価したことを記録する。候補生成・SafetyEvaluator
  // を通った事実の診断であり、PASS開始や未完了transactionのtarget/sideを
  // 変更するauthorityには使わない。
  bool parallel_follow_recheck_attempted{false};
  std::string parallel_follow_recheck_reason{};
  // PASS開始認可前の攻めFOLLOWは、候補PASS側へ先回りせず現在dを保持する。
  // freshな同一moving target、全相手予測、controller healthが揃う周期だけ
  // Coreが立て、生成したFOLLOWは通常どおりSafetyEvaluatorへ通す。
  bool prestart_attack_follow_hold_lateral{false};
  // PASS中に前方chainへ攻めFOLLOWする場合は、現在dへ毎周期張り直さず、
  // SafetyEvaluatorを通したPASS側の横位置をトランザクション完了まで保持する。
  bool attack_follow_hold_pass_side{false};
  double attack_follow_target_d_m{std::numeric_limits<double>::quiet_NaN()};
  // freshなPASS transactionとchain-tail制動が同時に成立した場合だけ、
  // 下流capまでの加速到達可能性をSafetyEvaluatorへ織り込む。
  // false時は現在速度を越える到達距離を仮定せず、短い空間horizonを
  // 横overrideとして実行可能と誤認させない。
  bool attack_follow_acceleration_allowed{false};
  // legacy候補集合へ実際に載せたATTACK_FOLLOWの評価結果。開始前のcurrent-d
  // FOLLOWとcommit後のPASS側FOLLOWを含み、上のphase別flagで区別する。
  // V2 shadow候補とは混同せず、候補生成からpublishまでをbagで追跡する。
  bool attack_follow_candidate_generated{false};
  bool attack_follow_candidate_feasible{false};
  bool attack_follow_candidate_safe_lateral_hold{false};
  bool attack_follow_candidate_opponent_collision_current_d_hold{false};
  bool attack_follow_candidate_opponent_collision_inward_connector{false};
  bool attack_follow_current_d_hold_variant_generated{false};
  bool attack_follow_current_d_hold_variant_feasible{false};
  bool attack_follow_current_d_hold_variant_used{false};
  std::string attack_follow_current_d_hold_variant_reject_reason{};
  bool attack_follow_inward_connector_variant_generated{false};
  bool attack_follow_inward_connector_variant_feasible{false};
  bool attack_follow_inward_connector_variant_used{false};
  int attack_follow_inward_connector_direction_sign{0};
  double attack_follow_inward_connector_terminal_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  std::string attack_follow_inward_connector_variant_reject_reason{};
  std::string attack_follow_current_d_hold_source_blocking_opponent_id{};
  double attack_follow_current_d_hold_source_blocking_time_sec{
      std::numeric_limits<double>::quiet_NaN()};
  bool attack_follow_current_d_hold_blocking_wall_footprint_valid{false};
  int attack_follow_current_d_hold_blocking_wall_segment_index{-1};
  double attack_follow_current_d_hold_blocking_wall_segment_ratio{
      std::numeric_limits<double>::quiet_NaN()};
  int attack_follow_current_d_hold_blocking_wall_corner_index{-1};
  double attack_follow_current_d_hold_blocking_wall_time_sec{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_current_d_hold_blocking_wall_candidate_x_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_current_d_hold_blocking_wall_candidate_y_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_current_d_hold_blocking_wall_candidate_yaw_rad{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_current_d_hold_blocking_wall_candidate_s_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_current_d_hold_blocking_wall_candidate_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_current_d_hold_blocking_wall_corner_x_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_current_d_hold_blocking_wall_corner_y_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_current_d_hold_blocking_wall_corner_s_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_current_d_hold_blocking_wall_corner_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_current_d_hold_blocking_wall_corridor_d_min_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_current_d_hold_blocking_wall_corridor_d_max_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_current_d_hold_blocking_wall_physical_clearance_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_current_d_hold_blocking_wall_effective_clearance_m{
      std::numeric_limits<double>::quiet_NaN()};
  bool attack_follow_candidate_tracking_profile_valid{false};
  std::string attack_follow_candidate_reject_reason{};
  double attack_follow_candidate_planned_target_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_candidate_committed_target_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_candidate_corridor_min_margin_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_candidate_min_safety_margin{
      std::numeric_limits<double>::quiet_NaN()};
  std::string attack_follow_candidate_blocking_opponent_id{};
  double attack_follow_candidate_blocking_time_sec{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_candidate_blocking_candidate_x_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_candidate_blocking_candidate_y_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_candidate_blocking_candidate_yaw_rad{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_candidate_blocking_candidate_s_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_candidate_blocking_candidate_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_candidate_blocking_opponent_x_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_candidate_blocking_opponent_y_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_candidate_blocking_opponent_s_m{
      std::numeric_limits<double>::quiet_NaN()};
  double attack_follow_candidate_blocking_opponent_d_m{
      std::numeric_limits<double>::quiet_NaN()};
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
  // blocked/FOLLOWへは昇格させず、PASSがGate
  // 2を通らない限り通常走行を置き換えない。
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
  // CandidateBuilderへ渡されたopponentsが、Nodeで観測した近接車を欠落なく含み、
  // V2X/reference/egoもfreshであること。長時間の静止相手予測にだけ使う。
  bool opponent_prediction_inputs_complete{false};
  PredictivePassTargetShadowDiagnostic predictive_pass_target_shadow{};
  // strictなparallel接近で後方へ譲る間は、相手dへ横切らず現在dを保持する。
  bool parallel_yield_hold_lateral{false};
  // V2 ABORTでRECOVERY holdが不成立な場合の停止候補だけに使う。
  // 停止しながら未評価の中心復帰を始めないよう、現在dを維持する。
  bool abort_safe_stop_hold_lateral{false};
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
  bool early_wall_recovery_probe_requested{false};
  bool early_wall_recovery_probe_generated{false};
  bool early_wall_recovery_probe_feasible{false};
  double early_wall_recovery_clearance_threshold_m{
      std::numeric_limits<double>::quiet_NaN()};
  // Authorityを変更しないread-only診断。既存のprobe request predicateを
  // 評価順にたどり、最初にfalseになった固定tokenを記録する。
  // 全predicate成立時は "none"、評価前は "not_evaluated"。
  std::string early_wall_recovery_probe_first_false{"not_evaluated"};
  std::string early_wall_recovery_reject_reason{};
  double left_pass_gap_m{std::numeric_limits<double>::infinity()};
  double right_pass_gap_m{std::numeric_limits<double>::infinity()};
  bool can_pass_left{false};
  bool can_pass_right{false};
  // can_pass_* は静的gap診断値。実際の開始可否は候補安全評価の結果で判断する。
  bool pass_left_candidate_generated{false};
  bool pass_right_candidate_generated{false};
  bool pass_left_candidate_feasible{false};
  bool pass_right_candidate_feasible{false};
  bool pass_left_candidate_tracking_profile_valid{false};
  bool pass_right_candidate_tracking_profile_valid{false};
  bool pass_left_candidate_desired_path_trackable{false};
  bool pass_right_candidate_desired_path_trackable{false};
  bool pass_left_candidate_pure_pursuit_command_trackable{false};
  bool pass_right_candidate_pure_pursuit_command_trackable{false};
  // commit済み同一target/sideで、直前に実publish・安全確認した横d列だけを
  // 現周期の縦profileへ移植し、追従性と全相手SafetyEvaluatorを再実行した診断。
  bool committed_pass_snapshot_continuity_used{false};
  // 0.25 sの旧Safety proofではなく、commit済みの同一target/side空間形状を、
  // 現周期の縦予測・全相手SafetyEvaluatorへ再投入した時だけtrue。
  // 過去のfeasible結果は継承せず、ControllerTrackingStatusは実行authority側で
  // 独立に検証する。
  bool committed_pass_spatial_profile_continuity_used{false};
  double committed_pass_spatial_profile_tracking_error_m{
      std::numeric_limits<double>::quiet_NaN()};
  double committed_pass_spatial_profile_source_age_sec{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_left_candidate_endpoint_arc_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_right_candidate_endpoint_arc_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_left_candidate_required_arc_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_right_candidate_required_arc_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_left_candidate_target_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_right_candidate_target_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_left_candidate_corridor_min_margin_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_right_candidate_corridor_min_margin_m{
      std::numeric_limits<double>::quiet_NaN()};
  PassTransitionDeadlineDiagnostic pass_left_candidate_transition_deadline{};
  PassTransitionDeadlineDiagnostic pass_right_candidate_transition_deadline{};
  PassStartTrackingDiagnostic pass_start_tracking_diagnostic{};
  std::string pass_left_candidate_reject_reason{};
  std::string pass_right_candidate_reject_reason{};
  // early stationary PASSがGate 2で不成立の間、中心線へ横断せず現在dを
  // SafetyEvaluator済みRECOVERY候補で保持する。reentry holdとは別文脈。
  bool early_stationary_parallel_pass_hold_lateral{false};
  // Race arm直後だけ、停止した前方/隣接グリッド車を追越評価対象として扱う。
  // 分類自体はPASS許可ではなく、左右候補は通常どおりSafetyEvaluatorを通す。
  bool start_grid_target_active{false};
  bool start_grid_target_confirmation_pending{false};
  bool start_grid_target_confirmed_stationary{false};
  bool start_grid_follow_hold_lateral{false};
  // Gate 2未認可・未commitのstart-grid追従だけを、初期anchorへの復帰ではなく
  // freshな現在d保持として扱う明示契約。PASS準備/実行後のABORT/reentryや、
  // stale入力・回廊外・過大driftでは必ずfalseへ閉じる。
  bool start_grid_uncommitted_hold_active{false};
  // Gate 2未認可のstart-grid対象より、BlockedRiskAnalyzerが別IDの
  // 前方閉塞車を優先した周期だけtrue。認可後のID固定には使わない。
  bool start_grid_target_superseded_by_blocked_front{false};
  // authoritative blocked frontへ引き継いだ後、同じstart window内で古い
  // grid候補を再選択しない。次周期以降もhandoff契約を診断できるよう保持する。
  bool start_grid_target_reselection_suppressed{false};
  std::string start_grid_superseded_target_id{};
  std::string start_grid_replacement_target_id{};
  // start-gridの初期横位置はanchorに保持するだけで、それ自体は復帰対象ではない。
  // SafetyEvaluator/Gate 2がPASSを認可した後だけtrueとし、対象がFOLLOW範囲外へ
  // 出た場合は長期reentry gateが中心復帰を認可するまでtarget
  // ID/anchorを保持する。
  bool start_grid_lateral_release_pending{false};
  double start_grid_hold_target_d_m{std::numeric_limits<double>::quiet_NaN()};
  int start_grid_target_index{-1};
  std::string start_grid_target_id{};
  double start_grid_target_delta_s{std::numeric_limits<double>::infinity()};
  double start_grid_target_delta_d{0.0};
  double start_grid_target_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double start_grid_target_age_sec{0.0};
  double start_grid_ego_progress_m{0.0};
  // 停止障害物へ制動距離で接近したFOLLOWは、通常ラインへ横切らず現dを
  // SafetyEvaluatorへ渡す。成立しない時も同じ現d RECOVERYだけを再評価する。
  bool braking_follow_active{false};
  bool braking_follow_hold_lateral{false};
  bool braking_follow_feasible{false};
  std::string braking_follow_candidate_reject_reason{};
  bool braking_follow_candidate_tracking_profile_valid{false};
  double braking_follow_candidate_min_safety_margin{
      std::numeric_limits<double>::infinity()};
  int braking_follow_index{-1};
  std::string braking_follow_id{};
  double braking_follow_delta_s{std::numeric_limits<double>::infinity()};
  double braking_follow_required_distance_m{
      std::numeric_limits<double>::infinity()};
  double braking_follow_trigger_distance_m{
      std::numeric_limits<double>::infinity()};
  double braking_follow_available_distance_m{
      std::numeric_limits<double>::infinity()};
  double braking_follow_relative_speed_mps{0.0};
  double braking_follow_target_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double braking_follow_ttc_sec{std::numeric_limits<double>::infinity()};
  double braking_follow_speed_cap_mps{std::numeric_limits<double>::quiet_NaN()};
  // FOLLOW/減速を発動する制動距離より手前で、停止・低速対象をPASS候補生成へ
  // 渡すcallback-local分類。これ自体は速度・横移動・motion authorityを持たず、
  // 既存profile、SafetyEvaluator、safe-cycle、exact ACKを迂回しない。
  bool early_low_speed_pass_target_active{false};
  int early_low_speed_pass_target_index{-1};
  std::string early_low_speed_pass_target_id{};
  double early_low_speed_pass_target_delta_s_m{
      std::numeric_limits<double>::infinity()};
  double early_low_speed_pass_target_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double early_low_speed_pass_required_distance_m{
      std::numeric_limits<double>::infinity()};
  double early_low_speed_pass_required_transition_m{
      std::numeric_limits<double>::infinity()};
  double early_low_speed_pass_required_controller_arc_m{
      std::numeric_limits<double>::infinity()};
  // 追越禁止区間の停止parallel車に対する、限定PASS開始例外の最終承認。
  // permission CSV自体の診断値は変更せず、freshness・同一ID確認・曲率・
  // future/reentry除外・SafetyEvaluator通過を同じ周期に満たす場合だけtrue。
  bool confirmed_stationary_parallel_permission_exception{false};
  // current-section permissionを例外的に開始できる最終承認。direct slow-frontと
  // confirmed
  // stationary-parallelのどちらでもtrueになるが、CSV診断値は変えない。
  bool permission_start_exception_active{false};
  // 復帰ゲートが閉じている間はRECOVERY候補を中心線へ動かさず、現在横位置を保持する。
  bool reentry_hold_active{false};
  // NaNなら従来のreentry_hold_v_max_mpsを使う。MPC
  // solve遅延だけの短時間holdは、
  // SafetyEvaluatorを通した現d保持候補に限って別の低速capを指定する。
  double reentry_hold_speed_cap_mps{std::numeric_limits<double>::quiet_NaN()};
  // reentry gateを通過し、通常ラインへ向かうRECOVERYを継続中ならtrue。
  // ABORT holdとは分離するが、FASTEST/FOLLOW/PASSへの遷移許可ではない。
  bool reentry_centering_authorized{false};
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
  // curve例外を「候補feasible」の一語で開かないための段階診断。direct系は
  // freshな同一targetの認識、capacity/speed/curvatureは物理開始包絡、後段は
  // Gate 2認可入力をそれぞれ表す。
  bool gentle_curve_target_identity_valid{false};
  bool gentle_curve_direct_normal_target{false};
  bool gentle_curve_direct_braking_target{false};
  bool gentle_curve_fresh_dynamic_gap_target{false};
  bool gentle_curve_dynamic_target_speed_reachable{false};
  std::string gentle_curve_target_context_reason{};
  bool gentle_curve_context_clean{false};
  bool gentle_curve_lateral_capacity_sufficient{false};
  bool gentle_curve_dynamic_speed_cap_valid{false};
  bool gentle_curve_curvature_within_limit{false};
  bool gentle_curve_observation_inputs_complete{false};
  bool gentle_curve_prediction_complete{false};
  bool gentle_curve_tracking_usable{false};
  bool gentle_curve_mpc_ready{false};
  bool gentle_curve_safe_pass_found{false};
  CandidateType gentle_curve_safe_pass_side{CandidateType::FASTEST};
  double gentle_curve_safe_pass_cbf_slack{
      std::numeric_limits<double>::quiet_NaN()};
  std::string gentle_curve_safe_pass_block_reason{};
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
  // freshness/MPC
  // health条件を満たす周期だけ。falseならPASS候補も現速度で予測する。
  bool pass_acceleration_allowed{false};
  // V2 shadow proposalだけが使う仮想PASS加速の入力gate。実行candidateの
  // authorityには使わない。
  bool pass_proposal_acceleration_allowed{false};
  // 状態機械が同一target/sideについて蓄積した新規PASSの連続安全周期。
  // debug専用で、SafetyEvaluator結果の代わりには使わない。
  int pass_left_safe_cycles{0};
  int pass_right_safe_cycles{0};
  std::string pass_safe_cycle_reset_reason{};
  // 通常FOLLOWで実際にSafetyEvaluatorへ渡した速度契約。front速度だけでは
  // 攻めFOLLOW bonusが下流へ届いたか判別できないため、候補の先頭/終端を残す。
  double follow_candidate_speed_cap_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double follow_candidate_terminal_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  // PASS対象より前へ出る前に実横分離が安全楕円+目標余裕へ届いていない場合、
  // 相手速度を超えて縦gapを消費しないためのlateral-first速度契約。
  // 停止/極低速対象だけは横profileを実行するための設定済み低速creepを許すが、
  // そのs(t)も通常どおりSafetyEvaluatorへ渡す。
  bool pass_lateral_first_speed_gate_active{false};
  bool pass_lateral_clearance_ready{false};
  std::string pass_lateral_first_target_id{};
  double pass_lateral_first_target_relative_s_m{
      std::numeric_limits<double>::infinity()};
  double pass_lateral_separation_actual_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_lateral_separation_required_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_lateral_first_target_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_lateral_first_speed_cap_mps{
      std::numeric_limits<double>::quiet_NaN()};
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
  double decision_freeze_lateral_error_m{
      std::numeric_limits<double>::quiet_NaN()};
  std::string pass_decision_freeze_reason{};
  // PREPARE/OVERTAKE開始時に固定した対象の実観測契約。front/side分類器の
  // 代表indexとは分離し、同じIDをfreshな相手一覧から毎周期引き直す。
  bool maneuver_target_latched{false};
  std::string maneuver_target_id{};
  int maneuver_target_index{-1};
  bool maneuver_target_observed{false};
  bool maneuver_target_fresh{false};
  double maneuver_target_age_sec{std::numeric_limits<double>::infinity()};
  double maneuver_target_relative_s_m{std::numeric_limits<double>::infinity()};
  double maneuver_target_relative_d_m{std::numeric_limits<double>::quiet_NaN()};
  double maneuver_target_relative_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  bool maneuver_target_pass_geometric_complete{false};
  bool maneuver_target_pass_safety_approved{false};
  bool maneuver_target_pass_complete{false};
  // Gate 2認可済み・未実行の同一dynamic targetが、瞬時front分類だけを失った
  // 周期に限って同側PASS候補を再生成する診断。candidate、SafetyEvaluator、
  // permission、tracking、wall/CBFは現周期に再評価される。
  bool pass_start_target_continuity_active{false};
  int pass_start_target_continuity_cycles{0};
  int pass_start_target_continuity_budget_cycles{0};
  std::string pass_start_target_continuity_reason{};
  // bounded lifetimeを使い切った未実行profileを破棄した周期だけtrue。
  bool pass_start_target_continuity_expired{false};
  // start-gridの停止targetに対してGate 2を通過したが、同じPASS
  // generationのcontroller tracking proofをまだ確認していない状態。
  // 候補承認と実行commitを分離し、trueの間は停止constraint下のPREPAREとして
  // target/side/profileを保持する。
  bool maneuver_transaction_prepared{false};
  // target/sideを保持すべき未完了PASS transaction。front分類ではなく
  // LocalizedLateralProfileの完了契約をstate machineへ明示する。
  bool maneuver_transaction_incomplete{false};
  // このupdateへ入る前からGate 2認可済みだった未完了transaction。
  // 初回PASS開始と、一時FOLLOWから同じID/sideへ戻る再開を区別する。
  bool maneuver_transaction_retry_active{false};
  CandidateType maneuver_transaction_pass_type{CandidateType::FASTEST};
  // 未完了PASS中に通常のATTACK_FOLLOW横補正だけが不成立でも、同じ対象・
  // 同じ側を保つ現d YIELD/RECOVERY、またはtracking失効時の現d SAFE_STOPが
  // 当該周期のSafetyEvaluatorを通過した時だけtrue。target/sideを破棄する
  // ABORTと、安全評価済みの一時hold/停止を区別する契約である。
  bool maneuver_transaction_safe_lateral_hold_active{false};
  // Gate 2認可済みPASS包絡内へ到達済みで、同じtarget/sideを抜き切るまで
  // 現在dを保持する候補を評価すべき周期だけtrue。PASS候補そのものの
  // trackabilityを例外合格させず、別のRECOVERYを全相手へ再評価する契約。
  bool authorized_pass_current_d_hold_active{false};
  // 認可済みstart-grid PASSでfinal controller契約が失効し、現在dの
  // SafetyEvaluator済みSAFE_STOPを選んだ周期だけtrue。FOLLOW_BLOCKEDの
  // target/side保持とは独立に、SafetyConstraintAuthorityへstopを要求する。
  bool maneuver_transaction_tracking_stop_active{false};
  // start-gridのGate 2承認済みPASSを初回commitまたは再開する際、plannerの
  // 停止constraintを残したまま同じplan generationのPP command/proofを
  // 確認する契約。
  bool maneuver_transaction_tracking_release_pending{false};
  bool maneuver_transaction_tracking_release_confirmed{false};
  int maneuver_transaction_tracking_release_cycles{0};
  // PASS warm-upへ入る前に、同じtarget/sideのsnapshotが次のfresh観測でも
  // SafetyEvaluator・PP追従性を保つか確認する。probe中は安全なFOLLOWを続ける。
  int maneuver_transaction_tracking_probe_cycles{0};
  int maneuver_transaction_tracking_probe_required_cycles{0};
  std::string maneuver_transaction_tracking_probe_reset_reason{};
  bool maneuver_transaction_tracking_continuity_armed{false};
  // 認可済みPASS包絡から逸脱した状態でauthoritative planがstop/未認可に
  // なった後、同じPASSを一周期だけ再認可しないための回復契約。
  bool pass_reauthorization_lockout_active{false};
  double pass_authorized_envelope_min_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_authorized_envelope_max_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_authorized_envelope_error_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pass_reauthorization_recovery_target_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  int pass_reauthorization_clear_cycles{0};
  bool maneuver_target_safety_evaluated{false};
  bool maneuver_target_pass_candidate_feasible{false};
  double maneuver_target_pass_min_safety_margin{
      std::numeric_limits<double>::quiet_NaN()};
  std::string maneuver_target_pass_reject_reason{};
  std::string maneuver_target_previous_id{};
  std::string maneuver_target_new_id{};
  std::string maneuver_target_change_reason{};
  // Gate 2認可だけ成立し、実横移動が選択側へ始まる前に対象が前方へ
  // 離脱したtransactionを、安全な一周期handoffで解放した診断。
  bool maneuver_unstarted_target_released{false};
  // 解放した同じIDがinteraction外へ離れ続ける間、新規transactionとして
  // 即再ラッチしない。近い別ID、または同一IDが再びcatch可能になれば解除する。
  bool maneuver_unstarted_target_reacquire_suppressed{false};
  int maneuver_unstarted_target_pulling_away_cycles{0};
  double maneuver_pass_lateral_progress_m{
      std::numeric_limits<double>::quiet_NaN()};
  std::string maneuver_chain_tail_id{};
  int maneuver_chain_tail_index{-1};
  int maneuver_chain_target_count{0};
  bool maneuver_chain_tail_observed{false};
  double maneuver_chain_tail_relative_s_m{
      std::numeric_limits<double>::infinity()};
  double maneuver_chain_tail_relative_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
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

enum class StartGridProbeTransportEvidence : std::uint8_t {
  UNKNOWN = 0,
  EXACT_CURRENT = 1,
  NORMAL_DELIVERY_GAP = 2,
  STALE = 3,
  PAYLOAD_MUTATION = 4,
  REGRESSION_OR_GAP = 5,
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
  // final muxがPure Pursuitを選択し、fresh commandを実際に追従可能と
  // 型付きstatusで報告した時だけtrue。debug JSONは安全契約に使わない。
  bool pure_pursuit_primary_and_fresh{false};
  // Nodeが受信したControllerTrackingStatusの観測値。CoreはPASS開始がtracking
  // 不成立になった理由を出力するだけで、既存authority判定には用いない。
  bool controller_tracking_status_received{false};
  std::uint32_t controller_tracking_plan_generation{0U};
  std::uint32_t controller_tracking_expected_generation{0U};
  std::string controller_tracking_status_reason{"NOT_EVALUATED"};
  bool controller_tracking_mpc_horizon_usable{false};
  bool controller_tracking_continuity_usable{false};
  // Muxがplan/constraintのexact peer待ちを、高々1世代・fresh・非mutationの
  // 正常な配送順差として分類した時だけNORMAL_DELIVERY_GAPになる。
  // Coreは既取得PASS probeのcount凍結だけに使い、速度/PASS/warm-up/tokenの
  // authorityには使わない。UNKNOWNを含む他状態は従来どおりresetする。
  StartGridProbeTransportEvidence start_grid_pass_probe_transport_evidence{
      StartGridProbeTransportEvidence::UNKNOWN};
  // Muxが現在generationのSTOP/PASS_WARMUP full tupleとtyped tokenを
  // exact照合したdiagnostic ACK。motion authorityには使わず、同一probeの
  // countを進める入力だけに限定する。
  bool pass_probe_exact_current_usable{false};
  std::uint64_t pass_probe_lateral_stop_authority_token{0U};
  // 同じControllerTrackingStatusをPlanner timerが複数回読んでもprobeを
  // 二重に進めないためのexact sample identity。stampはMuxが実際に照合した
  // PP command stamp、generationはそのcommandを束縛したplan generation。
  double pass_probe_exact_sample_stamp_sec{
      std::numeric_limits<double>::quiet_NaN()};
  std::uint32_t pass_probe_exact_plan_generation{0U};
  // final muxが同一attempt/targetのATTACK_FOLLOWについて、exact current
  // plan/constraintと1世代前のfresh PP proofを専用契約で照合した時だけtrue。
  // PASS release・reentry・通常Gate 2には使わず、準備済みstart-grid PASSの
  // probeをtopic delivery順だけでresetしないために限定して使う。
  bool pure_pursuit_attack_follow_transport_usable{false};
  // final sourceが実際にPPで、fresh command/proofを返し、planner更新に対する
  // 非同期遅れが高々1 generationの時だけtrue。初回PASS認可には使わず、
  // 一度認可済みの連続transaction監視だけに使う。
  bool pure_pursuit_tracking_continuity_usable{false};
  // continuity proofが実際に追従したPASS generationのtarget/side。
  // plannerの現在transactionと一致しない古い世代は継続根拠に使わない。
  std::string pure_pursuit_tracking_target_id{};
  CandidateType pure_pursuit_tracking_pass_type{CandidateType::FASTEST};
  // final sourceがplanner自身の停止constraintでstop中でも、現在generationの
  // PP
  // command/proofが揃い、外部停止や現在進行中のwatchdog故障が無い時だけtrue。
  bool pure_pursuit_release_ready{false};
  // 上記の追従証明に加え、その制御周期でMPC horizonを適用していない時だけtrue。
  // 非PASS走行のMPC health切り分けと、認可済み同一PASSのcontinuityにだけ使う。
  // 初回PASS/reentry認可やSafetyEvaluatorの代用にはしない。
  bool verified_non_mpc_pure_pursuit{false};
  // Nodeが前周期に実publishしたlegacy authoritative plan/filtered
  // constraintのfeedback。Core内部の候補状態ではなく、muxへ渡った契約が
  // stop/未認可になった同一transactionだけをPASS再進入lockoutへ入れる。
  bool previous_authoritative_plan_feedback_valid{false};
  bool previous_authoritative_plan_stop_requested{false};
  bool previous_authoritative_plan_trajectory_authorized{false};
  std::string previous_authoritative_plan_target_id{};
  CandidateType previous_authoritative_plan_pass_type{CandidateType::FASTEST};
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

enum class TacticalPhase : std::uint8_t {
  FREE_RUN = 0,
  ATTACK_FOLLOW = 1,
  PASSING = 2,
  ABORT_HOLD = 3,
};

enum SupervisorV2AuthorizationFailure : std::uint32_t {
  SUPERVISOR_V2_AUTH_NONE = 0U,
  SUPERVISOR_V2_AUTH_CANDIDATE_MISSING = 1U << 0U,
  SUPERVISOR_V2_AUTH_CANDIDATE_NOT_SAFETY_EVALUATED = 1U << 1U,
  SUPERVISOR_V2_AUTH_CANDIDATE_REJECTED = 1U << 2U,
  SUPERVISOR_V2_AUTH_EGO_STALE = 1U << 3U,
  SUPERVISOR_V2_AUTH_V2X_STALE = 1U << 4U,
  SUPERVISOR_V2_AUTH_OPPONENT_STALE = 1U << 5U,
  SUPERVISOR_V2_AUTH_OPPONENT_EXCLUDED = 1U << 6U,
  SUPERVISOR_V2_AUTH_REFERENCE_INVALID = 1U << 7U,
  SUPERVISOR_V2_AUTH_PREDICTION_INCOMPLETE = 1U << 8U,
  SUPERVISOR_V2_AUTH_TRACKING_UNUSABLE = 1U << 9U,
  SUPERVISOR_V2_AUTH_MPC_HEALTH_STALE = 1U << 10U,
  SUPERVISOR_V2_AUTH_MPC_HARD_FAILURE = 1U << 11U,
  SUPERVISOR_V2_AUTH_TRAJECTORY_NOT_PUBLISHABLE = 1U << 12U,
  SUPERVISOR_V2_AUTH_CONSTRAINT_STOP_REQUESTED = 1U << 13U,
  SUPERVISOR_V2_AUTH_SAFETY_INPUTS_INCOMPLETE = 1U << 14U,
  SUPERVISOR_V2_AUTH_PROFILE_PROGRESS_INVALID = 1U << 15U,
};

struct SupervisorV2Decision {
  TacticalPhase phase{TacticalPhase::FREE_RUN};
  std::uint32_t plan_generation{0};
  std::uint64_t attempt_id{0};
  std::string target_vehicle_id{};
  int pass_direction{0};
  bool trajectory_authorized{false};
  bool safety_inputs_complete{false};
  bool tracking_usable{false};
  bool lateral_maneuver_required{false};
  std::uint32_t authorization_failure_mask{SUPERVISOR_V2_AUTH_NONE};
  std::string candidate_reject_reason{};
  bool candidate_set_limited_by_legacy{false};
  CandidateType selected{CandidateType::FASTEST};
  CandidateTrajectory trajectory{};
  SupervisorV2CandidateDiagnostic follow_candidate{};
  SupervisorV2CandidateDiagnostic pass_left_candidate{};
  SupervisorV2CandidateDiagnostic pass_right_candidate{};
  std::string reason{"shadow_disabled"};
};

struct PlannerConfig {
  // 追い越し候補生成、安全マージン、状態遷移をまとめて調整するパラメータ群。
  bool enabled{true};
  // C-002AZ-001: currentだけがlive backend。state_lattice_shadowは診断専用、
  // state_lattice_candidateは予約値でありlive candidate setへは入れない。
  std::string trajectory_backend{"current"};
  bool supervisor_v2_shadow_enabled{false};
  int supervisor_v2_abort_release_cycles{3};
  int supervisor_v2_pass_completion_cycles{2};
  int supervisor_v2_target_missing_hold_cycles{2};
  int supervisor_v2_tracking_unusable_hold_cycles{2};
  int start_grid_tracking_probe_required_cycles{2};
  int start_grid_tracking_release_timeout_cycles{60};
  double start_grid_tracking_continuity_max_mpc_solve_time_ms{120.0};
  std::size_t horizon_points{20};
  double horizon_dt_sec{0.025};
  double control_rate_hz{20.0};
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
  // ROS clockとsensor/V2X stampの小さな進みだけを許容する。stale側上限は
  // 各入力の既存timeoutから変えない。
  double input_future_stamp_tolerance_sec{0.05};
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
  // 停止parallel車のcurrent-section permission例外。early PASS
  // probeとは別opt-inで、 falseならCSVの追越禁止をそのまま維持する。
  bool early_stationary_parallel_permission_exception_enabled{false};
  double early_stationary_parallel_pass_distance_m{8.0};
  double early_stationary_parallel_pass_lateral_width_m{1.5};
  bool start_grid_target_enabled{false};
  double start_grid_target_window_sec{5.0};
  double start_grid_target_window_distance_m{8.0};
  double start_grid_target_max_ego_speed_mps{3.0};
  double start_grid_target_min_delta_s_m{-1.0};
  double start_grid_target_max_delta_s_m{8.0};
  double start_grid_target_lateral_width_m{1.5};
  double start_grid_stationary_confirmation_sec{1.0};
  double start_grid_stationary_confirmation_distance_m{3.0};
  double start_grid_attack_follow_v_max_mps{3.0};
  // Gate 2未認可のstart-grid追従で、初期anchorからこの範囲内の実測driftは
  // 方向を問わず現在d保持として再評価する。範囲を超える場合は、基準線が
  // 安全回廊内にある同側の内向きdriftだけを許可し、大横断と外向きdriftは
  // anchor補正の回廊・controller追従性検査へ戻す。
  double start_grid_uncommitted_hold_max_lateral_drift_m{0.15};
  // start-gridの初期dをFOLLOW/RECOVERYで保持する時、毎周期の現在dから
  // 通常merge距離(12 m)で張り直さず、この物理距離内でanchorへ戻す。
  // 実際の距離は操舵角/操舵速度制約を満たすまで自動的に延長する。
  double start_grid_hold_correction_distance_m{3.0};
  // まだ停止確認されていないgrid車に対し、現在dからこの距離を超える横断PASSを
  // 一気に開始しない。FOLLOWで縦位置を作り、通常候補が成立するまで待つ。
  double start_grid_moving_pass_max_lateral_displacement_m{2.2};
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
  // 遠方から早期評価する対象のFrenet進行速度上限[m/s]。通常速度の前走車まで
  // 60m探索へ広げず、停止・低速対象だけに限定する。
  double braking_follow_max_target_speed_mps{10.0};
  double braking_follow_trigger_margin_m{0.0};
  // 制動距離よりTTC包絡の方が長い場合に早期検出へ使う上限時間[s]。
  // 0以下なら制動距離だけを使う。
  double braking_follow_ttc_threshold_sec{0.0};
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
  // 長期reentry gate許可後の物理収束判定。rearm側を大きくして境界での
  // ABORT再進入チャタリングを防ぐ。
  double reentry_completion_lateral_error_m{0.20};
  double reentry_completion_rearm_lateral_error_m{0.30};
  double reentry_hold_v_max_mps{0.50};
  // MPC
  // solve遅延だけの時は、SafetyEvaluatorを通した現d保持に限りこのcapを使う。
  // stale/infeasible/CBF制約は常にreentry_hold_v_max_mps以下へ閉じる。
  double reentry_mpc_degraded_hold_v_max_mps{3.0};
  // reentry許可・中心収束後の高速カーブで使う現d保持の上限。ABORTを
  // 継続させず、SafetyEvaluatorを通したSPEED_GUARDへ分離する。
  double post_abort_curve_hold_v_max_mps{3.0};
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
  // PASS/攻めFOLLOWの横補正を、active primary PurePursuitと最終Muxの
  // 操舵角・操舵速度制約より急にしないための共有車両契約。
  double attack_follow_tracking_wheelbase_m{1.087};
  double attack_follow_max_steering_angle_rad{kVehicleHardSteeringTireAngleRad};
  // active primary PurePursuitを拘束する最終Muxの出力制限。
  // Builderはraw command領域へgain除算し、さらにreserveを適用する。
  double attack_follow_max_steering_rate_radps{kVehicleHardSteeringRateRadps};
  double attack_follow_steering_tire_angle_gain{1.639};
  double attack_follow_steering_rate_reserve_ratio{0.80};
  // PPが検証済み横profileとして採用する最短空間長と、それをSafetyEvaluator
  // でも同じ時刻列として評価するために許すATTACK_FOLLOW専用horizon上限。
  double attack_follow_min_spatial_horizon_m{0.55};
  double attack_follow_max_evaluation_horizon_sec{4.0};
  // active Pure Pursuitが横overrideを実行する時のlookahead契約。同じ値から
  // Planner側の最低検証arcを算出し、未評価の終端をPPへ渡さない。
  double lateral_override_lookahead_gain{0.5};
  double lateral_override_lookahead_min_distance_m{3.5};
  // moving/欠損相手を含む時はCartesian等速予測の誤差を長時間へ外挿しない。
  // 全観測相手がstationary閾値以下の時だけ下の長い上限を使える。
  double moving_lateral_override_max_evaluation_horizon_sec{6.0};
  double lateral_override_max_evaluation_horizon_sec{20.0};
  // Planner発行から下流controller実行までの加速分を、空間horizonの停止可能距離へ
  // 先回りして含める。PP側の実行時速度で検証arc不足へ落ちる瞬断を防ぐ。
  double lateral_override_execution_speed_reserve_sec{0.10};
  double maneuver_latch_min_hold_sec{1.0};
  double maneuver_latch_target_update_alpha{1.0};
  // 認可済みPASSでも、実車が選択側へまだ進んでおらず、fresh対象がこのgapを
  // 越えて指定相対速度以上で離れ続けた場合だけ、旧targetを解放して近い障害物を
  // 次周期に再評価する。横へコミット済みのPASSには適用しない。
  bool unstarted_pass_target_release_enabled{true};
  double unstarted_pass_target_release_min_gap_m{8.0};
  double unstarted_pass_target_release_min_opening_speed_mps{1.0};
  double unstarted_pass_target_release_max_lateral_progress_m{0.10};
  int unstarted_pass_target_release_required_cycles{5};
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
  // moving targetへのPASSは、横分離後にこの時間内でmerge前方余裕へ到達し、
  // 正の相対速度を連続確認できる候補だけを認可する。停止対象には適用しない。
  bool moving_pass_reachability_enabled{true};
  double moving_pass_max_completion_time_sec{20.0};
  double moving_pass_min_closing_speed_mps{0.05};
  // 前走車中心から必要楕円間隔よりさらに確保する横方向余裕。
  double pass_target_lateral_margin_m{0.10};
  // 停止/極低速PASS対象へ、横分離が成立するまで許す最大creep速度[m/s]。
  // 通常の相手速度追従とは分離し、1.0 m/s以下だけを有効設定として扱う。
  double pass_lateral_first_stationary_creep_v_max_mps{0.75};
  // commit済みPASSの固定空間profileに対する実横追従遅れを検出し、
  // 停止・低速対象へ横分離する間だけ縦速度を閉じる。
  double pass_lateral_tracking_lag_threshold_m{0.05};
  double pass_lateral_tracking_lag_speed_cap_mps{0.35};
  double max_overtake_v_bonus_mps{0.30};
  double recovery_v_max_mps{3.0};
  // Recovery再加速をSafetyEvaluatorのs(t)へ反映する保守的な最大加速度。
  double recovery_assumed_accel_mps2{3.0};
  double wall_margin_recovery_v_max_mps{0.5};
  double outside_corridor_recovery_centering_time_sec{1.0};
  double v_passthrough_mps{50.0};
  double d_min_m{-1.35};
  double d_max_m{1.35};
  double min_wall_margin_m{0.50};
  // corridor CSVは車体中心境界ではなく路面端を表す。中心点の従来marginに加え、
  // base_link基準の車体四隅を路面端の内側へ収める二重の独立条件を課す。
  bool wall_footprint_check_enabled{true};
  double ego_front_extent_m{1.554};
  double ego_rear_extent_m{0.510};
  double ego_half_width_m{0.650};
  // GNSS/EKF/delay補正の走行中差を、静的lever armへ誤って足さずに壁側へだけ
  // 有界に確保する初期値。実bagのplanner入力とcollision topicで再同定可能。
  double wall_localization_uncertainty_m{0.250};
  // horizon点間も車体cornerが壁を横切らないことを確認する補間上限。
  double wall_footprint_max_sample_distance_m{0.250};
  double wall_footprint_max_sample_yaw_rad{0.050};
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
  double preemptive_wall_recovery_min_curvature_m_inv{0.50};
  bool speed_only_fallback_enabled{true};
  double speed_only_fallback_v_max_mps{0.5};
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
  double wall_risk_v_max_mps{0.5};
  bool mpc_health_speed_guard_enabled{true};
  bool mpc_health_clean_free_run_soft_guard_bypass_enabled{false};
  // 非稼働MPCのhealth異常を、型付きstatusで非MPCと証明済みの通常PP走行から
  // 切り離す。障害物、壁、復帰、横maneuver、SAFE_STOP中は常に無効。
  bool mpc_health_inactive_mpc_free_run_bypass_enabled{false};
  int mpc_health_infeasible_count_threshold{1};
  double mpc_health_solve_time_warn_ms{80.0};
  double mpc_health_v_max_mps{0.5};
  double mpc_health_stale_time_sec{0.60};
  bool recovery_speed_guard_enabled{true};
  double recovery_speed_guard_v_max_mps{0.5};
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

// 入力stampの共通freshness契約。小さなfuture skewだけを許し、NaN/Inf、
// 大きなfuture、既存stale timeout超過はいずれもfail-closedにする。
inline bool inputTimestampFresh(double now_sec, double stamp_sec,
                                double stale_timeout_sec,
                                double future_tolerance_sec) {
  if (!std::isfinite(now_sec) || !std::isfinite(stamp_sec) ||
      !std::isfinite(stale_timeout_sec) || stale_timeout_sec <= 0.0 ||
      !std::isfinite(future_tolerance_sec) || future_tolerance_sec < 0.0) {
    return false;
  }
  const double age_sec = now_sec - stamp_sec;
  return age_sec >= -future_tolerance_sec && age_sec <= stale_timeout_sec;
}

// 入力: PASS方向、現在の自車d、対象車d、必要横離隔。
// 出力: CandidateBuilderとCoreの局所profileが共通で使うPASS目標d。
// 処理概要: minimum_clearanceでは余計に壁側へ寄せず、相手楕円間隔を満たす
// 最小横移動を選ぶ。対象が無い呼出しでは使わず、従来offsetは別途fallbackする。
inline double passTargetOffset(const PlannerConfig &config,
                               CandidateType pass_type, double ego_d_m,
                               double opponent_d_m, double required_gap_m) {
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
  // 1周期内の候補選択、状態機械、reentry仲裁、publish結果を分離した診断。
  // 最終modeだけでは、状態機械がABORTを選んだのか、後段gateが上書きした
  // のか判別できないため、実bagで責任境界を追える形にする。
  BehaviorMode transition_previous_mode{BehaviorMode::FREE_RUN};
  BehaviorMode state_machine_mode{BehaviorMode::FREE_RUN};
  BehaviorMode post_reentry_arbitration_mode{BehaviorMode::FREE_RUN};
  CandidateType raw_selected{CandidateType::FASTEST};
  bool raw_selected_feasible{false};
  std::string raw_selected_reject_reason{};
  bool reentry_phase_before_update{false};
  bool reentry_lockout_before_update{false};
  bool generic_recovery_before_update{false};
  bool reentry_phase_after_update{false};
  bool reentry_lockout_after_update{false};
  bool generic_recovery_after_update{false};
  std::vector<double> lateral_offsets;
  std::vector<double> speed_caps;
  // lateral_offsets/speed_capsと同じ時間horizon点が、先頭から何m先か。
  // controllerはこれを使って参照軌道の空間点へ再サンプルする。
  std::vector<double> longitudinal_offsets_m;
  BlockedInfo blocked_info{};
  // motion authorityと分離したC-002AI shadow診断。Node debugだけへ転記する。
  AttackFollowInnerBandDiagnostic attack_follow_inner_band_diagnostic{};
  // State Lattice geometryはこの比較record以外へ出さない。
  StateLatticeShadowComparison state_lattice_shadow_comparison{};
  std::string reason{};
  // 横軌道と縦速度capは別契約でpublishできる。active_overrideは横列が
  // SafetyEvaluatorを通った時だけtrueにし、横列を作れないfail-closed時も
  // longitudinal_speed_cap_activeで安全側の減速要求を下流へ届ける。
  bool active_override{false};
  bool longitudinal_speed_cap_active{false};
  // 選択横profileが、この周期のSafetyEvaluator、controller幾何制約、
  // longitudinal modelをすべて通過した証跡。mode名やfeasible既定値だけで
  // 停止中の横操舵を認可しないため、publish直前の契約へ明示的に運ぶ。
  bool selected_lateral_profile_safety_verified{false};
  // ego/V2X/referenceと全観測相手の包含・予測列が同周期で完全な時だけtrue。
  // current-d SAFE_STOPの横認可専用で、false時は縦停止だけへfail-closeする。
  bool lateral_stop_inputs_complete{false};
  // safe stop候補自体は不成立でも、同周期のSafetyEvaluatorとpublish直前再評価を
  // 通った必須RECOVERY、または完全入力で評価済みのcurrent-d SAFE_STOP横列
  // だけは、縦停止と直交して追従してよいことを示す。
  // stale入力、中心復帰を含むSAFE_STOP、未評価profileでは必ずfalseにする。
  bool lateral_tracking_authorized_during_stop{false};
  // CandidateBuilderの必要arcとSafetyEvaluator済みd/v/ds proofを、最終
  // publish payloadへ明示搬送する。default trueのtrackability
  // flagでは代用不可。
  double required_controller_spatial_horizon_m{
      std::numeric_limits<double>::quiet_NaN()};
  bool controller_spatial_horizon_proof_valid{false};
  // start-grid PASSを停止constraint下でPPへwarm-upしている時だけtrue。
  // tokenは同じ相対payloadを安全上の理由で再生成した場合もACK世代を
  // 取り直すためのNode内部契約で、ROS wireへ直接は載せない。
  bool tracking_release_pass_warmup{false};
  std::uint64_t tracking_release_token{0U};
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
  // decision freeze用の中心/PASS基準とは分離した、選択RECOVERY目標への横誤差。
  double recovery_tracking_error_m{std::numeric_limits<double>::quiet_NaN()};
  double recovery_tracking_target_d_m{std::numeric_limits<double>::quiet_NaN()};
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
  int maneuver_latch_waypoint_count{0};
  std::string maneuver_latch_last_waypoint_id{};
  double maneuver_latch_last_waypoint_s_m{
      std::numeric_limits<double>::quiet_NaN()};
  double maneuver_latch_last_waypoint_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  double applied_speed_cap_mps{std::numeric_limits<double>::quiet_NaN()};
  std::string speed_cap_reason{};
  double wall_soft_margin_m{std::numeric_limits<double>::quiet_NaN()};
  ActiveSectionSafety active_section{};
  MpcHealthStatus mpc_health{};
  SupervisorV2Decision supervisor_v2{};
};

inline bool enforceStopLateralSpatialHorizonContract(
    PlannerOutput &output, const CandidateTrajectory &candidate,
    bool authority_requested, double safe_stop_v_mps,
    std::size_t horizon_points) {
  const bool bound =
      output.lateral_offsets == candidate.d &&
      output.speed_caps == candidate.v_ref &&
      output.longitudinal_offsets_m == candidate.longitudinal_offsets_m &&
      output.required_controller_spatial_horizon_m ==
          candidate.required_controller_spatial_horizon_m;
  const bool valid = output.controller_spatial_horizon_proof_valid &&
                     candidate.controller_spatial_horizon_proof_valid &&
                     hasControllerSpatialHorizonProof(candidate) && bound;
  if (!authority_requested || valid) {
    return true;
  }
  const double cap_mps = std::max(1.0e-3, safe_stop_v_mps);
  output.active_override = false;
  output.selected = CandidateType::SAFE_STOP;
  output.selected_lateral_profile_safety_verified = false;
  output.controller_spatial_horizon_proof_valid = false;
  output.lateral_tracking_authorized_during_stop = false;
  output.solver_horizon_intent = PlannerOutput::SolverHorizonIntent::NONE;
  output.lateral_offsets.clear();
  output.longitudinal_offsets_m.clear();
  output.speed_caps.assign(std::max<std::size_t>(1U, horizon_points), cap_mps);
  output.longitudinal_speed_cap_active = true;
  output.speed_only_fallback_active = true;
  output.safe_stop_triggered = true;
  output.applied_speed_cap_mps = cap_mps;
  output.speed_cap_reason =
      "controller_spatial_horizon_unproven_speed_only_stop";
  output.reason = "controller_spatial_horizon_unproven_speed_only_stop";
  output.target_lateral_offset_m = 0.0;
  return false;
}

const char *toString(BehaviorMode mode);
const char *toString(CandidateType type);
bool isPassMode(BehaviorMode mode);

} // namespace overtake_planner
