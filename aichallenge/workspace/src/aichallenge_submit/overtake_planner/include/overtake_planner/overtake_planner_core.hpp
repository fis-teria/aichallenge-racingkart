#pragma once

#include "overtake_planner/behavior_state_machine.hpp"
#include "overtake_planner/blocked_risk_analyzer.hpp"
#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/future_side_by_side_risk_analyzer.hpp"
#include "overtake_planner/overtake_supervisor_v2.hpp"
#include "overtake_planner/safety_evaluator.hpp"
#include "overtake_planner/types.hpp"

#include <limits>
#include <optional>
#include <vector>

namespace overtake_planner {

struct PurePursuitExactSnapshot;

namespace detail {
// publish直前のPlannerOutputが、縦stop中にも許可できる最新current-d holdかを
// 判定する。公開planner APIではなく、安全契約の単体回帰テスト用detail関数。
bool isSafetyEvaluatedCurrentLateralHoldDuringStop(const PlannerOutput &output);
// 未完了PASS transactionを保持するFOLLOW/RECOVERY/SAFE_STOPのうち、
// 同周期の完全入力とSafetyEvaluatorで検証したcurrent-d一定列だけを
// planner自身の縦stop中にもPPで追従できるか判定する。
bool isSafetyEvaluatedCurrentTransactionHoldDuringStop(
    const PlannerOutput &output);
// 通常SAFE_STOPでも、完全入力から同周期評価した厳密なcurrent-d保持だけを
// 縦stopと合成できるか判定する。中心復帰、古いprofile、入力欠損は拒否する。
bool isSafetyEvaluatedCurrentSafeStopLateralHold(const PlannerOutput &output);
} // namespace detail

class OvertakePlannerCore {
public:
  OvertakePlannerCore(FrenetFrame frame, PlannerConfig config);

  // 1制御周期の中核処理。障害判定、候補生成、安全評価、状態遷移をまとめて行う。
  PlannerOutput
  update(double now_sec, const EgoState &ego,
         const std::vector<OpponentState> &opponents,
         const MpcHealthStatus &mpc_health = MpcHealthStatus{},
         const ReentryInputStatus &reentry_input = ReentryInputStatus{},
         const PurePursuitExactSnapshot *pp_exact_snapshot = nullptr);

  BehaviorMode mode() const { return mode_; }

  // C-002AZ-002: shadow-only candidate evaluation facade.  This intentionally
  // reuses the exact own-time-axis safety path used by current candidates and
  // does not select, publish, or authorize the supplied candidate.
  bool evaluateShadowCandidate(
      CandidateTrajectory & candidate,
      const std::vector<OpponentState> & opponents,
      double now_sec) const;
  bool evaluateShadowCandidate(
      CandidateTrajectory &candidate,
      const std::vector<OpponentState> &opponents, double now_sec,
      const std::vector<PredictedOpponent> &predictions) const;

private:
  struct SupervisorV2CenteringAssessment {
    CandidateTrajectory candidate{};
    bool safe{false};
  };

  // V2Xで受けた他車位置を短いhorizonだけ等速予測する。
  std::vector<PredictedOpponent>
  predictOpponents(const std::vector<OpponentState> &opponents, double now_sec,
                   const std::vector<double> *time_points = nullptr) const;
  // 候補固有の時刻列と完全一致する相手予測でSafetyEvaluatorを実行する。
  // 通常horizonは既存予測を再利用し、延長horizonだけ再予測する。
  bool evaluateCandidateAtOwnTimeAxis(
      CandidateTrajectory &candidate,
      const std::vector<OpponentState> &opponents, double now_sec,
      const std::vector<PredictedOpponent> &default_predictions) const;
  StateLatticeShadowComparison evaluateStateLatticeShadowComparison(
      const CandidateTrajectory &current_candidate, const EgoState &ego,
      const std::string &target_id,
      const std::vector<OpponentState> &opponents, double now_sec,
      const std::vector<PredictedOpponent> &predictions,
      bool snapshot_complete,
      const PurePursuitExactSnapshot *pp_exact_snapshot) const;
  void evaluateAttackFollowInnerBandDiagnostic(
      const CandidateTrajectory &source,
      const CandidateTrajectory &current_d_hold, const EgoState &ego,
      const std::vector<OpponentState> &opponents, double now_sec,
      const std::vector<PredictedOpponent> &default_predictions);
  // FASTEST/FOLLOW/PASS/RECOVERYそれぞれの横オフセット列と速度上限を作る。
  CandidateTrajectory
  makeCandidate(CandidateType type, const EgoState &ego,
                const BlockedInfo &blocked_info,
                const std::vector<OpponentState> &opponents) const;
  CandidateTrajectory makeReentryEvaluationCandidate(
      const EgoState &ego, const BlockedInfo &blocked_info,
      const std::vector<OpponentState> &opponents) const;
  SupervisorV2CenteringAssessment
  assessSupervisorV2Centering(double now_sec, const EgoState &ego,
                              const BlockedInfo &blocked_info,
                              const std::vector<OpponentState> &opponents,
                              const ReentryInputStatus &reentry_input) const;
  ReentryGateResult
  evaluateReentryGate(double now_sec, const EgoState &ego,
                      const BlockedInfo &blocked_info,
                      const std::vector<OpponentState> &opponents,
                      const MpcHealthStatus &mpc_health,
                      const ReentryInputStatus &reentry_input,
                      ReentryMpcHealthState reentry_mpc_health);
  ReentryMpcHealthState
  updateReentryMpcHealthState(const ReentryInputStatus &reentry_input);
  void updateReentryPhase(const EgoState &ego, BehaviorMode previous_mode);
  bool reentryRequested(const EgoState &ego) const;
  bool candidateMovesTowardCenter(const EgoState &ego,
                                  const CandidateTrajectory &candidate) const;
  void applyGenericRecoveryStaleHold(PlannerOutput &output) const;
  void rememberGenericRecoveryHold(const CandidateTrajectory &candidate,
                                   const ReentryGateResult &gate);
  void rememberGenericRecoveryHold(const PlannerOutput &output);
  // 安全で目的に合う候補を、スコアが最小のものとして選ぶ。
  CandidateTrajectory
  selectCandidate(std::vector<CandidateTrajectory> &candidates) const;
  // 不可候補を大きく罰し、追い越し/追従/復帰の優先度を数値化する。
  double candidateScore(const CandidateTrajectory &candidate,
                        const BlockedInfo &blocked_info) const;
  // 横並び中に近い先の曲率を見て、カーブで横へ押し出す判断を抑える。
  double maxAbsCurvatureAhead(double s, double lookahead_m) const;
  ActiveSectionSafety activeSectionSafety(double s) const;
  ActiveOvertakePermission activeOvertakePermission(double s) const;
  ActiveOvertakePermission overtakePermissionAtS(double s) const;
  bool sectionContainsS(const SectionSafetyRule &rule, double s) const;
  bool permissionRuleContainsS(const OvertakePermissionRule &rule,
                               double s) const;
  double effectiveWallSoftMargin(const ActiveSectionSafety &section) const;
  bool
  promoteSlowObstacleChain(BlockedInfo &blocked,
                           const std::vector<OpponentState> &opponents) const;
  void classifyEarlyStationaryParallelPassTarget(
      double now_sec, const EgoState &ego,
      const ReentryInputStatus &reentry_input, BlockedInfo &blocked,
      const std::vector<OpponentState> &opponents);
  void classifyParallelFollowCandidate(
      double now_sec, const EgoState &ego, BlockedInfo &blocked,
      const std::vector<OpponentState> &opponents) const;
  void classifyStationaryFrontObstacle(
      double now_sec, const EgoState &ego, BlockedInfo &blocked,
      const std::vector<OpponentState> &opponents) const;
  void classifyBrakingFollowTarget(
      double now_sec, const EgoState &ego,
      const ReentryInputStatus &reentry_input, BlockedInfo &blocked,
      const std::vector<OpponentState> &opponents) const;
  void classifyEarlyLowSpeedPassTarget(
      double now_sec, const EgoState &ego,
      const ReentryInputStatus &reentry_input, BlockedInfo &blocked,
      const std::vector<OpponentState> &opponents) const;
  bool revalidatePublishedLateral(
      const PlannerOutput &output, const CandidateTrajectory &base_candidate,
      const EgoState &ego, const std::vector<OpponentState> &opponents,
      double now_sec,
      const std::vector<PredictedOpponent> &default_predictions) const;
  bool updateSlowFrontException(const BlockedInfo &blocked);
  bool shouldSuppressSafeStopForStartGrace(double now_sec, const EgoState &ego,
                                           const BlockedInfo &blocked) const;
  bool leaderPriorityCandidate(const BlockedInfo &blocked, double margin_m,
                               std::string &target_id, double &target_delta_s,
                               std::string &reason) const;
  void updateLeaderPriority(double now_sec, BlockedInfo &blocked);
  bool localizedLateralProfileEnabled() const;
  CandidateType preferredPassType(const BlockedInfo &blocked) const;
  int localizedProfileTargetIndex(const BlockedInfo &blocked) const;
  void
  updateLocalizedLateralProfile(double now_sec, const EgoState &ego,
                                BlockedInfo &blocked,
                                const std::vector<OpponentState> &opponents);
  void updatePassLateralFirstSpeedGate(
      double now_sec, const EgoState &ego, BlockedInfo &blocked,
      const std::vector<OpponentState> &opponents) const;
  std::optional<double>
  sampleCommittedPassSpatialProfileD(double offset_m) const;
  void clearLocalizedLateralProfile();
  void setLocalizedProfileMarkers(double target_s_m);
  void extendLocalizedProfileForSlowObstacleChain(
      double now_sec, const EgoState &ego, const OpponentState &target,
      const std::vector<OpponentState> &opponents);
  double targetOffsetForPass(CandidateType pass_type, double ego_d_m,
                             double opponent_d_m = 0.0) const;
  // 横並びで相手が縦方向に前へ出ている場合は、無理に並走せず後ろへ譲る。
  bool shouldYieldBehindSideBySide(const EgoState &ego,
                                   const BlockedInfo &blocked_info) const;
  void applyHighSpeedCurveLateralHold(double now_sec, const EgoState &ego,
                                      PlannerOutput &output);
  void applyLateralTargetRateLimit(double now_sec, PlannerOutput &output);
  void rememberPublishedLateralTarget(double now_sec,
                                      const PlannerOutput &output);
  void rememberAuthorizedPassEnvelope(const CandidateTrajectory &candidate,
                                      const LocalizedLateralProfile &profile);
  double authorizedPassEnvelopeErrorM(double ego_d_m) const;

  FrenetFrame frame_;
  PlannerConfig config_;
  BlockedRiskAnalyzer blocked_risk_;
  FutureSideBySideRiskAnalyzer future_side_risk_;
  SafetyEvaluator safety_;
  BehaviorStateMachine state_machine_;
  OvertakeSupervisorV2 supervisor_v2_;
  std::uint64_t supervisor_v2_cycle_sequence_{0U};
  // V2 ATTACK_FOLLOWはPASS profile未commitでもtarget identityを保持する。
  // wrapped signedDeltaSで半周折返ししないよう、同一fresh IDの周期進捗を
  // entityごとに積算し、PASSINGへ入る前の完了判定にも使う。
  std::string supervisor_v2_progress_target_id_{};
  double supervisor_v2_progress_ego_unwrapped_s_m_{
      std::numeric_limits<double>::quiet_NaN()};
  double supervisor_v2_progress_last_ego_wrapped_s_m_{
      std::numeric_limits<double>::quiet_NaN()};
  double supervisor_v2_progress_last_ego_stamp_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  double supervisor_v2_progress_target_unwrapped_s_m_{
      std::numeric_limits<double>::quiet_NaN()};
  double supervisor_v2_progress_last_target_wrapped_s_m_{
      std::numeric_limits<double>::quiet_NaN()};
  double supervisor_v2_progress_last_target_stamp_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  BehaviorMode mode_{BehaviorMode::FREE_RUN};
  double first_valid_update_sec_{std::numeric_limits<double>::quiet_NaN()};
  double first_motion_update_sec_{std::numeric_limits<double>::quiet_NaN()};
  double first_valid_update_s_m_{std::numeric_limits<double>::quiet_NaN()};
  int safe_stop_trigger_count_{0};
  int reentry_clear_cycles_{0};
  bool reentry_lockout_active_{false};
  bool reentry_phase_active_{false};
  // trueになっても実車が通常ラインへ収束するまではphaseを残す。次の復帰を
  // 未評価のまま許すフラグではなく、同じ復帰軌道を継続評価済みである記録。
  bool reentry_gate_permitted_{false};
  // FREE_RUN/FOLLOW由来のRECOVERYも中心側へ横断する時だけ同じ長期安全評価に
  // 載せる。ただし拒否時はABORTではなくSPEED_GUARDの現d保持を続ける。
  bool generic_recovery_phase_active_{false};
  std::vector<double> generic_recovery_hold_offsets_;
  double generic_recovery_hold_speed_cap_mps_{
      std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t last_reentry_mpc_health_sample_sequence_{0U};
  int reentry_mpc_bad_sample_count_{0};
  int reentry_mpc_good_sample_count_{0};
  ReentryMpcHealthState reentry_mpc_health_state_{
      ReentryMpcHealthState::HEALTHY};
  int slow_front_exception_count_{0};
  std::string slow_front_exception_id_{};
  // Gate 2認可後も横移動を開始せず前方へ離れたtargetを解放した時、同じ
  // interaction外IDを次周期に即再ラッチしない。profile本体とは別寿命で保持し、
  // 別target選択または同一targetがcatch可能になった時だけ解除する。
  std::string unstarted_pass_released_target_id_{};
  std::string early_stationary_parallel_pass_id_{};
  int early_stationary_parallel_pass_count_{0};
  std::string start_grid_target_id_{};
  // 未認可grid対象を実際のblocked frontへ引き継いだ後は、同一arm中に
  // start-grid専用分類へ戻さない。通常のblocked/PASS評価だけを継続する。
  bool start_grid_target_reselection_suppressed_{false};
  // 未認可start-grid対象をblocked frontへ引き継いだ世代のtarget ID。
  // current-d FOLLOWの限定例外を、引継ぎ先と同一のfresh profileだけへ束縛する。
  std::string start_grid_handoff_target_id_{};
  double start_grid_target_first_seen_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  double start_grid_target_first_seen_s_m_{
      std::numeric_limits<double>::quiet_NaN()};
  // 「最初に見えてから」の経過時間ではなく、同じtargetが連続して停止して
  // いる区間だけを数える。移動後の一瞬の低速観測を停止確定にしない。
  double start_grid_target_stationary_since_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  double start_grid_target_stationary_since_s_m_{
      std::numeric_limits<double>::quiet_NaN()};
  // start-grid FOLLOWの横目標を毎周期の実dへ張り替えると、MPC追従誤差を
  // 積算して並走車側へ徐々に寄るため、最初の横位置をanchorに保持する。
  // release_pendingは初期配置ではなくGate 2認可後にだけ立て、対象解除後は
  // reentry gateを通してから中心へ戻す。
  bool start_grid_lateral_release_pending_{false};
  double start_grid_lateral_anchor_d_m_{
      std::numeric_limits<double>::quiet_NaN()};
  // plannerのstop constraint中にPPをwarm-upし、同じPASS generationの
  // 追従証明を二回確認してから解除する。
  bool start_grid_tracking_release_pending_{false};
  bool start_grid_tracking_release_confirmed_{false};
  int start_grid_tracking_release_cycles_{0};
  int pass_probe_exact_ack_cycles_{0};
  double pass_probe_last_exact_sample_stamp_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  std::uint32_t pass_probe_last_exact_plan_generation_{0U};
  std::string start_grid_tracking_release_target_id_{};
  bool start_grid_tracking_release_candidate_valid_{false};
  CandidateTrajectory start_grid_tracking_release_candidate_{};
  // 未完了PASSが一度tracking STOPへ閉じた後は、start-grid以外でも同じ
  // target/sideのPASSを無証明で直接再開しない。このidentity latchを保持し、
  // STOP下で新しいPASS generation/tokenのPP proofを確認した後だけ解除する。
  bool maneuver_execution_hold_active_{false};
  std::string maneuver_execution_hold_target_id_{};
  CandidateType maneuver_execution_hold_pass_type_{CandidateType::FASTEST};
  // 1周期だけ成立するPASSを即warm-upへ昇格させない。前周期snapshotを
  // fresh観測へ張り直して再評価し、連続成立した場合だけrelease
  // tokenを発行する。
  int start_grid_tracking_probe_cycles_{0};
  std::string start_grid_tracking_probe_target_id_{};
  CandidateType start_grid_tracking_probe_pass_type_{CandidateType::FASTEST};
  double start_grid_tracking_probe_target_d_m_{
      std::numeric_limits<double>::quiet_NaN()};
  double start_grid_tracking_probe_last_target_stamp_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  bool start_grid_tracking_probe_candidate_valid_{false};
  CandidateTrajectory start_grid_tracking_probe_candidate_{};
  // warm-up中に追従不能になった同一wireを次周期に再投入しない。raw PASS評価は
  // 続け、target/sideまたはwireが変化した時だけfresh probeから再開する。
  bool start_grid_tracking_failed_candidate_valid_{false};
  std::string start_grid_tracking_failed_target_id_{};
  CandidateType start_grid_tracking_failed_pass_type_{CandidateType::FASTEST};
  CandidateTrajectory start_grid_tracking_failed_candidate_{};
  // 同じwireでも現在yawが変わればPP trackabilityは変わる。失敗時姿勢を
  // identityへ含め、姿勢復旧後のfresh probeを旧失敗payloadで抑止しない。
  double start_grid_tracking_failed_ego_yaw_rad_{
      std::numeric_limits<double>::quiet_NaN()};
  // 実際にpublishされ、publish後の追従性と現時刻SafetyEvaluatorを通った
  // commit済みPASSの横形状。有限prefixの先は同じtarget/side/chain profileで
  // 補完し、次周期のcurrent-d再アンカーで横移動を圧縮しない。
  bool committed_pass_snapshot_valid_{false};
  // true以降は通常走行中のrolling raw候補で形状を上書きしない。Safety proof
  // ではなく、最初にpublishしたtransaction空間形状を保持するauthorityである。
  // tracking STOP後の新token warm-upだけは、停止下で再評価したwireへ更新する。
  bool committed_pass_spatial_profile_frozen_{false};
  std::string committed_pass_snapshot_target_id_{};
  CandidateType committed_pass_snapshot_pass_type_{CandidateType::FASTEST};
  double committed_pass_snapshot_sec_{std::numeric_limits<double>::quiet_NaN()};
  double committed_pass_snapshot_ego_unwrapped_s_m_{
      std::numeric_limits<double>::quiet_NaN()};
  CandidateTrajectory committed_pass_snapshot_{};
  // 有限horizonのpublish wireを越えた先も、初回認可時と同じchain ID/d
  // 契約で補完するためのidentity snapshot。相手のfreshな前進に伴うs marker
  // 更新だけはlive profileから使い、ID/side/dの差し替えは許可しない。
  LocalizedLateralProfile committed_pass_profile_snapshot_{};
  // active tokenは現在warm-up中のPASS snapshotを識別する。counterはresetで
  // 戻さず、同じFloat32 payloadを再生成しても旧ACKを再利用させない。
  std::uint64_t start_grid_tracking_release_token_counter_{0U};
  std::uint64_t start_grid_tracking_release_token_{0U};
  // PREPARE中に別IDや入力欠損でpermission例外を引き継がないための一周期ラッチ。
  std::string stationary_parallel_permission_prepare_id_{};
  bool leader_priority_hold_active_{false};
  std::string leader_priority_hold_id_{};
  double leader_priority_hold_until_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  bool straight_overtake_start_allowed_{true};
  // 一度安全コリドー外へ出た車両は、内側へ戻る途中だけsoft wallでも
  // RECOVERYを継続する。通常のsoft guardを横overrideへ昇格させないための文脈。
  bool wall_recovery_latched_{false};
  // gentle curve safe PASSを開始した後に、次周期の通常PASS候補へ戻って
  // 速度・横移動上限が外れないよう、PREPARE/OVERTAKE中だけ制限をラッチする。
  bool gentle_curve_safe_pass_constraint_latched_{false};
  double gentle_curve_safe_pass_anchor_d_m_{
      std::numeric_limits<double>::quiet_NaN()};
  // 初回の制限PASSで決めた速度上限は、PREPARE/OVERTAKE中に緩めない。
  // 以後の曲率が高くなった場合だけ、より低い上限へ更新する。
  double gentle_curve_safe_pass_speed_cap_mps_{
      std::numeric_limits<double>::quiet_NaN()};
  // 停止障害物の禁止区間PASSは、PREPARE/OVERTAKE中も開始時の横移動・速度
  // 制約を緩めない。対象が消えるかGate 2を失えば直ちに解除する。
  bool stationary_no_pass_safe_pass_constraint_latched_{false};
  std::string stationary_no_pass_safe_pass_target_id_{};
  double stationary_no_pass_safe_pass_anchor_d_m_{
      std::numeric_limits<double>::quiet_NaN()};
  double stationary_no_pass_safe_pass_speed_cap_mps_{
      std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> last_published_lateral_offsets_;
  double last_published_lateral_target_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  LocalizedLateralProfile localized_lateral_profile_{};
  AttackFollowInnerBandDiagnostic attack_follow_inner_band_diagnostic_{};
  double last_attack_follow_inner_band_probe_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  bool high_speed_curve_lateral_hold_active_{false};
  std::vector<double> high_speed_curve_lateral_hold_offsets_;
  double high_speed_curve_lateral_hold_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  // 再合流を安全に完了した後だけ使う高速カーブの現d保持。ABORT中のPASSを
  // そのまま再利用せず、解除後に通常状態機械で改めて評価させる。
  bool post_abort_curve_hold_active_{false};
  // Gate 2通過時の実candidate/slow-chain waypointを含む横包絡。FSM mode名に
  // 依存せず、同一transactionのSTOP/未認可後だけPASS再進入を閉じる。
  bool pass_reauthorization_lockout_active_{false};
  std::string pass_reauthorization_lockout_target_id_{};
  CandidateType pass_reauthorization_lockout_pass_type_{CandidateType::FASTEST};
  double authorized_pass_envelope_min_d_m_{
      std::numeric_limits<double>::quiet_NaN()};
  double authorized_pass_envelope_max_d_m_{
      std::numeric_limits<double>::quiet_NaN()};
  std::string authorized_pass_envelope_target_id_{};
  CandidateType authorized_pass_envelope_pass_type_{CandidateType::FASTEST};
  int pass_reauthorization_clear_cycles_{0};
};

} // namespace overtake_planner
