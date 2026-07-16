#pragma once

#include "overtake_planner/behavior_state_machine.hpp"
#include "overtake_planner/blocked_risk_analyzer.hpp"
#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/future_side_by_side_risk_analyzer.hpp"
#include "overtake_planner/safety_evaluator.hpp"
#include "overtake_planner/overtake_supervisor_v2.hpp"
#include "overtake_planner/types.hpp"

#include <limits>
#include <vector>

namespace overtake_planner {

class OvertakePlannerCore {
public:
  OvertakePlannerCore(FrenetFrame frame, PlannerConfig config);

  // 1制御周期の中核処理。障害判定、候補生成、安全評価、状態遷移をまとめて行う。
  PlannerOutput
  update(double now_sec, const EgoState &ego,
         const std::vector<OpponentState> &opponents,
         const MpcHealthStatus &mpc_health = MpcHealthStatus{},
         const ReentryInputStatus &reentry_input = ReentryInputStatus{});

  BehaviorMode mode() const { return mode_; }

private:
  // V2Xで受けた他車位置を短いhorizonだけ等速予測する。
  std::vector<PredictedOpponent>
  predictOpponents(const std::vector<OpponentState> &opponents, double now_sec,
                   const std::vector<double> *time_points = nullptr) const;
  // FASTEST/FOLLOW/PASS/RECOVERYそれぞれの横オフセット列と速度上限を作る。
  CandidateTrajectory
  makeCandidate(CandidateType type, const EgoState &ego,
                const BlockedInfo &blocked_info,
                const std::vector<OpponentState> &opponents) const;
  CandidateTrajectory makeReentryEvaluationCandidate(
      const EgoState &ego, const BlockedInfo &blocked_info,
      const std::vector<OpponentState> &opponents) const;
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
      double now_sec, const ReentryInputStatus &reentry_input,
      BlockedInfo &blocked,
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
  bool revalidatePublishedLateral(
      const PlannerOutput &output, const CandidateTrajectory &base_candidate,
      const std::vector<PredictedOpponent> &predictions) const;
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
                                const BlockedInfo &blocked,
                                const std::vector<OpponentState> &opponents);
  void clearLocalizedLateralProfile();
  void setLocalizedProfileMarkers(double target_s_m);
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

  FrenetFrame frame_;
  PlannerConfig config_;
  BlockedRiskAnalyzer blocked_risk_;
  FutureSideBySideRiskAnalyzer future_side_risk_;
  SafetyEvaluator safety_;
  BehaviorStateMachine state_machine_;
  OvertakeSupervisorV2 supervisor_v2_;
  BehaviorMode mode_{BehaviorMode::FREE_RUN};
  double first_valid_update_sec_{std::numeric_limits<double>::quiet_NaN()};
  double first_motion_update_sec_{std::numeric_limits<double>::quiet_NaN()};
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
  std::string early_stationary_parallel_pass_id_{};
  int early_stationary_parallel_pass_count_{0};
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
  bool high_speed_curve_lateral_hold_active_{false};
  std::vector<double> high_speed_curve_lateral_hold_offsets_;
  double high_speed_curve_lateral_hold_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  // 再合流を安全に完了した後だけ使う高速カーブの現d保持。ABORT中のPASSを
  // そのまま再利用せず、解除後に通常状態機械で改めて評価させる。
  bool post_abort_curve_hold_active_{false};
};

} // namespace overtake_planner
