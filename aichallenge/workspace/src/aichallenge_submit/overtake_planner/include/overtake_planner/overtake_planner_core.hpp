#pragma once

#include "overtake_planner/behavior_state_machine.hpp"
#include "overtake_planner/blocked_risk_analyzer.hpp"
#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/future_side_by_side_risk_analyzer.hpp"
#include "overtake_planner/safety_evaluator.hpp"
#include "overtake_planner/types.hpp"

#include <limits>
#include <vector>

namespace overtake_planner {

class OvertakePlannerCore {
public:
  OvertakePlannerCore(FrenetFrame frame, PlannerConfig config);

  // 1制御周期の中核処理。障害判定、候補生成、安全評価、状態遷移をまとめて行う。
  PlannerOutput update(double now_sec, const EgoState &ego,
                       const std::vector<OpponentState> &opponents,
                       const MpcHealthStatus &mpc_health = MpcHealthStatus{});

  BehaviorMode mode() const { return mode_; }

private:
  // V2Xで受けた他車位置を短いhorizonだけ等速予測する。
  std::vector<PredictedOpponent>
  predictOpponents(const std::vector<OpponentState> &opponents,
                   double now_sec) const;
  // FASTEST/FOLLOW/PASS/RECOVERYそれぞれの横オフセット列と速度上限を作る。
  CandidateTrajectory
  makeCandidate(CandidateType type, const EgoState &ego,
                const BlockedInfo &blocked_info,
                const std::vector<OpponentState> &opponents) const;
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
  bool updateSlowFrontException(const BlockedInfo &blocked);
  bool shouldSuppressSafeStopForStartGrace(double now_sec,
                                           const EgoState &ego,
                                           const BlockedInfo &blocked) const;
  bool localizedLateralProfileEnabled() const;
  CandidateType preferredPassType(const BlockedInfo &blocked) const;
  int localizedProfileTargetIndex(const BlockedInfo &blocked) const;
  void updateLocalizedLateralProfile(double now_sec, const EgoState &ego,
                                     const BlockedInfo &blocked,
                                     const std::vector<OpponentState> &opponents);
  void clearLocalizedLateralProfile();
  void setLocalizedProfileMarkers(double target_s_m);
  double targetOffsetForPass(CandidateType pass_type) const;
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
  BehaviorMode mode_{BehaviorMode::FREE_RUN};
  double first_valid_update_sec_{std::numeric_limits<double>::quiet_NaN()};
  int safe_stop_trigger_count_{0};
  int slow_front_exception_count_{0};
  bool straight_overtake_start_allowed_{true};
  std::vector<double> last_published_lateral_offsets_;
  double last_published_lateral_target_sec_{
      std::numeric_limits<double>::quiet_NaN()};
  LocalizedLateralProfile localized_lateral_profile_{};
  bool high_speed_curve_lateral_hold_active_{false};
  std::vector<double> high_speed_curve_lateral_hold_offsets_;
  double high_speed_curve_lateral_hold_sec_{
      std::numeric_limits<double>::quiet_NaN()};
};

} // namespace overtake_planner
