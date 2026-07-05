#pragma once

#include "overtake_planner/behavior_state_machine.hpp"
#include "overtake_planner/blocked_risk_analyzer.hpp"
#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/future_side_by_side_risk_analyzer.hpp"
#include "overtake_planner/safety_evaluator.hpp"
#include "overtake_planner/types.hpp"

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
  bool sectionContainsS(const SectionSafetyRule &rule, double s) const;
  double effectiveWallSoftMargin(const ActiveSectionSafety &section) const;
  // 横並びで相手が縦方向に前へ出ている場合は、無理に並走せず後ろへ譲る。
  bool shouldYieldBehindSideBySide(const EgoState &ego,
                                   const BlockedInfo &blocked_info) const;

  FrenetFrame frame_;
  PlannerConfig config_;
  BlockedRiskAnalyzer blocked_risk_;
  FutureSideBySideRiskAnalyzer future_side_risk_;
  SafetyEvaluator safety_;
  BehaviorStateMachine state_machine_;
  BehaviorMode mode_{BehaviorMode::FREE_RUN};
  double overtake_start_sec_{0.0};
  int safe_stop_trigger_count_{0};
  bool straight_overtake_start_allowed_{true};
};

} // namespace overtake_planner
