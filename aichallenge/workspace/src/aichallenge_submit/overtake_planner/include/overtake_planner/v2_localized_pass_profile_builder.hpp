#pragma once

#include "overtake_planner/blocked_risk_analyzer.hpp"
#include "overtake_planner/candidate_builder.hpp"
#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/types.hpp"

#include <optional>
#include <vector>

namespace overtake_planner {

// V2専用localized PASS profileを純生成し、commit後はidentity/side/dを固定した
// ままego/targetの縦進捗だけを更新する。legacy FSM/profileの寿命やside選択を
// 参照せず、同じ物理型とCandidateBuilderだけを共有する。
class V2LocalizedPassProfileBuilder {
public:
  V2LocalizedPassProfileBuilder(const FrenetFrame &frame,
                                const PlannerConfig &config);

  std::optional<LocalizedLateralProfile>
  build(double now_sec, const EgoState &ego, const OpponentState &target,
        const std::vector<OpponentState> &opponents,
        CandidateType pass_type, const BlockedInfo &blocked_info) const;

  bool advanceLongitudinalProgress(
      double now_sec, const EgoState &ego,
      const std::vector<OpponentState> &opponents,
      LocalizedLateralProfile &profile) const;

private:
  double targetOffset(CandidateType pass_type, double ego_d_m,
                      double opponent_d_m) const;
  void setMarkers(LocalizedLateralProfile &profile,
                  double target_s_m) const;
  void extendSlowObstacleChain(
      double now_sec, const OpponentState &target,
      const std::vector<OpponentState> &opponents,
      LocalizedLateralProfile &profile) const;

  const FrenetFrame &frame_;
  const PlannerConfig &config_;
  BlockedRiskAnalyzer blocked_risk_;
  CandidateBuilder candidate_builder_;
};

} // namespace overtake_planner
