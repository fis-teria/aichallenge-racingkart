#pragma once

#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/types.hpp"

#include <vector>

namespace overtake_planner {

class CandidateBuilder {
public:
  CandidateBuilder(const FrenetFrame &frame, const PlannerConfig &config);

  CandidateTrajectory
  makeCandidate(CandidateType type, const EgoState &ego,
                const BlockedInfo &blocked_info,
                const std::vector<OpponentState> &opponents,
                const LocalizedLateralProfile *localized_profile = nullptr) const;

private:
  struct LongitudinalProfile {
    bool valid{true};
    bool braking_requested{false};
    double initial_speed_mps{0.0};
    double target_speed_mps{0.0};
    double brake_decel_mps2{0.0};
    double response_delay_sec{0.0};
    double distanceAt(double t_sec) const;
    double speedAt(double t_sec) const;
    double requiredDistanceTo(double target_speed_mps) const;
  };

  LongitudinalProfile makeLongitudinalProfile(CandidateType type,
                                              const EgoState &ego,
                                              double speed_cap_mps) const;
  double localizedProfileD(const LocalizedLateralProfile &profile,
                           const EgoState &ego, double s, double ds) const;
  double nominalLocalizedProfileD(const LocalizedLateralProfile &profile,
                                  double s) const;
  double wallClearance(double d) const;

  const FrenetFrame &frame_;
  const PlannerConfig &config_;
};

} // namespace overtake_planner
