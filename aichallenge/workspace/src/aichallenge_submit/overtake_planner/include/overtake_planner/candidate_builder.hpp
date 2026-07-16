#pragma once

#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/types.hpp"

#include <limits>
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
  struct CorridorPreflightResult {
    bool valid{false};
    double min_margin_m{std::numeric_limits<double>::quiet_NaN()};
  };

  struct LongitudinalProfile {
    bool valid{true};
    bool braking_requested{false};
    bool acceleration_requested{false};
    double initial_speed_mps{0.0};
    double target_speed_mps{0.0};
    double brake_decel_mps2{0.0};
    double accel_mps2{0.0};
    double response_delay_sec{0.0};
    double distanceAt(double t_sec) const;
    double speedAt(double t_sec) const;
    double requiredDistanceTo(double target_speed_mps) const;
  };

  LongitudinalProfile makeLongitudinalProfile(CandidateType type,
                                              const EgoState &ego,
                                              double speed_cap_mps,
                                              bool acceleration_allowed) const;
  double localizedProfileD(const LocalizedLateralProfile &profile,
                           const EgoState &ego, double s, double ds) const;
  double localizedProfileRawD(const LocalizedLateralProfile &profile,
                              const EgoState &ego, double s, double ds) const;
  double nominalLocalizedProfileD(const LocalizedLateralProfile &profile,
                                  double s) const;
  CorridorPreflightResult passTargetCorridorReachable(
      const EgoState &ego, double target_d, double shift_distance_m) const;
  CorridorPreflightResult localizedPassTargetCorridorReachable(
      const LocalizedLateralProfile &profile, const EgoState &ego) const;
  double wallClearance(double s, double d) const;

  const FrenetFrame &frame_;
  const PlannerConfig &config_;
};

} // namespace overtake_planner
