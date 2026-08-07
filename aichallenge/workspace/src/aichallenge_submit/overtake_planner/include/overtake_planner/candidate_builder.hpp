#pragma once

#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/types.hpp"

#include <functional>
#include <limits>
#include <vector>

namespace overtake_planner {

struct PassTransitionFeasibility {
  bool reachable{true};
  bool input_valid{false};
  double lateral_shift_m{std::numeric_limits<double>::quiet_NaN()};
  double evaluated_tracking_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double required_transition_m{std::numeric_limits<double>::infinity()};
  double available_deadline_m{std::numeric_limits<double>::infinity()};
};

// Candidate generation is shared by execution and non-authoritative safety
// proposals.  Only the latter may model a gated PASS acceleration.
enum class CandidatePurpose {
  EXECUTION,
  PROPOSAL_SAFETY_EVALUATION,
};

class CandidateBuilder {
public:
  CandidateBuilder(const FrenetFrame &frame, const PlannerConfig &config);

  CandidateTrajectory
  makeCandidate(CandidateType type, const EgoState &ego,
                const BlockedInfo &blocked_info,
                const std::vector<OpponentState> &opponents,
                const LocalizedLateralProfile *localized_profile = nullptr,
                bool force_localized_pass_profile = false,
                CandidatePurpose purpose = CandidatePurpose::EXECUTION) const;
  bool
  centeringProfileMatchesNominal(const EgoState &ego,
                                 const BlockedInfo &blocked_info,
                                 const CandidateTrajectory &candidate) const;
  bool publishedLateralProfileTrackable(const CandidateTrajectory &candidate,
                                        const EgoState &ego) const;
  // opponent_collisionになったcommit済みATTACK_FOLLOWから、時刻・縦位置・
  // 速度列を一切変えず、実測current-dを保持する独立候補を生成する。
  // 戻り値は未安全評価であり、呼び出し側が全相手SafetyEvaluatorを再実行する。
  CandidateTrajectory
  makeAttackFollowCurrentDHoldVariant(const CandidateTrajectory &source,
                                      const EgoState &ego) const;
  // C-002AIのshadow診断専用。sourceの縦profile/transaction目標は保持し、
  // 実測current-dから指定terminal dまで滑らかにつないだ未安全評価候補を返す。
  // 実行候補identity flagは立てず、呼び出し側が全相手を再評価する。
  CandidateTrajectory makeAttackFollowInnerBandDiagnosticVariant(
      const CandidateTrajectory &source, const EgoState &ego,
      double terminal_d_m,
      double shift_distance_m = std::numeric_limits<double>::quiet_NaN()) const;
  // C-002AJ実行用。shadow reportは使わず、sourceの縦profileを保って
  // 実測current-dから指定terminal dへ滑らかに接続する。戻り値は
  // 未SafetyEvaluator評価で、実行identityはCoreの全再評価後だけ立つ。
  CandidateTrajectory makeAttackFollowInwardConnectorVariant(
      const CandidateTrajectory &source, const EgoState &ego,
      double terminal_d_m,
      double shift_distance_m = std::numeric_limits<double>::quiet_NaN()) const;
  double minimumTrackableLateralShiftDistance(
      const EgoState &ego, double target_d, double speed_cap_mps,
      const BlockedInfo &blocked_info, double requested_distance_m) const;
  PassTransitionFeasibility passTransitionFeasibility(
      const EgoState &ego, double target_d, double speed_cap_mps,
      const BlockedInfo &blocked_info, double available_deadline_m) const;
  // active PPが横authorityを受けるために必要な空間horizon。候補生成と
  // Coreの早期PASS admissionが同じ式を使い、required arcを短縮しない。
  double requiredControllerSpatialHorizon(
      double execution_speed_mps, double controller_target_speed_mps,
      bool attack_follow_profile = false) const;
  // commit済みPASSの有限publish wireより先を、同じlatched transactionの
  // 段階profileから補完するための純粋な空間sampling。実測d connectorや
  // corridor clampは呼び出し側で最終候補へ適用し、SafetyEvaluatorへ通す。
  double
  nominalLocalizedProfileDAtUnwrappedS(const LocalizedLateralProfile &profile,
                                       double unwrapped_s_m) const;

private:
  struct LateralTrackabilityResult {
    bool desired_path_trackable{false};
    bool pure_pursuit_command_trackable{false};

    bool valid() const {
      return desired_path_trackable && pure_pursuit_command_trackable;
    }
  };

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

  LongitudinalProfile
  makeLongitudinalProfile(CandidateType type, const EgoState &ego,
                          double speed_cap_mps, bool acceleration_allowed,
                          bool controller_spatial_profile) const;
  double localizedProfileD(const LocalizedLateralProfile &profile,
                           const EgoState &ego, double s, double ds) const;
  double localizedProfileRawD(const LocalizedLateralProfile &profile,
                              const EgoState &ego, double s, double ds) const;
  double nominalLocalizedProfileD(const LocalizedLateralProfile &profile,
                                  double s) const;
  CorridorPreflightResult
  passTargetCorridorReachable(const EgoState &ego, double target_d,
                              double shift_distance_m) const;
  CorridorPreflightResult
  attackFollowTargetCorridorReachable(const EgoState &ego, double target_d,
                                      double shift_distance_m) const;
  CorridorPreflightResult
  localizedPassTargetCorridorReachable(const LocalizedLateralProfile &profile,
                                       const EgoState &ego) const;
  double recoveryShiftDistanceM(const EgoState &ego,
                                const BlockedInfo &blocked_info) const;
  double attackFollowShiftDistanceM(const EgoState &ego, double target_d,
                                    double speed_cap_mps,
                                    const BlockedInfo &blocked_info) const;
  double trackingLimitedLateralShiftDistanceM(
      const EgoState &ego, double target_d, double speed_cap_mps,
      const BlockedInfo &blocked_info, double requested_distance_m) const;
  bool nominalLateralCorrectionPathTrackable(const EgoState &ego,
                                             double target_d,
                                             double shift_distance_m,
                                             double evaluated_speed_mps) const;
  LateralTrackabilityResult
  lateralCorrectionProfileTrackability(const CandidateTrajectory &candidate,
                                       const EgoState &ego, double target_d,
                                       double shift_distance_m) const;
  LateralTrackabilityResult
  localizedPassProfileTrackability(const LocalizedLateralProfile &profile,
                                   const CandidateTrajectory &candidate,
                                   const EgoState &ego) const;
  LateralTrackabilityResult sampledLateralProfileTrackability(
      const CandidateTrajectory &candidate, const EgoState &ego,
      double check_distance_m,
      const std::function<double(double)> &sample_d_at_distance) const;
  double wallClearance(double s, double d) const;

  const FrenetFrame &frame_;
  const PlannerConfig &config_;
};

} // namespace overtake_planner
