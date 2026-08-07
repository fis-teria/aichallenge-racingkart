#pragma once

#include "state_lattice_overtake_planner/speed_transition_evidence.hpp"

#include "state_lattice_overtake_planner/config.hpp"
#include "state_lattice_overtake_planner/frenet_frame.hpp"
#include "state_lattice_overtake_planner/grid_map.hpp"
#include "state_lattice_overtake_planner/types.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace state_lattice_overtake_planner {

double requiredLateralTransitionDistance(const PlannerConfig &config,
                                         double speed_mps, double current_d_m,
                                         double goal_d_m);
double derivedFrontDetectionRadius(const PlannerConfig &config,
                                   const OpponentState &opponent);

class ParametricQuintic {
public:
  bool configure(const Pose2d &start, double start_curvature,
                 const Pose2d &goal, double goal_curvature,
                 double tangent_scale);
  TrajectoryPoint sample(double u) const;
  bool valid() const { return valid_; }

private:
  struct Polynomial {
    std::array<double, 6> a{};
  };
  static Polynomial solve(double p0, double v0, double a0, double p1, double v1,
                          double a1);
  static void evaluate(const Polynomial &polynomial, double u, double *p,
                       double *dp, double *ddp);
  Polynomial x_;
  Polynomial y_;
  bool valid_{false};
};

class FrontDetector {
public:
  explicit FrontDetector(const PlannerConfig &config) : config_(config) {}
  std::optional<std::string> update(const EgoState &ego,
                                    const std::vector<OpponentState> &opponents,
                                    const FrenetFrame &frame);
  void requestInitialSweep() { initial_sweep_ = true; }
  void reset() {
    previous_terminals_.clear();
    current_terminals_.clear();
    sweep_starts_.clear();
    initial_sweep_ = true;
    detected_ = false;
    enter_cycles_ = 0;
    clear_cycles_ = 0;
    detection_radius_m_ = config_.front_detection_radius_m;
    maximum_transition_distance_m_ = 0.0;
    diagnostic_ = FrontDetectionDiagnostic{};
  }
  bool detected() const { return detected_; }
  int clearCycles() const { return clear_cycles_; }
  const std::vector<Pose2d> &currentTerminals() const {
    return current_terminals_;
  }
  const std::vector<Pose2d> &sweepStarts() const { return sweep_starts_; }
  double detectionRadiusM() const { return detection_radius_m_; }
  double maximumTransitionDistanceM() const {
    return maximum_transition_distance_m_;
  }
  const FrontDetectionDiagnostic &diagnostic() const { return diagnostic_; }

private:
  PlannerConfig config_;
  std::vector<Pose2d> previous_terminals_;
  std::vector<Pose2d> current_terminals_;
  std::vector<Pose2d> sweep_starts_;
  bool initial_sweep_{true};
  bool detected_{false};
  int enter_cycles_{0};
  int clear_cycles_{0};
  double detection_radius_m_{0.0};
  double maximum_transition_distance_m_{0.0};
  FrontDetectionDiagnostic diagnostic_;
};

class EarlyAwareSelector {
public:
  explicit EarlyAwareSelector(const PlannerConfig &config) : config_(config) {}
  void update(const EgoState &ego, const std::vector<OpponentState> &opponents,
              const FrenetFrame &frame, bool inputs_fresh, double now_sec);
  void reset();
  const EarlyAwareDiagnostic &diagnostic() const { return diagnostic_; }

private:
  void clear(const std::string &reason);

  PlannerConfig config_;
  std::string target_id_;
  double last_ego_stamp_sec_{-1.0};
  double last_target_stamp_sec_{-1.0};
  int distinct_fresh_stamp_count_{0};
  EarlyAwareDiagnostic diagnostic_;
};

class LatticePlanner {
public:
  LatticePlanner(PlannerConfig config, const FrenetFrame *frame,
                 const GridMap *map, const FrenetFrame *output_frame = nullptr);
  PlannerOutput update(const EgoState &ego,
                       const std::vector<OpponentState> &opponents,
                       bool inputs_fresh, double now_sec,
                       const MpcHealthStatus &mpc_health = MpcHealthStatus{},
                       TrialSpeedEvidence *speed_evidence = nullptr);
  FrontDetector &detector() { return detector_; }
  const EarlyAwareSelector &earlyAwareSelector() const {
    return early_aware_selector_;
  }
  const std::vector<CandidateTrajectory> &candidates() const {
    return candidates_;
  }
  const std::optional<CandidateTrajectory> &selected() const {
    return selected_;
  }
  const std::vector<TrajectoryPoint> &rearSafetyPath() const {
    return rear_path_;
  }
  const PlanningCycleMetrics &lastPlanningCycleMetrics() const {
    return last_planning_cycle_metrics_;
  }
  bool active() const { return active_; }
  const std::string &targetId() const { return target_id_; }
  // Clear only pass-continuation identity. Node-level fail-closed paths that
  // bypass update() must not reset unrelated maneuver or detector state.
  void clearPassContinuationLatch();
  void resetManeuverState();

  std::vector<CandidateTrajectory>
  generateCandidates(const EgoState &ego,
                     const std::vector<OpponentState> &opponents) const;
  std::vector<TrajectoryPoint> generateRearSafetyPath(const EgoState &ego,
                                                      double goal_d_m) const;
  bool evaluateTrajectory(CandidateTrajectory *candidate,
                          const std::vector<OpponentState> &opponents,
                          double initial_speed_mps = 0.0) const;
  bool validateOutputHorizon(const EgoState &ego,
                             const std::vector<OpponentState> &opponents,
                             const std::vector<double> &d,
                             const std::vector<double> &speed,
                             const std::vector<double> &longitudinal_offsets_m,
                             double now_sec) const;
  bool validateSideRolePeerSeparation(
      const EgoState &ego, const OpponentState &peer,
      const std::vector<TrajectoryPoint> &trajectory,
      PreventiveSideRoleCandidateDiagnostic *diagnostic = nullptr) const;
  bool validateSideRoleOutputHorizon(
      const EgoState &ego, const OpponentState &peer,
      const std::vector<OpponentState> &all_opponents,
      const std::vector<double> &d, const std::vector<double> &speed,
      const std::vector<double> &longitudinal_offsets_m,
      OutputHorizonDiagnostic *output_diagnostic = nullptr,
      PreventiveSideRoleCandidateDiagnostic *role_diagnostic = nullptr) const;

private:
#ifdef STATE_LATTICE_TEST_ACCESS
  friend struct StateLatticeTestAccess;
#endif
  enum class PreventiveSideRoleKind {
    NONE = 0,
    LEADER = 1,
    FOLLOWER = 2,
    NEUTRAL = 3,
  };

  enum class PreventiveSideRoleDecisionStatus {
    NONE = 0,
    ACTIVE = 1,
    RELEASED = 2,
    AMBIGUOUS = 3,
  };

  struct PreventiveSideRoleDecision {
    PreventiveSideRoleDecisionStatus status{
        PreventiveSideRoleDecisionStatus::NONE};
    PreventiveSideRoleKind role{PreventiveSideRoleKind::NONE};
    std::size_t peer_index{0U};
    double clearance_m{std::numeric_limits<double>::infinity()};
    double required_gap_m{std::numeric_limits<double>::quiet_NaN()};
    std::string reason;
    PreventiveSideRoleEligibilityDiagnostic diagnostic;
  };

  struct PreventiveSideRoleLatch {
    std::string peer_id;
    PreventiveSideRoleKind role{PreventiveSideRoleKind::NONE};
    PreventiveSideRolePhase phase{PreventiveSideRolePhase::NONE};
    double last_observation_stamp_sec{-1.0};
    double previous_peer_x_m{std::numeric_limits<double>::quiet_NaN()};
    double previous_peer_y_m{std::numeric_limits<double>::quiet_NaN()};
    PreventiveSideRoleKind pending_role{PreventiveSideRoleKind::NONE};
    int role_confirmation_count{0};
    std::uint64_t generation{0U};
    double decision_deadline_sec{std::numeric_limits<double>::quiet_NaN()};
    double first_yield_observation_stamp_sec{-1.0};
    int yield_sample_count{0};
    int exit_sample_count{0};
  };

  struct PassContinuationLatch {
    std::string target_id;
    int side{0};
    std::size_t lateral_index{0U};
    std::size_t tangent_index{0U};
    double goal_d_m{0.0};
    double required_arc_m{0.0};
    double target_observation_stamp_sec{0.0};
    std::uint64_t revision{0U};
  };

  struct RoleCandidateEvaluation {
    bool safe{false};
    std::vector<double> lateral_offsets_m;
    std::vector<double> speed_caps_mps;
    std::vector<double> longitudinal_offsets_m;
    OutputHorizonDiagnostic output_diagnostic;
    PreventiveSideRoleCandidateDiagnostic role_diagnostic;
  };

  std::vector<TrajectoryPoint>
  denseSamples(const ParametricQuintic &polynomial) const;
  std::vector<CandidateTrajectory>
  generateCandidatesInternal(const EgoState &ego,
                             const std::vector<OpponentState> &opponents,
                             const PassContinuationLatch *continuation) const;
  std::optional<CandidateTrajectory> movingTargetFollowCandidate(
      const EgoState &ego, const OpponentState &target,
      const std::vector<OpponentState> &opponents) const;
  std::optional<CandidateTrajectory>
  roleCorridorCandidate(const EgoState &ego, double goal_d_m,
                        const std::vector<OpponentState> &opponents) const;
  PreventiveSideRoleDecision detectPreventiveSideRole(
      const EgoState &ego, const std::vector<OpponentState> &observed_opponents,
      const std::vector<OpponentState> &planning_opponents, double now_sec);
  RoleCandidateEvaluation
  evaluateRoleCandidate(const EgoState &ego, const OpponentState &peer,
                        const std::vector<OpponentState> &all_opponents,
                        const CandidateTrajectory &candidate,
                        double target_speed_mps,
                        std::uint64_t generation) const;
  struct SideRoleSeparationReference {
    const OpponentState *peer{nullptr};
    double initial_lateral_gap_m{std::numeric_limits<double>::quiet_NaN()};
    double initial_clearance_m{std::numeric_limits<double>::quiet_NaN()};
    PreventiveSideRoleCandidateDiagnostic *diagnostic{nullptr};
    PreventiveSideRoleCandidateFailure failure{
        PreventiveSideRoleCandidateFailure::DENSE_PEER_SEPARATION};
  };
  bool
  sideRolePeerSeparationAtPoint(const TrajectoryPoint &point,
                                const SideRoleSeparationReference &reference,
                                int sample_index,
                                bool interpolated_sample) const;
  struct PoseCostBreakdown {
    int total_cost{0};
    int reference_cost{0};
    int wall_cost{0};
    int object_cost{0};
    int nominal_wall_level{-1};
    bool nominal_wall_clearance_proxy_valid{false};
    double nominal_wall_clearance_proxy_m{
        std::numeric_limits<double>::quiet_NaN()};
    bool representative_object_winner_valid{false};
    bool representative_object_prediction_valid{false};
    int representative_object_level{-1};
    double representative_object_clearance_m{
        std::numeric_limits<double>::quiet_NaN()};
    const OpponentState *representative_object_opponent{nullptr};
  };
  PoseCostBreakdown
  poseCostBreakdown(const TrajectoryPoint &pose,
                    const std::vector<OpponentState> &opponents) const;
  int poseCost(const TrajectoryPoint &pose,
               const std::vector<OpponentState> &opponents) const;
  bool hardCollision(const TrajectoryPoint &pose,
                     const std::vector<OpponentState> &opponents) const;
  bool opponentCollision(const TrajectoryPoint &pose,
                         const std::vector<OpponentState> &opponents) const;
  CollisionDiagnostic opponentCollisionDiagnostic(
      const TrajectoryPoint &pose,
      const std::vector<OpponentState> &opponents) const;
  CurrentPoseOpponentRelation
  classifyCurrentPoseOpponentRelation(const EgoState &ego,
                                      const OpponentState &opponent) const;
  bool rearOnlyCurrentPoseExemptionProvenSafe(
      const EgoState &ego, const OpponentState &predicted_opponent,
      const OpponentState &observed_opponent, double now_sec) const;
  bool currentPoseRearOnlyExemptionApplies(const TrajectoryPoint &pose,
                                           const OpponentState &opponent) const;
  bool outputHorizon(const EgoState &ego, const CandidateTrajectory &candidate,
                     const std::vector<OpponentState> &opponents,
                     double target_speed_mps, std::vector<double> *d,
                     std::vector<double> *speed,
                     std::vector<double> *longitudinal_offsets_m,
                     OutputHorizonDiagnostic *diagnostic) const;
  bool validateResampledHorizon(
      const EgoState &ego, const std::vector<OpponentState> &opponents,
      const std::vector<double> &d, const std::vector<double> &speed,
      const std::vector<double> &longitudinal_offsets_m,
      OutputHorizonDiagnostic *diagnostic,
      const SideRoleSeparationReference *side_role_reference = nullptr) const;
  int rearCost(const std::vector<TrajectoryPoint> &path,
               const std::vector<OpponentState> &opponents,
               bool *hard_safe) const;
  const OpponentState *
  target(const std::vector<OpponentState> &opponents) const;
  std::vector<OpponentState>
  extrapolateOpponents(const std::vector<OpponentState> &opponents,
                       double now_sec) const;
  bool permissionRuleContainsS(const OvertakePermissionRule &rule,
                               double s) const;
  ActiveOvertakePermission overtakePermissionAtS(double s) const;
  ActiveOvertakePermission activeOvertakePermission(double s) const;
  bool updateMpcHealthGuard(const MpcHealthStatus &health, std::string *reason);
  void applyMpcHealthGuard(PlannerOutput *output,
                           const std::string &reason) const;
  void populatePassContinuationDiagnostic(PlannerOutput *output,
                                          bool active) const;

  PlannerConfig config_;
  const FrenetFrame *frame_{nullptr};
  // The controller consumes d samples relative to this reference. A null
  // value preserves the legacy single-reference constructor behavior.
  const FrenetFrame *output_frame_{nullptr};
  const GridMap *map_{nullptr};
  FrontDetector detector_;
  EarlyAwareSelector early_aware_selector_;
  bool active_{false};
  bool safe_stop_latched_{false};
  int safe_stop_release_count_{0};
  int return_ready_count_{0};
  std::string target_id_;
  int previous_lateral_index_{-1};
  int previous_tangent_index_{-1};
  double last_mode_change_sec_{-1.0};
  BehaviorMode last_mode_{BehaviorMode::FREE_RUN};
  bool speed_state_initialized_{false};
  double previous_command_speed_mps_{0.0};
  double previous_acceleration_mps2_{0.0};
  std::vector<CandidateTrajectory> candidates_;
  std::optional<CandidateTrajectory> selected_;
  std::vector<TrajectoryPoint> rear_path_;
  std::optional<OpponentState> last_target_state_;
  double last_target_observed_sec_{-1.0};
  double target_missing_since_sec_{-1.0};
  bool mpc_health_guard_latched_{false};
  int mpc_health_release_count_{0};
  std::uint64_t last_mpc_health_sequence_{0U};
  std::string mpc_health_guard_reason_;
  std::optional<PassContinuationLatch> pass_continuation_latch_;
  std::optional<PassContinuationLatch> suspended_pass_continuation_latch_;
  std::optional<PreventiveSideRoleLatch> preventive_side_role_latch_;
  std::uint64_t preventive_side_role_generation_{0U};
  // Set only while update() evaluates a proven-safe rear-only current pose.
  // It permits that exact t=0 sample, never a future or interpolated sample.
  std::vector<std::string> current_pose_rear_only_exempt_opponent_ids_;
  PlanningCycleMetrics last_planning_cycle_metrics_;
};

} // namespace state_lattice_overtake_planner
