#pragma once

#include "overtake_planner/types.hpp"

#include <string>
#include <vector>

namespace overtake_planner {

struct SupervisorV2Input {
  bool target_present{false};
  std::string target_vehicle_id{};
  bool safety_inputs_complete{false};
  bool tracking_usable{false};
  bool pass_start_allowed{false};
  bool abort_release_allowed{false};
  bool candidate_set_limited_by_legacy{false};
  const std::vector<CandidateTrajectory> *candidates{nullptr};
  const CandidateTrajectory *abort_hold_candidate{nullptr};
};

class OvertakeSupervisorV2 {
public:
  explicit OvertakeSupervisorV2(int abort_release_cycles = 3);

  SupervisorV2Decision update(const SupervisorV2Input &input);
  TacticalPhase phase() const { return phase_; }

private:
  const CandidateTrajectory *findCandidate(const SupervisorV2Input &input,
                                           CandidateType type) const;
  const CandidateTrajectory *bestPass(const SupervisorV2Input &input) const;
  const CandidateTrajectory *bestFallback(const SupervisorV2Input &input) const;
  SupervisorV2Decision makeDecision(const SupervisorV2Input &input,
                                    TacticalPhase phase,
                                    const CandidateTrajectory *candidate,
                                    const std::string &reason);
  void updateGeneration(SupervisorV2Decision &decision);

  int abort_release_cycles_{3};
  int abort_clear_cycles_{0};
  TacticalPhase phase_{TacticalPhase::FREE_RUN};
  std::string target_vehicle_id_{};
  CandidateType pass_type_{CandidateType::FASTEST};
  std::uint64_t attempt_id_{0};
  std::uint32_t plan_generation_{0};
  SupervisorV2Decision last_decision_{};
  bool has_last_decision_{false};
};

} // namespace overtake_planner
