#pragma once

#include "overtake_planner/types.hpp"

#include <optional>
#include <string>
#include <vector>

namespace overtake_planner {

// Gate 2で評価した軌道と、その軌道を生成したV2専用localized profileを
// 取り違えずにSupervisorへ渡す。両者とtarget IDが一致したprobeだけが
// PASS開始を認可できる。
struct SupervisorV2PassProbe {
  const CandidateTrajectory *trajectory{nullptr};
  const LocalizedLateralProfile *profile{nullptr};
  std::string target_vehicle_id{};
};

struct SupervisorV2Input {
  // Core updateの連番。early returnや入力欠損でV2評価周期が飛んだ場合に、
  // ABORTの連続safe cycleを引き継がないために使う。0は単体テスト用の未指定値。
  std::uint64_t cycle_sequence{0U};
  bool target_present{false};
  std::string target_vehicle_id{};
  bool safety_inputs_complete{false};
  bool tracking_usable{false};
  bool pass_left_start_allowed{false};
  bool pass_right_start_allowed{false};
  // V2がPASSINGまたはATTACK_FOLLOWで保持中のtarget IDを幾何学的に
  // 抜き切った同周期だけtrue。
  // committed profileに別のchain車両が含まれていても、target identityと
  // chain
  // tailを混同しない。classifier上の次target出現だけではhandoffを許可しない。
  bool current_target_pass_complete{false};
  // PASS完了前の固定targetがこの周期だけfresh観測できない場合。安全入力は
  // incompleteへ閉じ、bounded hold中もtarget/side/profileは破棄しない。
  bool latched_target_temporarily_unavailable{false};
  bool abort_centering_safe{false};
  bool abort_centered{false};
  bool abort_release_allowed{false};
  bool candidate_set_limited_by_legacy{false};
  std::uint32_t authorization_failure_mask{SUPERVISOR_V2_AUTH_NONE};
  const std::vector<CandidateTrajectory> *candidates{nullptr};
  const CandidateTrajectory *abort_hold_candidate{nullptr};
  const CandidateTrajectory *abort_centering_candidate{nullptr};
  const CandidateTrajectory *abort_stop_candidate{nullptr};
  SupervisorV2PassProbe pass_left_probe{};
  SupervisorV2PassProbe pass_right_probe{};
};

class OvertakeSupervisorV2 {
public:
  explicit OvertakeSupervisorV2(int abort_release_cycles = 3,
                                int abort_centering_safe_cycles = 5,
                                int pass_completion_cycles = 2,
                                int target_missing_hold_cycles = 2,
                                int tracking_unusable_hold_cycles = 2);

  SupervisorV2Decision update(const SupervisorV2Input &input);
  TacticalPhase phase() const { return phase_; }
  const std::string &targetVehicleId() const { return target_vehicle_id_; }
  CandidateType passType() const { return pass_type_; }
  bool passCompletionPending() const { return pass_complete_pending_; }
  bool generationExhausted() const { return generation_exhausted_; }
  const LocalizedLateralProfile *passProfile() const {
    return pass_profile_.has_value() ? &*pass_profile_ : nullptr;
  }
  // Coreがfresh観測から進めた縦markerだけをatomicに反映する。target/side/d、
  // chain identity、認可フラグが変化したcopyは拒否して既存snapshotを保持する。
  bool applyPassProfileProgress(const LocalizedLateralProfile &updated_profile);

private:
  const CandidateTrajectory *findCandidate(const SupervisorV2Input &input,
                                           CandidateType type) const;
  const SupervisorV2PassProbe *
  bestPassProbe(const SupervisorV2Input &input) const;
  const CandidateTrajectory *
  bestAbortFallback(const SupervisorV2Input &input) const;
  SupervisorV2Decision makeDecision(const SupervisorV2Input &input,
                                    TacticalPhase phase,
                                    const CandidateTrajectory *candidate,
                                    const std::string &reason);
  void updateGeneration(SupervisorV2Decision &decision);

  int abort_release_cycles_{3};
  int abort_centering_safe_cycles_{5};
  int pass_completion_cycles_{2};
  int pass_completion_clear_cycles_{0};
  int target_missing_hold_cycles_{2};
  int tracking_unusable_hold_cycles_{2};
  // target欠測とcontroller trackingの一時不成立を交互に繰り返して
  // transactionを無期限保持しないよう、両方で共有する連続hold回数。
  int temporary_hold_cycles_{0};
  int abort_centering_clear_cycles_{0};
  int abort_clear_cycles_{0};
  std::uint64_t last_cycle_sequence_{0U};
  bool has_last_cycle_sequence_{false};
  bool abort_centering_active_{false};
  bool pass_complete_pending_{false};
  TacticalPhase phase_{TacticalPhase::FREE_RUN};
  std::string target_vehicle_id_{};
  CandidateType pass_type_{CandidateType::FASTEST};
  // Gate 2で選んだexact profile snapshot。Core側のlegacy profileや毎周期の
  // provisional profileから独立させ、target/side/attemptと同じownerで保持する。
  std::optional<LocalizedLateralProfile> pass_profile_{};
  std::uint64_t attempt_id_{0};
  std::uint32_t plan_generation_{0};
  bool generation_exhausted_{false};
  SupervisorV2Decision last_decision_{};
  bool has_last_decision_{false};
};

// V2 shadow decisionをSafetyConstraintへ変換する際の停止契約。
// 評価済みSAFE_STOPはtrajectoryとして有効でも、停止要求を必ず伴う。
bool supervisorV2TrajectoryPublishable(const SupervisorV2Decision &decision);
bool supervisorV2EffectivelyAuthorized(const SupervisorV2Decision &decision);
bool supervisorV2RequiresStop(const SupervisorV2Decision &decision);

} // namespace overtake_planner
