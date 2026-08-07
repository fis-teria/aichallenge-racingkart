#pragma once

#include <cstdint>
#include <cstring>
#include <optional>
#include <type_traits>

namespace state_lattice_overtake_planner {

enum class SpeedTransitionBranch : std::uint8_t {
  NONE = 0,
  ROLE_STOP_JERK_LIMITED = 1,
  ROLE_ACTIVE_JERK_LIMITED = 2,
  MOVING_FOLLOW_DIRECT = 3,
  TARGET_MISSING_YIELD_DIRECT = 4,
  PREPARE_OVERTAKE_DIRECT = 5,
  CANDIDATE_JERK_LIMITED = 6,
  CANDIDATE_LOGICAL_STOP = 7,
  LATCH_ONLY_STOP = 8,
  LATCH_ONLY_CLEAR = 9,
};

enum class SpeedEvidenceCaptureState : std::uint8_t {
  EMPTY = 0,
  TRIAL_COMPLETE_UNSEALED = 1,
  COMMITTED_SEALED = 2,
  INVALID_CAPTURE = 3,
};

enum class SpeedEvidenceFailure : std::uint8_t {
  NONE = 0,
  INCOMPLETE_TRANSITION = 1,
  PAYLOAD_BINDING_MISSING = 2,
  UNQUALIFIED_BRANCH = 3,
};

enum SpeedEvidenceField : std::uint32_t {
  SPEED_EVIDENCE_PRE_STATE = 1U << 0U,
  SPEED_EVIDENCE_EFFECTIVE_DT = 1U << 1U,
  SPEED_EVIDENCE_TRANSITION_INPUT = 1U << 2U,
  SPEED_EVIDENCE_POST_STATE = 1U << 3U,
  SPEED_EVIDENCE_PAYLOAD_BINDING = 1U << 4U,
};

constexpr std::uint32_t kSpeedEvidenceTransitionFields =
    SPEED_EVIDENCE_PRE_STATE | SPEED_EVIDENCE_EFFECTIVE_DT |
    SPEED_EVIDENCE_TRANSITION_INPUT | SPEED_EVIDENCE_POST_STATE;
constexpr std::uint32_t kSpeedEvidenceRequiredFields =
    kSpeedEvidenceTransitionFields | SPEED_EVIDENCE_PAYLOAD_BINDING;

struct SpeedTransitionStateEvidence {
  std::uint8_t initialized{0U};
  std::uint8_t safe_stop_latched{0U};
  std::int32_t safe_stop_release_count{0};
  double command_speed_mps{0.0};
  double acceleration_mps2{0.0};
};

// Composite key of the already-existing fixed proposal record. The raw ego,
// ordered opponents, candidate, base, config and implementation digests are
// resolved from that record outside the planning callback.
struct SpeedEvidencePayloadBinding {
  std::uint64_t session_generation{0U};
  std::uint64_t session_nonce{0U};
  std::uint64_t planner_instance_id{0U};
  std::uint64_t attempt_id{0U};
  std::uint64_t connector_transaction_id{0U};
  std::uint64_t authority_token{0U};
  std::uint64_t safety_snapshot_id{0U};
  std::uint32_t plan_generation{0U};
  std::uint32_t candidate_revision{0U};
  std::uint64_t race_arm_epoch{0U};
  std::uint64_t controller_instance_id{0U};
  std::uint64_t controller_sequence{0U};
  std::uint64_t base_lease_id{0U};
  std::int32_t base_source_stamp_sec{0};
  std::uint32_t base_source_stamp_nanosec{0U};
  std::uint32_t base_source_generation{0U};
};

struct TrialSpeedEvidence {
  std::uint16_t schema_version{1U};
  SpeedEvidenceCaptureState capture_state{SpeedEvidenceCaptureState::EMPTY};
  SpeedEvidenceFailure failure{SpeedEvidenceFailure::NONE};
  std::uint32_t present_fields{0U};
  SpeedTransitionBranch branch{SpeedTransitionBranch::NONE};
  std::uint8_t logical_stop{0U};
  std::uint8_t target_missing_recovery{0U};
  std::uint8_t state_changed{0U};
  double effective_dt_sec{0.0};
  double requested_speed_mps{0.0};
  double lower_speed_limit_mps{0.0};
  double upper_speed_limit_mps{0.0};
  std::int32_t candidate_lateral_index{-1};
  std::int32_t candidate_tangent_index{-1};
  double candidate_cost{0.0};
  SpeedEvidencePayloadBinding payload_binding{};
  SpeedTransitionStateEvidence pre_state{};
  SpeedTransitionStateEvidence post_state{};
};

struct SpeedEvidenceIdentity {
  std::uint64_t planner_session_id{0U};
  std::uint64_t state_epoch{0U};
  std::uint64_t attempt_ordinal{0U};
  std::uint64_t commit_ordinal{0U};
  std::uint64_t predecessor_commit_ordinal{0U};
  std::uint64_t previous_committed_attempt_ordinal{0U};
  std::uint32_t wire_semantic_generation{0U};
  std::uint64_t producer_record_sequence{0U};
  std::uint64_t cumulative_drop_count{0U};
};

struct CommittedSpeedEvidence {
  TrialSpeedEvidence trial{};
  SpeedEvidenceIdentity identity{};
  std::uint8_t sealed_after_commit{0U};
};

inline bool sameSpeedTransitionState(
    const SpeedTransitionStateEvidence &left,
    const SpeedTransitionStateEvidence &right);

enum class SpeedEvidenceMailboxState : std::uint8_t {
  EMPTY = 0,
  WRITING = 1,
  READY = 2,
  CONSUMED = 3,
};

class OneShotSpeedEvidencePairMailbox {
public:
  bool offer(const CommittedSpeedEvidence &record) noexcept {
    if (record.sealed_after_commit == 0U ||
        record.trial.capture_state !=
            SpeedEvidenceCaptureState::COMMITTED_SEALED ||
        record.trial.failure != SpeedEvidenceFailure::NONE) {
      ++drop_count_;
      return false;
    }
    if (state_ == SpeedEvidenceMailboxState::EMPTY) {
      predecessor_ = record;
      state_ = SpeedEvidenceMailboxState::WRITING;
      return true;
    }
    if (state_ != SpeedEvidenceMailboxState::WRITING ||
        record.identity.planner_session_id !=
            predecessor_.identity.planner_session_id ||
        record.identity.state_epoch != predecessor_.identity.state_epoch ||
        record.identity.commit_ordinal !=
            predecessor_.identity.commit_ordinal + 1U ||
        record.identity.predecessor_commit_ordinal !=
            predecessor_.identity.commit_ordinal ||
        record.identity.previous_committed_attempt_ordinal !=
            predecessor_.identity.attempt_ordinal ||
        !sameSpeedTransitionState(predecessor_.trial.post_state,
                                  record.trial.pre_state)) {
      ++drop_count_;
      state_ = SpeedEvidenceMailboxState::CONSUMED;
      return false;
    }
    target_ = record;
    state_ = SpeedEvidenceMailboxState::READY;
    return true;
  }

  SpeedEvidenceMailboxState state() const noexcept { return state_; }
  std::uint64_t dropCount() const noexcept { return drop_count_; }
  const CommittedSpeedEvidence &predecessor() const noexcept {
    return predecessor_;
  }
  const CommittedSpeedEvidence &target() const noexcept { return target_; }
  void consume() noexcept {
    if (state_ == SpeedEvidenceMailboxState::READY) {
      state_ = SpeedEvidenceMailboxState::CONSUMED;
    }
  }
  void invalidate() noexcept {
    if (state_ == SpeedEvidenceMailboxState::WRITING) {
      state_ = SpeedEvidenceMailboxState::CONSUMED;
      ++drop_count_;
    }
  }
  void beginEpoch() noexcept {
    if (state_ == SpeedEvidenceMailboxState::WRITING ||
        state_ == SpeedEvidenceMailboxState::READY) {
      ++drop_count_;
    }
    state_ = SpeedEvidenceMailboxState::EMPTY;
    predecessor_ = CommittedSpeedEvidence{};
    target_ = CommittedSpeedEvidence{};
  }

private:
  SpeedEvidenceMailboxState state_{SpeedEvidenceMailboxState::EMPTY};
  CommittedSpeedEvidence predecessor_{};
  CommittedSpeedEvidence target_{};
  std::uint64_t drop_count_{0U};
};

// Node-owned transaction state. prepareSpeedEvidenceCommit() is deliberately
// called only after the transactional planner move. Deadline reuse/stop call
// discardSpeedEvidenceTrial(), which is an explicit no-op on commit authority.
struct SpeedEvidenceCommitState {
  std::uint64_t commit_ordinal{0U};
  std::uint64_t record_sequence{0U};
  std::optional<CommittedSpeedEvidence> last_committed;
  OneShotSpeedEvidencePairMailbox mailbox;
};

inline SpeedEvidenceIdentity prepareSpeedEvidenceCommitAfterPlannerMove(
    SpeedEvidenceCommitState &state, std::uint64_t planner_session_id,
    std::uint64_t state_epoch, std::uint64_t attempt_ordinal) noexcept {
  SpeedEvidenceIdentity identity{};
  identity.planner_session_id = planner_session_id;
  identity.state_epoch = state_epoch;
  identity.attempt_ordinal = attempt_ordinal;
  identity.commit_ordinal = ++state.commit_ordinal;
  if (state.last_committed.has_value() &&
      state.last_committed->identity.state_epoch == state_epoch) {
    identity.predecessor_commit_ordinal =
        state.last_committed->identity.commit_ordinal;
    identity.previous_committed_attempt_ordinal =
        state.last_committed->identity.attempt_ordinal;
  }
  identity.producer_record_sequence = ++state.record_sequence;
  identity.cumulative_drop_count = state.mailbox.dropCount();
  return identity;
}

inline void finishSpeedEvidenceCommit(
    SpeedEvidenceCommitState &state,
    const CommittedSpeedEvidence &committed) noexcept {
  state.last_committed = committed;
  if (committed.trial.capture_state ==
      SpeedEvidenceCaptureState::COMMITTED_SEALED) {
    (void)state.mailbox.offer(committed);
  } else {
    state.mailbox.invalidate();
  }
}

inline void discardSpeedEvidenceTrial(SpeedEvidenceCommitState &) noexcept {}

inline void beginSpeedEvidenceEpoch(SpeedEvidenceCommitState &state) noexcept {
  state.mailbox.beginEpoch();
  state.last_committed.reset();
}

inline bool speedEvidenceBranchReadyEligible(
    SpeedTransitionBranch branch) noexcept {
  switch (branch) {
  case SpeedTransitionBranch::PREPARE_OVERTAKE_DIRECT:
  case SpeedTransitionBranch::CANDIDATE_JERK_LIMITED:
  case SpeedTransitionBranch::CANDIDATE_LOGICAL_STOP:
    return true;
  default:
    return false;
  }
}

inline bool sameSpeedTransitionState(const SpeedTransitionStateEvidence &left,
                                     const SpeedTransitionStateEvidence &right) {
  return left.initialized == right.initialized &&
         left.safe_stop_latched == right.safe_stop_latched &&
         left.safe_stop_release_count == right.safe_stop_release_count &&
         std::memcmp(&left.command_speed_mps, &right.command_speed_mps,
                     sizeof(double)) == 0 &&
         std::memcmp(&left.acceleration_mps2, &right.acceleration_mps2,
                     sizeof(double)) == 0;
}

inline CommittedSpeedEvidence sealCommittedSpeedEvidenceAfterCommit(
    const TrialSpeedEvidence &trial, const SpeedEvidenceIdentity &identity) {
  CommittedSpeedEvidence committed{};
  committed.trial = trial;
  committed.identity = identity;
  committed.sealed_after_commit = 1U;
  if ((trial.present_fields & kSpeedEvidenceRequiredFields) !=
      kSpeedEvidenceRequiredFields) {
    committed.trial.capture_state = SpeedEvidenceCaptureState::INVALID_CAPTURE;
    committed.trial.failure =
        (trial.present_fields & kSpeedEvidenceTransitionFields) ==
                kSpeedEvidenceTransitionFields
            ? SpeedEvidenceFailure::PAYLOAD_BINDING_MISSING
            : SpeedEvidenceFailure::INCOMPLETE_TRANSITION;
  } else if (!speedEvidenceBranchReadyEligible(trial.branch)) {
    committed.trial.capture_state = SpeedEvidenceCaptureState::INVALID_CAPTURE;
    committed.trial.failure = SpeedEvidenceFailure::UNQUALIFIED_BRANCH;
  } else {
    committed.trial.capture_state =
        SpeedEvidenceCaptureState::COMMITTED_SEALED;
    committed.trial.failure = SpeedEvidenceFailure::NONE;
  }
  return committed;
}

static_assert(std::is_trivially_copyable_v<SpeedTransitionStateEvidence>);
static_assert(std::is_trivially_copyable_v<TrialSpeedEvidence>);
static_assert(std::is_trivially_copyable_v<CommittedSpeedEvidence>);

} // namespace state_lattice_overtake_planner
