#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace state_lattice_overtake_planner {

using StateLatticeV2Digest = std::array<std::uint8_t, 32U>;

struct StateLatticeV2SemanticKey {
  // V2 envelope identity/provenance (except sequence, plan generation, stamps,
  // and canonical payload digest, which are regenerated on each publication).
  std::uint8_t v2_schema_version{0U};
  std::string producer_instance_id;
  std::string session_id;
  std::uint32_t identity_source_generation{0U};
  std::int32_t identity_source_stamp_sec{0};
  std::uint32_t identity_source_stamp_nanosec{0U};
  std::string frame_id;

  // Candidate metadata and its stable planner sample identity.
  std::uint8_t trajectory_schema_version{0U};
  bool authority_eligible{false};
  std::uint64_t race_arm_epoch{0U};
  std::uint64_t planner_instance_id{0U};
  std::string target_id;
  std::int8_t pass_direction{0};
  int planner_mode{0};
  int planner_intent{0};
  std::uint8_t candidate_type{0U};
  std::uint8_t phase{0U};
  std::uint8_t authorization_state{0U};

  // Exact PP base-attestation tuple and its provenance.
  std::uint64_t source_controller_instance_id{0U};
  std::uint64_t source_controller_sequence{0U};
  std::uint64_t base_lease_id{0U};
  std::int32_t base_lease_valid_until_sec{0};
  std::uint32_t base_lease_valid_until_nanosec{0U};
  std::uint8_t base_source_kind{0U};
  std::int32_t base_source_stamp_sec{0};
  std::uint32_t base_source_stamp_nanosec{0U};
  std::uint32_t base_source_generation{0U};
  std::uint32_t base_original_point_count{0U};
  std::uint32_t base_first_source_index{0U};
  std::uint32_t base_last_source_index{0U};
  std::uint32_t base_nearest_source_index{0U};
  std::uint8_t base_source_digest_state{0U};
  std::uint8_t canonical_algorithm_version{0U};
  StateLatticeV2Digest base_geometry_sha256{};
  StateLatticeV2Digest base_source_sha256{};
  StateLatticeV2Digest base_snapshot_sha256{};

  // Candidate geometry and safety/provenance evidence.
  StateLatticeV2Digest geometry_sha256{};
  std::uint32_t original_candidate_point_count{0U};
  double total_arc_length_m{0.0};
  double required_spatial_horizon_m{0.0};
  double join_end_arc_length_m{0.0};
  double post_join_arc_length_m{0.0};
  std::uint8_t safety_evaluation_result{0U};
  std::int32_t safety_valid_until_sec{0};
  std::uint32_t safety_valid_until_nanosec{0U};
  StateLatticeV2Digest world_safety_snapshot_sha256{};
  StateLatticeV2Digest safety_evaluator_implementation_sha256{};
  StateLatticeV2Digest safety_evaluator_config_sha256{};
  StateLatticeV2Digest safety_proof_sha256{};
  StateLatticeV2Digest controller_implementation_sha256{};
  StateLatticeV2Digest controller_config_sha256{};
  StateLatticeV2Digest candidate_start_control_pose_sha256{};
};

inline bool
sameStateLatticeV2SemanticKey(const StateLatticeV2SemanticKey &left,
                              const StateLatticeV2SemanticKey &right) {
  return left.v2_schema_version == right.v2_schema_version &&
         left.producer_instance_id == right.producer_instance_id &&
         left.session_id == right.session_id &&
         left.identity_source_generation == right.identity_source_generation &&
         left.identity_source_stamp_sec == right.identity_source_stamp_sec &&
         left.identity_source_stamp_nanosec ==
             right.identity_source_stamp_nanosec &&
         left.frame_id == right.frame_id &&
         left.trajectory_schema_version == right.trajectory_schema_version &&
         left.authority_eligible == right.authority_eligible &&
         left.race_arm_epoch == right.race_arm_epoch &&
         left.planner_instance_id == right.planner_instance_id &&
         left.target_id == right.target_id &&
         left.pass_direction == right.pass_direction &&
         left.planner_mode == right.planner_mode &&
         left.planner_intent == right.planner_intent &&
         left.candidate_type == right.candidate_type &&
         left.phase == right.phase &&
         left.authorization_state == right.authorization_state &&
         left.source_controller_instance_id ==
             right.source_controller_instance_id &&
         left.source_controller_sequence == right.source_controller_sequence &&
         left.base_lease_id == right.base_lease_id &&
         left.base_lease_valid_until_sec == right.base_lease_valid_until_sec &&
         left.base_lease_valid_until_nanosec ==
             right.base_lease_valid_until_nanosec &&
         left.base_source_kind == right.base_source_kind &&
         left.base_source_stamp_sec == right.base_source_stamp_sec &&
         left.base_source_stamp_nanosec == right.base_source_stamp_nanosec &&
         left.base_source_generation == right.base_source_generation &&
         left.base_original_point_count == right.base_original_point_count &&
         left.base_first_source_index == right.base_first_source_index &&
         left.base_last_source_index == right.base_last_source_index &&
         left.base_nearest_source_index == right.base_nearest_source_index &&
         left.base_source_digest_state == right.base_source_digest_state &&
         left.canonical_algorithm_version ==
             right.canonical_algorithm_version &&
         left.base_geometry_sha256 == right.base_geometry_sha256 &&
         left.base_source_sha256 == right.base_source_sha256 &&
         left.base_snapshot_sha256 == right.base_snapshot_sha256 &&
         left.geometry_sha256 == right.geometry_sha256 &&
         left.original_candidate_point_count ==
             right.original_candidate_point_count &&
         left.total_arc_length_m == right.total_arc_length_m &&
         left.required_spatial_horizon_m == right.required_spatial_horizon_m &&
         left.join_end_arc_length_m == right.join_end_arc_length_m &&
         left.post_join_arc_length_m == right.post_join_arc_length_m &&
         left.safety_evaluation_result == right.safety_evaluation_result &&
         left.safety_valid_until_sec == right.safety_valid_until_sec &&
         left.safety_valid_until_nanosec == right.safety_valid_until_nanosec &&
         left.world_safety_snapshot_sha256 ==
             right.world_safety_snapshot_sha256 &&
         left.safety_evaluator_implementation_sha256 ==
             right.safety_evaluator_implementation_sha256 &&
         left.safety_evaluator_config_sha256 ==
             right.safety_evaluator_config_sha256 &&
         left.safety_proof_sha256 == right.safety_proof_sha256 &&
         left.controller_implementation_sha256 ==
             right.controller_implementation_sha256 &&
         left.controller_config_sha256 == right.controller_config_sha256 &&
         left.candidate_start_control_pose_sha256 ==
             right.candidate_start_control_pose_sha256;
}

enum class StateLatticeV2PublicationPrepareResult : std::uint8_t {
  kReady,
  kDuplicate,
  kSequenceExhausted,
  kPlanGenerationExhausted,
  kHalted,
};

struct StateLatticeV2PublicationIntent {
  std::uint64_t proposal_sequence{0U};
  std::uint32_t plan_generation{0U};
  StateLatticeV2SemanticKey semantic_key{};
};

struct StateLatticeV2PublicationPreparation {
  StateLatticeV2PublicationPrepareResult result{
      StateLatticeV2PublicationPrepareResult::kDuplicate};
  std::optional<StateLatticeV2PublicationIntent> intent;
};

// This state is deliberately private to the V2 publisher. It advances only
// after a validated message is handed to the local publisher.
class StateLatticeV2PublicationState {
public:
  explicit StateLatticeV2PublicationState(std::uint64_t proposal_sequence = 0U,
                                          std::uint32_t plan_generation = 0U)
      : proposal_sequence_(proposal_sequence),
        plan_generation_(plan_generation) {}

  StateLatticeV2PublicationPreparation
  prepare(const StateLatticeV2SemanticKey &semantic_key) const {
    if (halted_) {
      return {StateLatticeV2PublicationPrepareResult::kHalted, std::nullopt};
    }
    if (proposal_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
      return {StateLatticeV2PublicationPrepareResult::kSequenceExhausted,
              std::nullopt};
    }
    if (plan_generation_ == std::numeric_limits<std::uint32_t>::max()) {
      return {StateLatticeV2PublicationPrepareResult::kPlanGenerationExhausted,
              std::nullopt};
    }
    if (last_published_semantic_key_.has_value() &&
        sameStateLatticeV2SemanticKey(last_published_semantic_key_.value(),
                                      semantic_key)) {
      return {StateLatticeV2PublicationPrepareResult::kDuplicate, std::nullopt};
    }
    return {StateLatticeV2PublicationPrepareResult::kReady,
            StateLatticeV2PublicationIntent{
                proposal_sequence_ + 1U, plan_generation_ + 1U, semantic_key}};
  }

  bool commit(const StateLatticeV2PublicationIntent &intent) {
    const auto expected = prepare(intent.semantic_key);
    if (expected.result != StateLatticeV2PublicationPrepareResult::kReady ||
        !expected.intent.has_value() ||
        expected.intent->proposal_sequence != intent.proposal_sequence ||
        expected.intent->plan_generation != intent.plan_generation) {
      return false;
    }
    proposal_sequence_ = intent.proposal_sequence;
    plan_generation_ = intent.plan_generation;
    last_published_semantic_key_ = intent.semantic_key;
    return true;
  }

  std::uint64_t proposalSequence() const { return proposal_sequence_; }
  std::uint32_t planGeneration() const { return plan_generation_; }
  bool halted() const { return halted_; }
  void halt() { halted_ = true; }

private:
  std::uint64_t proposal_sequence_{0U};
  std::uint32_t plan_generation_{0U};
  std::optional<StateLatticeV2SemanticKey> last_published_semantic_key_;
  bool halted_{false};
};

} // namespace state_lattice_overtake_planner
