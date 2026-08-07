#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace overtake_planner::aw2 {

constexpr std::size_t kSha256Size = 32U;
constexpr std::size_t kMaxGeometryPoints = 100U;
constexpr std::size_t kMaxSourceWireBytes = 4096U;
constexpr std::size_t kMaxSourceLayoutDimensions = 8U;
constexpr std::size_t kMaxSourceLayoutLabelBytes = 64U;
constexpr std::size_t kMaxFrameIdBytes = 128U;
constexpr std::size_t kMaxTargetIdBytes = 64U;
constexpr std::size_t kMaxReasonBytes = 256U;
constexpr std::size_t kMaxCanonicalRecordBytes = 1024U * 1024U;
constexpr std::size_t kDeliveryConflictHistorySize = 8U;

struct TransactionKey {
  std::uint64_t race_arm_epoch{0U};
  std::uint64_t planner_instance_id{0U};
  std::uint64_t attempt_id{0U};
  std::string target_vehicle_id;
  std::int8_t pass_direction{0};
  std::uint64_t connector_transaction_id{0U};

  bool operator==(const TransactionKey &other) const;
};

struct PlanSampleKey {
  TransactionKey transaction;
  std::int32_t plan_stamp_sec{0};
  std::uint32_t plan_stamp_nanosec{0U};
  std::uint32_t plan_generation{0U};

  bool operator==(const PlanSampleKey &other) const;
};

enum class CandidateSourceKind : std::uint8_t {
  UNKNOWN = 0U,
  LEGACY_REFERENCE_OVERRIDE = 1U,
  V2_TRAJECTORY = 2U,
};

struct CandidateContent {
  PlanSampleKey key;
  std::uint32_t candidate_revision{0U};
  std::string frame_id;
  CandidateSourceKind source_kind{CandidateSourceKind::UNKNOWN};
  std::size_t geometry_point_count{0U};
  std::vector<std::uint8_t> source_wire;
};

enum class ValidationError : std::uint8_t {
  NONE = 0U,
  INVALID_IDENTITY,
  INVALID_STAMP,
  INVALID_GENERATION,
  INVALID_REVISION,
  INVALID_SIDE,
  INVALID_SOURCE_KIND,
  FRAME_ID_LIMIT_EXCEEDED,
  TARGET_ID_LIMIT_EXCEEDED,
  GEOMETRY_LIMIT_EXCEEDED,
  SOURCE_WIRE_LIMIT_EXCEEDED,
  CANONICAL_RECORD_LIMIT_EXCEEDED,
};

struct CanonicalCandidate {
  ValidationError error{ValidationError::INVALID_IDENTITY};
  std::vector<std::uint8_t> bytes;
  std::array<std::uint8_t, kSha256Size> sha256{};

  bool valid() const { return error == ValidationError::NONE; }
};

CanonicalCandidate
canonicalizeCandidateContentV1(const CandidateContent &content);

struct Float32LayoutDimension {
  std::string label;
  std::uint32_t size{0U};
  std::uint32_t stride{0U};
};

struct Float32SourceWire {
  std::vector<Float32LayoutDimension> dimensions;
  std::uint32_t data_offset{0U};
  std::vector<float> values;
};

std::optional<std::vector<std::uint8_t>>
canonicalizeFloat32SourceWire(const Float32SourceWire &source);

std::optional<std::vector<std::uint8_t>>
canonicalizeFloat32SourceWire(const std::vector<float> &values);

std::array<std::uint8_t, kSha256Size>
sha256(const std::vector<std::uint8_t> &bytes);

struct CandidateExecutionPoint {
  std::int32_t time_sec{0};
  std::uint32_t time_nanosec{0U};
  double position_x_m{0.0};
  double position_y_m{0.0};
  double position_z_m{0.0};
  double orientation_x{0.0};
  double orientation_y{0.0};
  double orientation_z{0.0};
  double orientation_w{1.0};
  float longitudinal_velocity_mps{0.0F};
  float lateral_velocity_mps{0.0F};
  float acceleration_mps2{0.0F};
  float heading_rate_rps{0.0F};
  float front_wheel_angle_rad{0.0F};
  float rear_wheel_angle_rad{0.0F};
};

enum class AuthorizationState : std::uint8_t {
  NOT_AUTHORIZED = 0U,
  AUTHORIZED = 1U,
  UNKNOWN = 2U,
};

enum class GeometryRequirement : std::uint8_t {
  NOT_REQUIRED = 0U,
  REQUIRED = 1U,
  UNKNOWN = 2U,
};

enum class SafetyEvaluationResult : std::uint8_t {
  NOT_EVALUATED = 0U,
  REJECTED = 1U,
  PASSED = 2U,
};

struct CandidateExecutionRecord {
  PlanSampleKey key;
  std::uint32_t candidate_revision{0U};
  std::array<std::uint8_t, kSha256Size> candidate_content_sha256{};
  std::string plan_frame_id;
  std::uint8_t phase{0U};
  AuthorizationState authorization_state{AuthorizationState::NOT_AUTHORIZED};
  GeometryRequirement geometry_requirement{GeometryRequirement::NOT_REQUIRED};
  std::uint8_t candidate_type{0U};
  bool trajectory_authorized_legacy{false};
  bool lateral_maneuver_required_legacy{false};
  std::int8_t published_pass_direction{0};
  bool typed_trajectory_present{false};
  std::vector<CandidateExecutionPoint> geometry_points;
  CandidateSourceKind source_kind{CandidateSourceKind::UNKNOWN};
  std::uint32_t source_generation{0U};
  std::uint32_t source_original_size_bytes{0U};
  std::vector<std::uint8_t> canonical_source_wire;
  std::int32_t constraint_stamp_sec{0};
  std::uint32_t constraint_stamp_nanosec{0U};
  std::string constraint_frame_id;
  std::uint32_t constraint_generation{0U};
  std::uint32_t constraint_plan_generation{0U};
  bool constraint_valid{false};
  bool constraint_stop_requested{true};
  bool constraint_release_authorized{false};
  float constraint_speed_limit_mps{0.0F};
  float constraint_required_brake_decel_mps2{0.0F};
  std::string constraint_reason;
  double required_controller_spatial_horizon_m{
      std::numeric_limits<double>::quiet_NaN()};
  double planned_target_d_m{std::numeric_limits<double>::quiet_NaN()};
  double committed_target_d_m{std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t safety_snapshot_id{0U};
  SafetyEvaluationResult safety_evaluation_result{
      SafetyEvaluationResult::NOT_EVALUATED};
  std::string safety_evaluation_reason;
};

struct CanonicalExecutionRecord {
  ValidationError error{ValidationError::INVALID_IDENTITY};
  std::vector<std::uint8_t> bytes;
  std::array<std::uint8_t, kSha256Size> delivered_geometry_sha256{};
  std::array<std::uint8_t, kSha256Size> canonical_source_sha256{};
  std::array<std::uint8_t, kSha256Size> plan_sample_record_sha256{};

  bool valid() const { return error == ValidationError::NONE; }
};

CanonicalExecutionRecord
canonicalizeCandidateExecutionRecordV1(const CandidateExecutionRecord &record);

struct V2CanonicalSource {
  std::uint8_t candidate_type{0U};
  std::vector<double> t;
  std::vector<double> longitudinal_offsets_m;
  std::vector<double> s;
  std::vector<double> d;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  double longitudinal_initial_measured_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> predicted_speed_mps;
  std::vector<double> v_ref;
  bool safety_evaluated{false};
  bool feasible{false};
  bool pass_target_corridor_valid{false};
  bool controller_tracking_profile_valid{false};
  bool desired_path_trackable{false};
  bool pure_pursuit_command_trackable{false};
  bool moving_target_relatively_reachable{false};
  double planned_target_d_m{std::numeric_limits<double>::quiet_NaN()};
  double committed_attack_follow_target_d_m{
      std::numeric_limits<double>::quiet_NaN()};
  bool attack_follow_safe_lateral_hold{false};
  bool attack_follow_opponent_collision_current_d_hold{false};
  bool attack_follow_opponent_collision_inward_connector{false};
  double required_controller_spatial_horizon_m{0.0};
  bool controller_spatial_horizon_proof_valid{false};
  double score{0.0};
  double min_safety_margin{0.0};
  double cbf_slack{0.0};
  std::int32_t active_safety_constraint_count{0};
  bool longitudinal_profile_valid{false};
  double assumed_brake_decel_mps2{0.0};
  double response_delay_sec{0.0};
  double required_brake_distance_m{0.0};
  double available_brake_distance_m{0.0};
  std::string reject_reason;
};

std::optional<std::vector<std::uint8_t>>
canonicalizeV2SourceWire(const V2CanonicalSource &source);

enum class DeliveryObservation : std::uint8_t {
  ACCEPTED = 0U,
  CONSISTENT_DUPLICATE,
  INVALID,
  CONFLICT,
};

class DeliveryRecordTracker {
public:
  DeliveryObservation
  observe(const PlanSampleKey &key,
          const std::array<std::uint8_t, kSha256Size> &record_sha256);
  void reset();

private:
  struct Entry {
    PlanSampleKey key;
    std::array<std::uint8_t, kSha256Size> record_sha256{};
    bool conflicted{false};
  };
  std::deque<Entry> entries_;
  std::optional<PlanSampleKey> eviction_watermark_;
};

enum class BindingObservation : std::uint8_t {
  ACCEPTED = 0U,
  CONSISTENT,
  INVALID,
  GENERATION_REGRESSION,
  SAME_GENERATION_PAYLOAD_MUTATION,
};

struct CandidateBinding {
  std::uint64_t race_arm_epoch{0U};
  std::uint64_t planner_instance_id{0U};
  std::uint32_t plan_generation{0U};
  std::uint32_t candidate_revision{0U};
  std::array<std::uint8_t, kSha256Size> candidate_content_sha256{};
};

class SameGenerationBindingTracker {
public:
  BindingObservation observe(const CandidateBinding &binding);
  void reset();

private:
  std::optional<CandidateBinding> last_binding_;
  std::optional<std::uint32_t> rejected_generation_;
};

std::uint8_t
identitySchemaVersion(bool canonical_source_complete,
                      const std::optional<std::uint64_t> &transaction_id,
                      BindingObservation observation);

std::optional<std::uint64_t> nextRaceArmEpoch(std::uint64_t current_epoch);

std::int8_t transactionPassDirection(std::int8_t published_pass_direction,
                                     bool maneuver_transaction_incomplete,
                                     std::int8_t latched_pass_direction);

class ConnectorTransactionSequencer {
public:
  explicit ConnectorTransactionSequencer(std::uint64_t initial_counter = 0U);

  std::optional<std::uint64_t> update(std::uint64_t race_arm_epoch,
                                      std::uint64_t attempt_id,
                                      const std::string &target_vehicle_id,
                                      std::int8_t pass_direction, bool active);

  bool resetForRaceEpoch(std::uint64_t race_arm_epoch);
  std::uint64_t currentId() const { return current_id_; }
  bool exhausted() const { return exhausted_; }

private:
  std::uint64_t race_arm_epoch_{0U};
  std::uint64_t counter_{0U};
  std::uint64_t current_id_{0U};
  std::uint64_t current_attempt_id_{0U};
  std::string current_target_vehicle_id_;
  std::int8_t current_pass_direction_{0};
  bool active_{false};
  bool exhausted_{false};
};

} // namespace overtake_planner::aw2
