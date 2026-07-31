#ifndef SIMPLE_PURE_PURSUIT__AW2_SHADOW_TRANSPORT_HPP_
#define SIMPLE_PURE_PURSUIT__AW2_SHADOW_TRANSPORT_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace simple_pure_pursuit::aw2_shadow {

inline constexpr std::uint64_t kSharedMagic = 0x4157325348505032ULL;
inline constexpr std::uint32_t kAbiVersion = 1U;
inline constexpr std::uint32_t kLayoutVersion = 2U;
inline constexpr std::uint32_t kLittleEndianMarker = 0x01020304U;
inline constexpr std::size_t kQueueCapacity = 8U;
inline constexpr std::size_t kLossRangeCapacity = 16U;
inline constexpr std::size_t kMaxTargetBytes = 64U;
inline constexpr std::size_t kMaxFrameBytes = 128U;
inline constexpr std::size_t kMaxSourceBytes = 4096U;
inline constexpr std::size_t kMaxGeometryPoints = 100U;
inline constexpr std::size_t kMaxRolloutSamples = 100U;
inline constexpr std::size_t kMaxConfigScalars = 32U;
inline constexpr std::uint8_t kControllerRolePrimary = 1U;

template <std::size_t Capacity> struct FixedString {
  std::uint16_t size{0U};
  std::array<char, Capacity> bytes{};
};

template <std::size_t Capacity>
bool copyFixedString(std::string_view source,
                     FixedString<Capacity> &destination) noexcept {
  if (source.size() > Capacity) {
    destination = {};
    return false;
  }
  destination.size = static_cast<std::uint16_t>(source.size());
  for (std::size_t index = 0U; index < source.size(); ++index) {
    destination.bytes[index] = source[index];
  }
  return true;
}

struct FixedTime {
  std::int32_t sec{0};
  std::uint32_t nanosec{0U};
};

struct FixedControllerSampleKey {
  std::uint64_t race_arm_epoch{0U};
  std::uint64_t planner_instance_id{0U};
  std::uint64_t attempt_id{0U};
  FixedString<kMaxTargetBytes> target_vehicle_id{};
  std::int8_t pass_direction{0};
  std::uint64_t connector_transaction_id{0U};
  FixedTime plan_stamp{};
  std::uint32_t plan_generation{0U};
  std::uint8_t controller_role{0U};
  std::uint64_t controller_instance_id{0U};
  std::uint64_t controller_sequence{0U};
  FixedTime controller_command_stamp{};
};

struct FixedCommand {
  FixedTime command_stamp{};
  FixedTime lateral_stamp{};
  float steering_tire_angle_rad{0.0F};
  float steering_tire_rotation_rate_radps{0.0F};
  FixedTime longitudinal_stamp{};
  float longitudinal_speed_mps{0.0F};
  float longitudinal_acceleration_mps2{0.0F};
  float longitudinal_jerk_mps3{0.0F};
};

struct FixedTrajectoryPoint {
  FixedTime time_from_start{};
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

struct FixedGeometry {
  FixedString<kMaxFrameBytes> frame_id{};
  FixedTime source_stamp{};
  std::uint32_t original_point_count{0U};
  std::uint32_t first_source_index{0U};
  std::uint32_t last_source_index{0U};
  std::uint32_t nearest_source_index{0U};
  std::uint32_t speed_cap_source_index{0U};
  std::uint32_t curvature_last_read_source_index{0U};
  std::uint32_t lookahead_selected_source_index{0U};
  std::uint32_t required_horizon_end_source_index{0U};
  bool lookahead_endpoint_fallback{false};
  std::uint32_t point_count{0U};
  std::array<FixedTrajectoryPoint, kMaxGeometryPoints> points{};
};

struct FixedPose {
  double position_x_m{0.0};
  double position_y_m{0.0};
  double position_z_m{0.0};
  double orientation_x{0.0};
  double orientation_y{0.0};
  double orientation_z{0.0};
  double orientation_w{1.0};
};

struct FixedRolloutSample {
  float elapsed_time_sec{0.0F};
  float progress_m{0.0F};
  FixedPose pose{};
  float speed_mps{0.0F};
  float raw_steering_tire_angle_rad{0.0F};
  float bounded_steering_tire_angle_rad{0.0F};
  float raw_steering_tire_rotation_rate_radps{0.0F};
  float bounded_steering_tire_rotation_rate_radps{0.0F};
};

enum class SnapshotKind : std::uint8_t {
  kSample = 1U,
  kResourceLimit = 2U,
};

enum class ResourceLimitKind : std::uint8_t {
  kNone = 0U,
  kTarget = 1U,
  kFrame = 2U,
  kSource = 3U,
  kGeometry = 4U,
  kRollout = 5U,
  kGeometryInterval = 6U,
  kHorizonUnavailable = 7U,
};

struct FixedSnapshot {
  SnapshotKind kind{SnapshotKind::kSample};
  ResourceLimitKind resource_limit_kind{ResourceLimitKind::kNone};
  FixedControllerSampleKey controller_sample_key{};
  FixedString<kMaxTargetBytes> target_vehicle_id{};
  FixedString<kMaxFrameBytes> record_frame_id{};
  FixedTime record_stamp{};
  std::uint32_t candidate_revision{0U};
  std::array<std::uint8_t, 32U> candidate_content_sha256{};
  bool typed_plan_present{false};
  bool typed_plan_fresh{false};
  bool typed_plan_trajectory_authorized{false};
  std::uint8_t typed_plan_identity_schema_version{0U};
  std::uint8_t geometry_relation{0U};
  std::uint32_t source_generation{0U};
  std::uint32_t source_original_size_bytes{0U};
  bool source_wire_complete{false};
  std::uint32_t source_wire_size{0U};
  std::array<std::uint8_t, kMaxSourceBytes> source_wire{};
  FixedGeometry base_geometry{};
  FixedGeometry applied_geometry{};
  FixedPose control_pose{};
  FixedTime control_pose_stamp{};
  std::uint32_t nearest_trajectory_index{0U};
  std::uint32_t speed_cap_trajectory_index{0U};
  std::uint32_t curvature_last_read_trajectory_index{0U};
  std::uint32_t lookahead_selected_trajectory_index{0U};
  std::uint32_t required_horizon_end_trajectory_index{0U};
  bool lookahead_endpoint_fallback{false};
  double trajectory_progress_m{0.0};
  FixedCommand raw_command{};
  FixedCommand output_command{};
  double raw_steering_tire_angle_rad{0.0};
  double output_steering_tire_angle_rad{0.0};
  double raw_steering_tire_rotation_rate_radps{0.0};
  double output_steering_tire_rotation_rate_radps{0.0};
  double available_spatial_horizon_m{0.0};
  double required_spatial_horizon_m{0.0};
  std::uint8_t rollout_state{0U};
  std::uint32_t rollout_original_count{0U};
  std::uint32_t rollout_count{0U};
  std::array<FixedRolloutSample, kMaxRolloutSamples> rollout{};
  std::uint32_t config_scalar_count{0U};
  std::array<double, kMaxConfigScalars> config_scalars{};
  std::uint64_t config_boolean_bits{0U};
};

struct FixedLossRange {
  std::uint64_t first_sequence{0U};
  std::uint64_t last_sequence{0U};
  FixedControllerSampleKey first_key{};
  FixedControllerSampleKey last_key{};
};

struct FixedLossLedger {
  std::uint64_t loss_generation{0U};
  std::uint64_t cumulative_drop_count{0U};
  std::uint64_t last_accepted_sequence{0U};
  std::uint32_t range_count{0U};
  std::array<FixedLossRange, kLossRangeCapacity> ranges{};
  bool global_membership_unknown{false};
  std::uint64_t first_uncertain_sequence{0U};
};

struct alignas(64) SharedLossLedger {
  alignas(8) std::uint64_t seqlock{0U};
  FixedLossLedger value{};
};

struct alignas(64) SharedSlot {
  alignas(8) std::uint64_t commit_index{0U};
  FixedSnapshot payload{};
};

struct alignas(64) SharedDataRegion {
  std::uint64_t magic{0U};
  std::uint32_t abi_version{0U};
  std::uint32_t layout_version{0U};
  std::uint32_t endianness{0U};
  std::uint32_t slot_size{0U};
  std::uint32_t capacity{0U};
  std::uint32_t reserved{0U};
  std::uint64_t ring_generation{0U};
  std::uint64_t controller_instance_id{0U};
  alignas(8) std::uint64_t race_arm_epoch{0U};
  std::array<std::uint8_t, 32U> producer_build_digest{};
  std::array<std::uint8_t, 32U> producer_config_digest{};
  alignas(8) std::uint64_t accepting{0U};
  alignas(8) std::uint64_t closing{0U};
  alignas(8) std::uint64_t write_index{0U};
  alignas(8) std::uint64_t queue_high_water{0U};
  SharedLossLedger loss_ledger{};
  std::array<SharedSlot, kQueueCapacity> slots{};
};

struct alignas(64) SharedAckRegion {
  std::uint64_t magic{0U};
  std::uint32_t abi_version{0U};
  std::uint32_t layout_version{0U};
  std::uint64_t ring_generation{0U};
  std::uint64_t controller_instance_id{0U};
  alignas(8) std::uint64_t read_index{0U};
  alignas(8) std::uint64_t consumer_instance_id{0U};
  alignas(8) std::uint64_t consumer_ready{0U};
  alignas(8) std::uint64_t torn_slot_count{0U};
  alignas(8) std::uint64_t abi_error_count{0U};
  alignas(8) std::uint64_t dds_error_count{0U};
  alignas(8) std::uint64_t shadow_disabled{0U};
};

enum class PopResult : std::uint8_t {
  kEmpty,
  kPopped,
  kTorn,
  kAbiMismatch,
};

bool hostIsLittleEndian() noexcept;
bool sharedAtomicsAreLockFree() noexcept;

std::uint64_t atomicLoadAcquire(const std::uint64_t &value) noexcept;
void atomicStoreRelease(std::uint64_t &destination,
                        std::uint64_t value) noexcept;

bool initializeSharedRegions(SharedDataRegion &data, SharedAckRegion &ack,
                             std::size_t capacity,
                             std::uint64_t ring_generation,
                             std::uint64_t controller_instance_id) noexcept;
bool validateSharedAbi(const SharedDataRegion &data,
                       const SharedAckRegion &ack) noexcept;
bool tryPush(SharedDataRegion &data, const SharedAckRegion &ack,
             const FixedSnapshot &snapshot) noexcept;
PopResult tryPop(const SharedDataRegion &data, SharedAckRegion &ack,
                 FixedSnapshot &snapshot) noexcept;

void recordAcceptedSequence(SharedLossLedger &ledger,
                            std::uint64_t sequence) noexcept;
bool readLossLedger(const SharedLossLedger &shared,
                    FixedLossLedger &ledger) noexcept;
bool sequenceIsDefinitelyLost(const FixedLossLedger &ledger,
                              std::uint64_t sequence) noexcept;

FixedSnapshot makeResourceLimitSnapshot(const FixedControllerSampleKey &key,
                                        ResourceLimitKind kind) noexcept;

} // namespace simple_pure_pursuit::aw2_shadow

#endif // SIMPLE_PURE_PURSUIT__AW2_SHADOW_TRANSPORT_HPP_
