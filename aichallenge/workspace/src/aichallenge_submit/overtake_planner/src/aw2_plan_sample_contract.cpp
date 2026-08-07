#include "overtake_planner/aw2_plan_sample_contract.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>

namespace overtake_planner::aw2 {
namespace {

constexpr std::array<std::uint32_t, 64U> kSha256RoundConstants{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU,
    0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U,
    0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U,
    0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U,
    0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
    0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U,
    0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U, 0x1e376c08U,
    0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU,
    0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

std::uint32_t rotateRight(std::uint32_t value, std::uint32_t bits) {
  return (value >> bits) | (value << (32U - bits));
}

void appendU32(std::vector<std::uint8_t> &bytes, std::uint32_t value) {
  for (std::uint32_t shift = 0U; shift < 32U; shift += 8U) {
    bytes.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
  }
}

void appendU64(std::vector<std::uint8_t> &bytes, std::uint64_t value) {
  for (std::uint32_t shift = 0U; shift < 64U; shift += 8U) {
    bytes.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
  }
}

void appendI32(std::vector<std::uint8_t> &bytes, std::int32_t value) {
  appendU32(bytes, static_cast<std::uint32_t>(value));
}

void appendFloat(std::vector<std::uint8_t> &bytes, float value) {
  std::uint32_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  appendU32(bytes, bits);
}

void appendDouble(std::vector<std::uint8_t> &bytes, double value) {
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  appendU64(bytes, bits);
}

void appendString(std::vector<std::uint8_t> &bytes, const std::string &value) {
  appendU32(bytes, static_cast<std::uint32_t>(value.size()));
  bytes.insert(bytes.end(), value.begin(), value.end());
}

bool digestEqual(const std::array<std::uint8_t, kSha256Size> &left,
                 const std::array<std::uint8_t, kSha256Size> &right) {
  return std::equal(left.begin(), left.end(), right.begin());
}

bool sameProducerScope(const PlanSampleKey &left, const PlanSampleKey &right) {
  return left.transaction.race_arm_epoch == right.transaction.race_arm_epoch &&
         left.transaction.planner_instance_id ==
             right.transaction.planner_instance_id;
}

bool samplePositionLessOrEqual(const PlanSampleKey &left,
                               const PlanSampleKey &right) {
  if (left.plan_generation != right.plan_generation) {
    return left.plan_generation < right.plan_generation;
  }
  if (left.plan_stamp_sec != right.plan_stamp_sec) {
    return left.plan_stamp_sec < right.plan_stamp_sec;
  }
  return left.plan_stamp_nanosec <= right.plan_stamp_nanosec;
}

bool validTime(std::int32_t sec, std::uint32_t nanosec) {
  return sec >= 0 && nanosec < 1000000000U;
}

bool finitePoint(const CandidateExecutionPoint &point) {
  const double quaternion_norm =
      std::sqrt(point.orientation_x * point.orientation_x +
                point.orientation_y * point.orientation_y +
                point.orientation_z * point.orientation_z +
                point.orientation_w * point.orientation_w);
  return validTime(point.time_sec, point.time_nanosec) &&
         std::isfinite(point.position_x_m) &&
         std::isfinite(point.position_y_m) &&
         std::isfinite(point.position_z_m) &&
         std::isfinite(point.orientation_x) &&
         std::isfinite(point.orientation_y) &&
         std::isfinite(point.orientation_z) &&
         std::isfinite(point.orientation_w) && std::isfinite(quaternion_norm) &&
         std::abs(quaternion_norm - 1.0) <= 1.0e-6 &&
         std::isfinite(point.longitudinal_velocity_mps) &&
         std::isfinite(point.lateral_velocity_mps) &&
         std::isfinite(point.acceleration_mps2) &&
         std::isfinite(point.heading_rate_rps) &&
         std::isfinite(point.front_wheel_angle_rad) &&
         std::isfinite(point.rear_wheel_angle_rad);
}

void appendPoint(std::vector<std::uint8_t> &bytes,
                 const CandidateExecutionPoint &point) {
  appendI32(bytes, point.time_sec);
  appendU32(bytes, point.time_nanosec);
  appendDouble(bytes, point.position_x_m);
  appendDouble(bytes, point.position_y_m);
  appendDouble(bytes, point.position_z_m);
  appendDouble(bytes, point.orientation_x);
  appendDouble(bytes, point.orientation_y);
  appendDouble(bytes, point.orientation_z);
  appendDouble(bytes, point.orientation_w);
  appendFloat(bytes, point.longitudinal_velocity_mps);
  appendFloat(bytes, point.lateral_velocity_mps);
  appendFloat(bytes, point.acceleration_mps2);
  appendFloat(bytes, point.heading_rate_rps);
  appendFloat(bytes, point.front_wheel_angle_rad);
  appendFloat(bytes, point.rear_wheel_angle_rad);
}

bool appendFiniteVector(std::vector<std::uint8_t> &bytes,
                        const std::vector<double> &values) {
  if (values.size() > kMaxGeometryPoints) {
    return false;
  }
  appendU32(bytes, static_cast<std::uint32_t>(values.size()));
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
    appendDouble(bytes, value);
  }
  return true;
}

enum class OptionalDoubleSentinel : std::uint8_t {
  NAN_IS_ABSENT,
  POSITIVE_INFINITY_IS_ABSENT,
};

bool appendOptionalDouble(std::vector<std::uint8_t> &bytes, double value,
                          OptionalDoubleSentinel sentinel) {
  if (std::isfinite(value)) {
    bytes.push_back(1U);
    appendDouble(bytes, value);
    return true;
  }
  const bool absent =
      (sentinel == OptionalDoubleSentinel::NAN_IS_ABSENT &&
       std::isnan(value)) ||
      (sentinel == OptionalDoubleSentinel::POSITIVE_INFINITY_IS_ABSENT &&
       value == std::numeric_limits<double>::infinity());
  if (!absent) {
    return false;
  }
  bytes.push_back(0U);
  return true;
}

} // namespace

bool TransactionKey::operator==(const TransactionKey &other) const {
  return race_arm_epoch == other.race_arm_epoch &&
         planner_instance_id == other.planner_instance_id &&
         attempt_id == other.attempt_id &&
         target_vehicle_id == other.target_vehicle_id &&
         pass_direction == other.pass_direction &&
         connector_transaction_id == other.connector_transaction_id;
}

bool PlanSampleKey::operator==(const PlanSampleKey &other) const {
  return transaction == other.transaction &&
         plan_stamp_sec == other.plan_stamp_sec &&
         plan_stamp_nanosec == other.plan_stamp_nanosec &&
         plan_generation == other.plan_generation;
}

std::array<std::uint8_t, kSha256Size>
sha256(const std::vector<std::uint8_t> &bytes) {
  std::vector<std::uint8_t> padded = bytes;
  const std::uint64_t bit_count =
      static_cast<std::uint64_t>(padded.size()) * 8U;
  padded.push_back(0x80U);
  while ((padded.size() % 64U) != 56U) {
    padded.push_back(0U);
  }
  for (int shift = 56; shift >= 0; shift -= 8) {
    padded.push_back(static_cast<std::uint8_t>((bit_count >> shift) & 0xffU));
  }

  std::array<std::uint32_t, 8U> state{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U,
                                      0xa54ff53aU, 0x510e527fU, 0x9b05688cU,
                                      0x1f83d9abU, 0x5be0cd19U};

  for (std::size_t chunk = 0U; chunk < padded.size(); chunk += 64U) {
    std::array<std::uint32_t, 64U> words{};
    for (std::size_t index = 0U; index < 16U; ++index) {
      const std::size_t offset = chunk + index * 4U;
      words[index] = (static_cast<std::uint32_t>(padded[offset]) << 24U) |
                     (static_cast<std::uint32_t>(padded[offset + 1U]) << 16U) |
                     (static_cast<std::uint32_t>(padded[offset + 2U]) << 8U) |
                     static_cast<std::uint32_t>(padded[offset + 3U]);
    }
    for (std::size_t index = 16U; index < words.size(); ++index) {
      const std::uint32_t s0 = rotateRight(words[index - 15U], 7U) ^
                               rotateRight(words[index - 15U], 18U) ^
                               (words[index - 15U] >> 3U);
      const std::uint32_t s1 = rotateRight(words[index - 2U], 17U) ^
                               rotateRight(words[index - 2U], 19U) ^
                               (words[index - 2U] >> 10U);
      words[index] = words[index - 16U] + s0 + words[index - 7U] + s1;
    }

    std::uint32_t a = state[0];
    std::uint32_t b = state[1];
    std::uint32_t c = state[2];
    std::uint32_t d = state[3];
    std::uint32_t e = state[4];
    std::uint32_t f = state[5];
    std::uint32_t g = state[6];
    std::uint32_t h = state[7];
    for (std::size_t index = 0U; index < words.size(); ++index) {
      const std::uint32_t sum1 =
          rotateRight(e, 6U) ^ rotateRight(e, 11U) ^ rotateRight(e, 25U);
      const std::uint32_t choose = (e & f) ^ ((~e) & g);
      const std::uint32_t temp1 =
          h + sum1 + choose + kSha256RoundConstants[index] + words[index];
      const std::uint32_t sum0 =
          rotateRight(a, 2U) ^ rotateRight(a, 13U) ^ rotateRight(a, 22U);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temp2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
    }
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
  }

  std::array<std::uint8_t, kSha256Size> digest{};
  for (std::size_t index = 0U; index < state.size(); ++index) {
    digest[index * 4U] =
        static_cast<std::uint8_t>((state[index] >> 24U) & 0xffU);
    digest[index * 4U + 1U] =
        static_cast<std::uint8_t>((state[index] >> 16U) & 0xffU);
    digest[index * 4U + 2U] =
        static_cast<std::uint8_t>((state[index] >> 8U) & 0xffU);
    digest[index * 4U + 3U] = static_cast<std::uint8_t>(state[index] & 0xffU);
  }
  return digest;
}

CanonicalCandidate
canonicalizeCandidateContentV1(const CandidateContent &content) {
  CanonicalCandidate result;
  const auto &transaction = content.key.transaction;
  if (transaction.race_arm_epoch == 0U ||
      transaction.planner_instance_id == 0U || transaction.attempt_id == 0U ||
      transaction.connector_transaction_id == 0U ||
      transaction.target_vehicle_id.empty()) {
    result.error = ValidationError::INVALID_IDENTITY;
    return result;
  }
  if (transaction.pass_direction != -1 && transaction.pass_direction != 1) {
    result.error = ValidationError::INVALID_SIDE;
    return result;
  }
  if (content.key.plan_stamp_sec < 0 ||
      content.key.plan_stamp_nanosec >= 1000000000U) {
    result.error = ValidationError::INVALID_STAMP;
    return result;
  }
  if (content.key.plan_generation == 0U) {
    result.error = ValidationError::INVALID_GENERATION;
    return result;
  }
  if (content.candidate_revision == 0U) {
    result.error = ValidationError::INVALID_REVISION;
    return result;
  }
  if (content.source_kind != CandidateSourceKind::LEGACY_REFERENCE_OVERRIDE &&
      content.source_kind != CandidateSourceKind::V2_TRAJECTORY) {
    result.error = ValidationError::INVALID_SOURCE_KIND;
    return result;
  }
  if (content.frame_id.empty() || content.frame_id.size() > kMaxFrameIdBytes) {
    result.error = ValidationError::FRAME_ID_LIMIT_EXCEEDED;
    return result;
  }
  if (transaction.target_vehicle_id.size() > kMaxTargetIdBytes) {
    result.error = ValidationError::TARGET_ID_LIMIT_EXCEEDED;
    return result;
  }
  if (content.geometry_point_count > kMaxGeometryPoints) {
    result.error = ValidationError::GEOMETRY_LIMIT_EXCEEDED;
    return result;
  }
  if (content.source_wire.empty() ||
      content.source_wire.size() > kMaxSourceWireBytes) {
    result.error = ValidationError::SOURCE_WIRE_LIMIT_EXCEEDED;
    return result;
  }

  constexpr char kDomainTag[] = "AW2:CANDIDATE:v1";
  result.bytes.insert(result.bytes.end(), std::begin(kDomainTag),
                      std::end(kDomainTag) - 1);
  appendU32(result.bytes, 1U);
  appendString(result.bytes, content.frame_id);
  appendString(result.bytes, transaction.target_vehicle_id);
  appendU64(result.bytes, transaction.race_arm_epoch);
  appendU64(result.bytes, transaction.planner_instance_id);
  appendU64(result.bytes, transaction.attempt_id);
  result.bytes.push_back(static_cast<std::uint8_t>(transaction.pass_direction));
  appendU64(result.bytes, transaction.connector_transaction_id);
  result.bytes.push_back(static_cast<std::uint8_t>(content.source_kind));
  appendU32(result.bytes,
            static_cast<std::uint32_t>(content.geometry_point_count));
  appendU32(result.bytes,
            static_cast<std::uint32_t>(content.source_wire.size()));
  result.bytes.insert(result.bytes.end(), content.source_wire.begin(),
                      content.source_wire.end());
  if (result.bytes.size() > kMaxCanonicalRecordBytes) {
    result.bytes.clear();
    result.error = ValidationError::CANONICAL_RECORD_LIMIT_EXCEEDED;
    return result;
  }
  result.sha256 = sha256(result.bytes);
  result.error = ValidationError::NONE;
  return result;
}

std::optional<std::vector<std::uint8_t>>
canonicalizeFloat32SourceWire(const Float32SourceWire &source) {
  if (source.values.empty() ||
      source.dimensions.size() > kMaxSourceLayoutDimensions) {
    return std::nullopt;
  }
  std::vector<std::uint8_t> bytes;
  constexpr char kDomainTag[] = "AW2:SOURCE_WIRE:v1";
  bytes.insert(bytes.end(), std::begin(kDomainTag), std::end(kDomainTag) - 1);
  appendU32(bytes, 1U);
  appendU32(bytes, static_cast<std::uint32_t>(source.dimensions.size()));
  for (const auto &dimension : source.dimensions) {
    if (dimension.label.size() > kMaxSourceLayoutLabelBytes) {
      return std::nullopt;
    }
    appendString(bytes, dimension.label);
    appendU32(bytes, dimension.size);
    appendU32(bytes, dimension.stride);
  }
  appendU32(bytes, source.data_offset);
  appendU32(bytes, static_cast<std::uint32_t>(source.values.size()));
  for (const float value : source.values) {
    if (!std::isfinite(value)) {
      return std::nullopt;
    }
    std::uint32_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    appendU32(bytes, bits);
    if (bytes.size() > kMaxSourceWireBytes) {
      return std::nullopt;
    }
  }
  return bytes;
}

std::optional<std::vector<std::uint8_t>>
canonicalizeFloat32SourceWire(const std::vector<float> &values) {
  Float32SourceWire source;
  source.values = values;
  return canonicalizeFloat32SourceWire(source);
}

std::optional<std::vector<std::uint8_t>>
canonicalizeV2SourceWire(const V2CanonicalSource &source) {
  const std::size_t point_count = source.x.size();
  if (point_count == 0U || point_count > kMaxGeometryPoints ||
      source.t.size() != point_count ||
      source.longitudinal_offsets_m.size() != point_count ||
      source.s.size() != point_count || source.d.size() != point_count ||
      source.y.size() != point_count || source.yaw.size() != point_count ||
      source.predicted_speed_mps.size() != point_count ||
      source.v_ref.size() != point_count ||
      source.reject_reason.size() > kMaxReasonBytes ||
      !std::isfinite(source.required_controller_spatial_horizon_m) ||
      !std::isfinite(source.score) || !std::isfinite(source.cbf_slack) ||
      source.active_safety_constraint_count < 0 ||
      !std::isfinite(source.assumed_brake_decel_mps2) ||
      !std::isfinite(source.response_delay_sec)) {
    return std::nullopt;
  }
  std::vector<std::uint8_t> bytes;
  constexpr char kDomainTag[] = "AW2:V2_SOURCE:v2";
  bytes.insert(bytes.end(), std::begin(kDomainTag), std::end(kDomainTag) - 1);
  appendU32(bytes, 2U);
  bytes.push_back(source.candidate_type);
  if (!appendFiniteVector(bytes, source.t) ||
      !appendFiniteVector(bytes, source.longitudinal_offsets_m) ||
      !appendFiniteVector(bytes, source.s) ||
      !appendFiniteVector(bytes, source.d) ||
      !appendFiniteVector(bytes, source.x) ||
      !appendFiniteVector(bytes, source.y) ||
      !appendFiniteVector(bytes, source.yaw)) {
    return std::nullopt;
  }
  if (!appendOptionalDouble(bytes,
                            source.longitudinal_initial_measured_speed_mps,
                            OptionalDoubleSentinel::NAN_IS_ABSENT)) {
    return std::nullopt;
  }
  if (!appendFiniteVector(bytes, source.predicted_speed_mps) ||
      !appendFiniteVector(bytes, source.v_ref)) {
    return std::nullopt;
  }
  bytes.push_back(source.safety_evaluated ? 1U : 0U);
  bytes.push_back(source.feasible ? 1U : 0U);
  bytes.push_back(source.pass_target_corridor_valid ? 1U : 0U);
  bytes.push_back(source.controller_tracking_profile_valid ? 1U : 0U);
  bytes.push_back(source.desired_path_trackable ? 1U : 0U);
  bytes.push_back(source.pure_pursuit_command_trackable ? 1U : 0U);
  bytes.push_back(source.moving_target_relatively_reachable ? 1U : 0U);
  if (!appendOptionalDouble(bytes, source.planned_target_d_m,
                            OptionalDoubleSentinel::NAN_IS_ABSENT) ||
      !appendOptionalDouble(bytes, source.committed_attack_follow_target_d_m,
                            OptionalDoubleSentinel::NAN_IS_ABSENT)) {
    return std::nullopt;
  }
  bytes.push_back(source.attack_follow_safe_lateral_hold ? 1U : 0U);
  bytes.push_back(source.attack_follow_opponent_collision_current_d_hold ? 1U
                                                                         : 0U);
  bytes.push_back(
      source.attack_follow_opponent_collision_inward_connector ? 1U : 0U);
  appendDouble(bytes, source.required_controller_spatial_horizon_m);
  bytes.push_back(source.controller_spatial_horizon_proof_valid ? 1U : 0U);
  appendDouble(bytes, source.score);
  if (!appendOptionalDouble(
          bytes, source.min_safety_margin,
          OptionalDoubleSentinel::POSITIVE_INFINITY_IS_ABSENT)) {
    return std::nullopt;
  }
  appendDouble(bytes, source.cbf_slack);
  appendI32(bytes, source.active_safety_constraint_count);
  bytes.push_back(source.longitudinal_profile_valid ? 1U : 0U);
  appendDouble(bytes, source.assumed_brake_decel_mps2);
  appendDouble(bytes, source.response_delay_sec);
  if (!appendOptionalDouble(
          bytes, source.required_brake_distance_m,
          OptionalDoubleSentinel::POSITIVE_INFINITY_IS_ABSENT) ||
      !appendOptionalDouble(
          bytes, source.available_brake_distance_m,
          OptionalDoubleSentinel::POSITIVE_INFINITY_IS_ABSENT)) {
    return std::nullopt;
  }
  appendString(bytes, source.reject_reason);
  if (bytes.size() > kMaxSourceWireBytes) {
    return std::nullopt;
  }
  return bytes;
}

CanonicalExecutionRecord
canonicalizeCandidateExecutionRecordV1(const CandidateExecutionRecord &record) {
  CanonicalExecutionRecord result;
  const auto &transaction = record.key.transaction;
  if (transaction.race_arm_epoch == 0U ||
      transaction.planner_instance_id == 0U || transaction.attempt_id == 0U ||
      transaction.connector_transaction_id == 0U ||
      transaction.target_vehicle_id.empty()) {
    result.error = ValidationError::INVALID_IDENTITY;
    return result;
  }
  if (transaction.pass_direction != -1 && transaction.pass_direction != 1) {
    result.error = ValidationError::INVALID_SIDE;
    return result;
  }
  if (!validTime(record.key.plan_stamp_sec, record.key.plan_stamp_nanosec) ||
      !validTime(record.constraint_stamp_sec,
                 record.constraint_stamp_nanosec) ||
      record.constraint_stamp_sec != record.key.plan_stamp_sec ||
      record.constraint_stamp_nanosec != record.key.plan_stamp_nanosec) {
    result.error = ValidationError::INVALID_STAMP;
    return result;
  }
  if (record.key.plan_generation == 0U || record.constraint_generation == 0U ||
      record.constraint_plan_generation != record.key.plan_generation ||
      record.source_generation != record.key.plan_generation) {
    result.error = ValidationError::INVALID_GENERATION;
    return result;
  }
  if (record.candidate_revision == 0U) {
    result.error = ValidationError::INVALID_REVISION;
    return result;
  }
  if (record.source_kind != CandidateSourceKind::LEGACY_REFERENCE_OVERRIDE &&
      record.source_kind != CandidateSourceKind::V2_TRAJECTORY) {
    result.error = ValidationError::INVALID_SOURCE_KIND;
    return result;
  }
  if (record.phase > 3U || record.candidate_type > 7U ||
      (record.published_pass_direction != -1 &&
       record.published_pass_direction != 0 &&
       record.published_pass_direction != 1) ||
      static_cast<std::uint8_t>(record.authorization_state) > 2U ||
      static_cast<std::uint8_t>(record.geometry_requirement) > 2U ||
      static_cast<std::uint8_t>(record.safety_evaluation_result) > 2U ||
      record.safety_snapshot_id == 0U) {
    result.error = ValidationError::INVALID_SOURCE_KIND;
    return result;
  }
  if (record.plan_frame_id.empty() || record.constraint_frame_id.empty() ||
      record.plan_frame_id.size() > kMaxFrameIdBytes ||
      record.constraint_frame_id.size() > kMaxFrameIdBytes) {
    result.error = ValidationError::FRAME_ID_LIMIT_EXCEEDED;
    return result;
  }
  if (transaction.target_vehicle_id.size() > kMaxTargetIdBytes) {
    result.error = ValidationError::TARGET_ID_LIMIT_EXCEEDED;
    return result;
  }
  if (record.constraint_reason.size() > kMaxReasonBytes ||
      record.safety_evaluation_reason.size() > kMaxReasonBytes) {
    result.error = ValidationError::CANONICAL_RECORD_LIMIT_EXCEEDED;
    return result;
  }
  if (record.geometry_points.size() > kMaxGeometryPoints) {
    result.error = ValidationError::GEOMETRY_LIMIT_EXCEEDED;
    return result;
  }
  if (record.typed_trajectory_present != !record.geometry_points.empty() ||
      (record.geometry_requirement == GeometryRequirement::REQUIRED &&
       record.geometry_points.empty())) {
    result.error = ValidationError::GEOMETRY_LIMIT_EXCEEDED;
    return result;
  }
  if (record.canonical_source_wire.empty() ||
      record.canonical_source_wire.size() > kMaxSourceWireBytes ||
      record.source_original_size_bytes !=
          record.canonical_source_wire.size()) {
    result.error = ValidationError::SOURCE_WIRE_LIMIT_EXCEEDED;
    return result;
  }
  if (!std::isfinite(record.constraint_speed_limit_mps) ||
      !std::isfinite(record.constraint_required_brake_decel_mps2) ||
      !std::isfinite(record.required_controller_spatial_horizon_m) ||
      !std::isfinite(record.planned_target_d_m) ||
      !std::isfinite(record.committed_target_d_m)) {
    result.error = ValidationError::CANONICAL_RECORD_LIMIT_EXCEEDED;
    return result;
  }
  for (const auto &point : record.geometry_points) {
    if (!finitePoint(point)) {
      result.error = ValidationError::GEOMETRY_LIMIT_EXCEEDED;
      return result;
    }
  }

  std::vector<std::uint8_t> geometry_bytes;
  constexpr char kGeometryDomain[] = "AW2:DELIVERED_GEOMETRY:v1";
  geometry_bytes.insert(geometry_bytes.end(), std::begin(kGeometryDomain),
                        std::end(kGeometryDomain) - 1);
  appendU32(geometry_bytes,
            static_cast<std::uint32_t>(record.geometry_points.size()));
  for (const auto &point : record.geometry_points) {
    appendPoint(geometry_bytes, point);
  }
  result.delivered_geometry_sha256 = sha256(geometry_bytes);
  result.canonical_source_sha256 = sha256(record.canonical_source_wire);

  constexpr char kRecordDomain[] = "AW2:PLAN_SAMPLE_RECORD:v1";
  auto &bytes = result.bytes;
  bytes.insert(bytes.end(), std::begin(kRecordDomain),
               std::end(kRecordDomain) - 1);
  appendU32(bytes, 1U);
  appendI32(bytes, record.key.plan_stamp_sec);
  appendU32(bytes, record.key.plan_stamp_nanosec);
  appendString(bytes, record.plan_frame_id);
  appendU64(bytes, transaction.planner_instance_id);
  appendU64(bytes, transaction.race_arm_epoch);
  appendU64(bytes, transaction.attempt_id);
  appendString(bytes, transaction.target_vehicle_id);
  bytes.push_back(static_cast<std::uint8_t>(transaction.pass_direction));
  bytes.push_back(static_cast<std::uint8_t>(record.published_pass_direction));
  appendU64(bytes, transaction.connector_transaction_id);
  appendU32(bytes, record.key.plan_generation);
  appendU32(bytes, record.candidate_revision);
  bytes.insert(bytes.end(), record.candidate_content_sha256.begin(),
               record.candidate_content_sha256.end());
  bytes.push_back(record.phase);
  bytes.push_back(static_cast<std::uint8_t>(record.authorization_state));
  bytes.push_back(static_cast<std::uint8_t>(record.geometry_requirement));
  bytes.push_back(record.candidate_type);
  bytes.push_back(record.trajectory_authorized_legacy ? 1U : 0U);
  bytes.push_back(record.lateral_maneuver_required_legacy ? 1U : 0U);
  bytes.push_back(record.typed_trajectory_present ? 1U : 0U);
  bytes.insert(bytes.end(), geometry_bytes.begin(), geometry_bytes.end());
  bytes.push_back(static_cast<std::uint8_t>(record.source_kind));
  appendU32(bytes, record.source_generation);
  appendU32(bytes, record.source_original_size_bytes);
  bytes.insert(bytes.end(), record.canonical_source_wire.begin(),
               record.canonical_source_wire.end());
  appendI32(bytes, record.constraint_stamp_sec);
  appendU32(bytes, record.constraint_stamp_nanosec);
  appendString(bytes, record.constraint_frame_id);
  appendU32(bytes, record.constraint_generation);
  appendU32(bytes, record.constraint_plan_generation);
  bytes.push_back(record.constraint_valid ? 1U : 0U);
  bytes.push_back(record.constraint_stop_requested ? 1U : 0U);
  bytes.push_back(record.constraint_release_authorized ? 1U : 0U);
  appendFloat(bytes, record.constraint_speed_limit_mps);
  appendFloat(bytes, record.constraint_required_brake_decel_mps2);
  appendString(bytes, record.constraint_reason);
  appendDouble(bytes, record.required_controller_spatial_horizon_m);
  appendDouble(bytes, record.planned_target_d_m);
  appendDouble(bytes, record.committed_target_d_m);
  appendU64(bytes, record.safety_snapshot_id);
  bytes.push_back(static_cast<std::uint8_t>(record.safety_evaluation_result));
  appendString(bytes, record.safety_evaluation_reason);
  if (bytes.size() > kMaxCanonicalRecordBytes) {
    result.bytes.clear();
    result.error = ValidationError::CANONICAL_RECORD_LIMIT_EXCEEDED;
    return result;
  }
  result.plan_sample_record_sha256 = sha256(bytes);
  result.error = ValidationError::NONE;
  return result;
}

DeliveryObservation DeliveryRecordTracker::observe(
    const PlanSampleKey &key,
    const std::array<std::uint8_t, kSha256Size> &record_sha256) {
  if (key.transaction.race_arm_epoch == 0U ||
      key.transaction.planner_instance_id == 0U ||
      key.transaction.connector_transaction_id == 0U ||
      key.plan_generation == 0U ||
      !validTime(key.plan_stamp_sec, key.plan_stamp_nanosec)) {
    return DeliveryObservation::INVALID;
  }
  const PlanSampleKey *scope_key =
      eviction_watermark_.has_value()
          ? &eviction_watermark_.value()
          : (entries_.empty() ? nullptr : &entries_.front().key);
  if (scope_key != nullptr && !sameProducerScope(*scope_key, key)) {
    reset();
  }
  if (eviction_watermark_.has_value() &&
      samplePositionLessOrEqual(key, eviction_watermark_.value())) {
    return DeliveryObservation::INVALID;
  }
  const auto existing =
      std::find_if(entries_.begin(), entries_.end(),
                   [&key](const Entry &entry) { return entry.key == key; });
  if (existing != entries_.end()) {
    if (existing->conflicted ||
        !digestEqual(existing->record_sha256, record_sha256)) {
      existing->conflicted = true;
      return DeliveryObservation::CONFLICT;
    }
    return DeliveryObservation::CONSISTENT_DUPLICATE;
  }
  entries_.push_back(Entry{key, record_sha256, false});
  while (entries_.size() > kDeliveryConflictHistorySize) {
    const auto &evicted_key = entries_.front().key;
    if (!eviction_watermark_.has_value() ||
        samplePositionLessOrEqual(eviction_watermark_.value(), evicted_key)) {
      eviction_watermark_ = evicted_key;
    }
    entries_.pop_front();
  }
  return DeliveryObservation::ACCEPTED;
}

void DeliveryRecordTracker::reset() {
  entries_.clear();
  eviction_watermark_.reset();
}

BindingObservation
SameGenerationBindingTracker::observe(const CandidateBinding &binding) {
  if (binding.race_arm_epoch == 0U || binding.planner_instance_id == 0U ||
      binding.plan_generation == 0U || binding.candidate_revision == 0U) {
    return BindingObservation::INVALID;
  }
  if (!last_binding_.has_value() ||
      binding.race_arm_epoch != last_binding_->race_arm_epoch ||
      binding.planner_instance_id != last_binding_->planner_instance_id) {
    last_binding_ = binding;
    rejected_generation_.reset();
    return BindingObservation::ACCEPTED;
  }
  if (binding.plan_generation < last_binding_->plan_generation) {
    return BindingObservation::GENERATION_REGRESSION;
  }
  if (binding.plan_generation == last_binding_->plan_generation) {
    if (rejected_generation_ == binding.plan_generation) {
      return BindingObservation::SAME_GENERATION_PAYLOAD_MUTATION;
    }
    if (binding.candidate_revision != last_binding_->candidate_revision ||
        !digestEqual(binding.candidate_content_sha256,
                     last_binding_->candidate_content_sha256)) {
      rejected_generation_ = binding.plan_generation;
      return BindingObservation::SAME_GENERATION_PAYLOAD_MUTATION;
    }
    return BindingObservation::CONSISTENT;
  }
  last_binding_ = binding;
  rejected_generation_.reset();
  return BindingObservation::ACCEPTED;
}

void SameGenerationBindingTracker::reset() {
  last_binding_.reset();
  rejected_generation_.reset();
}

std::uint8_t
identitySchemaVersion(bool canonical_source_complete,
                      const std::optional<std::uint64_t> &transaction_id,
                      BindingObservation observation) {
  const bool binding_accepted = observation == BindingObservation::ACCEPTED ||
                                observation == BindingObservation::CONSISTENT;
  return canonical_source_complete && transaction_id.has_value() &&
                 transaction_id.value() != 0U && binding_accepted
             ? 1U
             : 0U;
}

std::optional<std::uint64_t> nextRaceArmEpoch(std::uint64_t current_epoch) {
  if (current_epoch == std::numeric_limits<std::uint64_t>::max()) {
    return std::nullopt;
  }
  return current_epoch + 1U;
}

std::int8_t transactionPassDirection(std::int8_t published_pass_direction,
                                     bool maneuver_transaction_incomplete,
                                     std::int8_t latched_pass_direction) {
  if (published_pass_direction == -1 || published_pass_direction == 1) {
    return published_pass_direction;
  }
  if (maneuver_transaction_incomplete &&
      (latched_pass_direction == -1 || latched_pass_direction == 1)) {
    return latched_pass_direction;
  }
  return 0;
}

ConnectorTransactionSequencer::ConnectorTransactionSequencer(
    std::uint64_t initial_counter)
    : race_arm_epoch_(initial_counter == 0U ? 0U : 1U),
      counter_(initial_counter) {}

bool ConnectorTransactionSequencer::resetForRaceEpoch(
    std::uint64_t race_arm_epoch) {
  if (race_arm_epoch == 0U ||
      (race_arm_epoch_ != 0U && race_arm_epoch <= race_arm_epoch_)) {
    return false;
  }
  race_arm_epoch_ = race_arm_epoch;
  counter_ = 0U;
  current_id_ = 0U;
  current_attempt_id_ = 0U;
  current_target_vehicle_id_.clear();
  current_pass_direction_ = 0;
  active_ = false;
  exhausted_ = false;
  return true;
}

std::optional<std::uint64_t>
ConnectorTransactionSequencer::update(std::uint64_t race_arm_epoch,
                                      std::uint64_t attempt_id,
                                      const std::string &target_vehicle_id,
                                      std::int8_t pass_direction, bool active) {
  if (race_arm_epoch == 0U || race_arm_epoch != race_arm_epoch_) {
    return std::nullopt;
  }
  if (!active) {
    active_ = false;
    current_id_ = 0U;
    current_attempt_id_ = 0U;
    current_target_vehicle_id_.clear();
    current_pass_direction_ = 0;
    return std::nullopt;
  }
  if (exhausted_ || attempt_id == 0U || target_vehicle_id.empty() ||
      (pass_direction != -1 && pass_direction != 1)) {
    return std::nullopt;
  }
  const bool same_transaction =
      active_ && attempt_id == current_attempt_id_ &&
      target_vehicle_id == current_target_vehicle_id_ &&
      pass_direction == current_pass_direction_;
  if (same_transaction) {
    return current_id_;
  }
  if (counter_ == std::numeric_limits<std::uint64_t>::max()) {
    active_ = false;
    current_id_ = 0U;
    exhausted_ = true;
    return std::nullopt;
  }
  ++counter_;
  if (counter_ == 0U) {
    active_ = false;
    current_id_ = 0U;
    exhausted_ = true;
    return std::nullopt;
  }
  active_ = true;
  current_id_ = counter_;
  current_attempt_id_ = attempt_id;
  current_target_vehicle_id_ = target_vehicle_id;
  current_pass_direction_ = pass_direction;
  return current_id_;
}

} // namespace overtake_planner::aw2
