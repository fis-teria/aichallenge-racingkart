#ifndef OVERTAKE_TRANSPORT_CONTRACT__C002AY1_RUNTIME_OBSERVER_HPP_
#define OVERTAKE_TRANSPORT_CONTRACT__C002AY1_RUNTIME_OBSERVER_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>

namespace overtake_transport_contract::c002ay1 {

inline constexpr std::uint64_t kRuntimeRingMagic = 0x4330303241593152ULL;
inline constexpr std::uint64_t kRuntimeHandshakeMagic = 0x4330303241593148ULL;
inline constexpr std::uint32_t kRuntimeAbiVersion = 1U;
inline constexpr std::uint32_t kRuntimeLayoutVersion = 1U;
inline constexpr std::size_t kPlannerRingCapacity = 4096U;
inline constexpr std::size_t kPpRingCapacity = 16384U;
inline constexpr std::size_t kMaxRunIdBytes = 64U;
inline constexpr std::chrono::milliseconds kAttachDeadline{100};

enum class ProducerRole : std::uint8_t {
  kInvalid = 0U,
  kPlanner = 1U,
  kPrimaryPurePursuit = 2U,
};

enum class RingState : std::uint64_t {
  kUnexposed = 0U,
  kHarnessReady = 1U,
  kProducerAttached = 2U,
  kRecording = 3U,
  kComplete = 4U,
  kInvalid = 5U,
};

enum class ConfiguredStreamKind : std::uint8_t {
  kNone = 0U,
  kLegacyReferenceOverride = 1U,
  kV2Trajectory = 2U,
};

enum class ReturnReason : std::uint16_t {
  kUnsetInvalid = 0U,
  kCompletedNoEmit = 1U,
  kCompletedEmit = 2U,
  kEarlyInputUnavailable = 3U,
  kEarlyInputStale = 4U,
  kEarlyInputInvalid = 5U,
  kExternalAuthorityStop = 6U,
  kExistingExceptionGuard = 7U,
  kUnknownInvalid = 255U,
};

inline constexpr std::uint16_t kFlagEmitted = 1U << 0U;
inline constexpr std::uint16_t kFlagIdentityCaptured = 1U << 1U;
inline constexpr std::uint16_t kFlagDropSeen = 1U << 2U;

struct PlannerCallbackObservationV1 {
  std::uint64_t sequence{0U};
  std::uint64_t steady_start_ns{0U};
  std::uint64_t duration_ns{0U};
  std::int32_t ros_sec{0};
  std::uint32_t ros_nanosec{0U};
  std::uint64_t selected_proposal_sequence{0U};
  std::uint64_t cadence_identity_hint_token{0U};
  std::uint16_t return_reason{
      static_cast<std::uint16_t>(ReturnReason::kUnsetInvalid)};
  std::uint16_t flags{0U};
  std::uint8_t configured_stream_kind{
      static_cast<std::uint8_t>(ConfiguredStreamKind::kNone)};
  std::uint8_t legacy_publish_count{0U};
  std::uint8_t v2_publish_count{0U};
  std::uint8_t selected_proposal_count{0U};
};

struct PpCallbackObservationV1 {
  std::uint64_t sequence{0U};
  std::uint64_t steady_start_ns{0U};
  std::uint64_t duration_ns{0U};
  std::int32_t ros_sec{0};
  std::uint32_t ros_nanosec{0U};
  std::uint16_t return_reason{
      static_cast<std::uint16_t>(ReturnReason::kUnsetInvalid)};
  std::uint16_t flags{0U};
  std::uint8_t controller_role{
      static_cast<std::uint8_t>(ProducerRole::kInvalid)};
  std::array<std::uint8_t, 3U> reserved{};
};

static_assert(sizeof(PlannerCallbackObservationV1) == 56U);
static_assert(sizeof(PpCallbackObservationV1) == 40U);
static_assert(std::is_trivially_copyable_v<PlannerCallbackObservationV1>);
static_assert(std::is_trivially_copyable_v<PpCallbackObservationV1>);
static_assert(std::is_standard_layout_v<PlannerCallbackObservationV1>);
static_assert(std::is_standard_layout_v<PpCallbackObservationV1>);

struct alignas(64) RuntimeRingHeaderV1 {
  std::uint64_t magic{0U};
  std::uint32_t abi_version{0U};
  std::uint32_t layout_version{0U};
  std::uint32_t header_size{0U};
  std::uint32_t record_size{0U};
  std::uint32_t capacity{0U};
  std::uint8_t producer_role{0U};
  std::array<std::uint8_t, 3U> reserved0{};
  std::uint64_t mapping_size{0U};
  std::uint64_t session_nonce{0U};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t run_id_hash{0U};
  alignas(64) std::uint64_t state{0U};
  alignas(64) std::uint64_t write_index{0U};
  alignas(64) std::uint64_t read_index{0U};
  alignas(64) std::uint64_t drop_count{0U};
};

template <typename Record, std::size_t Capacity>
struct alignas(64) RuntimeRingV1 {
  RuntimeRingHeaderV1 header{};
  std::array<Record, Capacity> records{};
};

using PlannerRuntimeRingV1 =
    RuntimeRingV1<PlannerCallbackObservationV1, kPlannerRingCapacity>;
using PpRuntimeRingV1 = RuntimeRingV1<PpCallbackObservationV1, kPpRingCapacity>;

static_assert(sizeof(RuntimeRingHeaderV1) == 320U);
static_assert(std::is_trivially_copyable_v<RuntimeRingHeaderV1>);
static_assert(std::is_standard_layout_v<RuntimeRingHeaderV1>);
static_assert(std::is_trivially_copyable_v<PlannerRuntimeRingV1>);
static_assert(std::is_standard_layout_v<PlannerRuntimeRingV1>);
static_assert(std::is_trivially_copyable_v<PpRuntimeRingV1>);
static_assert(std::is_standard_layout_v<PpRuntimeRingV1>);
static_assert(sizeof(PlannerRuntimeRingV1) + sizeof(PpRuntimeRingV1) <=
              1024U * 1024U);

struct RuntimeHandshakeV1 {
  std::uint64_t magic{kRuntimeHandshakeMagic};
  std::uint32_t abi_version{kRuntimeAbiVersion};
  std::uint8_t producer_role{0U};
  std::array<std::uint8_t, 3U> reserved{};
  std::int32_t producer_pid{0};
  std::uint32_t run_id_size{0U};
  std::uint64_t session_nonce{0U};
  std::uint64_t producer_instance_id{0U};
  std::array<char, kMaxRunIdBytes> run_id{};
};

static_assert(sizeof(RuntimeHandshakeV1) == 104U);
static_assert(std::is_trivially_copyable_v<RuntimeHandshakeV1>);
static_assert(std::is_standard_layout_v<RuntimeHandshakeV1>);

struct RuntimeObserverConfig {
  bool enabled{false};
  ProducerRole role{ProducerRole::kInvalid};
  std::string socket_path;
  std::string run_id;
  std::uint64_t session_nonce{0U};
  std::uint64_t producer_instance_id{0U};
};

std::uint64_t steadyNowNs() noexcept;
std::uint64_t hashRunId(const char *data, std::size_t size) noexcept;
std::size_t mappingSizeForRole(ProducerRole role) noexcept;
bool initializeRuntimeRingForHarness(void *mapping, std::size_t mapping_size,
                                     ProducerRole role,
                                     std::uint64_t session_nonce,
                                     std::uint64_t producer_instance_id,
                                     std::uint64_t run_id_hash) noexcept;
bool runtimeAtomicsAreLockFree() noexcept;

class RuntimeObservationWriter {
public:
  static RuntimeObservationWriter
  attach(const RuntimeObserverConfig &config) noexcept;
  static RuntimeObservationWriter
  attachMappedForTest(void *mapping, std::size_t mapping_size,
                      const RuntimeObserverConfig &config) noexcept;

  RuntimeObservationWriter() = default;
  ~RuntimeObservationWriter();
  RuntimeObservationWriter(const RuntimeObservationWriter &) = delete;
  RuntimeObservationWriter &
  operator=(const RuntimeObservationWriter &) = delete;
  RuntimeObservationWriter(RuntimeObservationWriter &&other) noexcept;
  RuntimeObservationWriter &
  operator=(RuntimeObservationWriter &&other) noexcept;

  bool enabled() const noexcept { return mapping_ != nullptr; }
  ProducerRole role() const noexcept { return role_; }
  std::uint64_t nextSequence() noexcept { return ++sequence_; }
  bool tryWrite(const PlannerCallbackObservationV1 &record) noexcept;
  bool tryWrite(const PpCallbackObservationV1 &record) noexcept;
  std::uint64_t observedDropCount() const noexcept;

private:
  static RuntimeObservationWriter
  validateMapped(void *mapping, std::size_t mapping_size, int owned_fd,
                 bool owns_mapping,
                 const RuntimeObserverConfig &config) noexcept;
  bool tryWriteRaw(const void *record, std::size_t record_size) noexcept;
  void invalidate() noexcept;
  void close() noexcept;

  void *mapping_{nullptr};
  std::size_t mapping_size_{0U};
  int owned_fd_{-1};
  bool owns_mapping_{false};
  ProducerRole role_{ProducerRole::kInvalid};
  ConfiguredStreamKind configured_stream_kind_{ConfiguredStreamKind::kNone};
  std::uint64_t last_read_index_{0U};
  std::uint64_t sequence_{0U};
};

class PlannerObservationScope {
public:
  PlannerObservationScope(RuntimeObservationWriter &writer,
                          std::int32_t ros_sec, std::uint32_t ros_nanosec,
                          ConfiguredStreamKind stream) noexcept;
  ~PlannerObservationScope();
  PlannerObservationScope(const PlannerObservationScope &) = delete;
  PlannerObservationScope &operator=(const PlannerObservationScope &) = delete;

  PlannerCallbackObservationV1 &record() noexcept { return record_; }
  void setReturnReason(ReturnReason reason) noexcept;

private:
  RuntimeObservationWriter *writer_{nullptr};
  PlannerCallbackObservationV1 record_{};
};

class PpObservationScope {
public:
  PpObservationScope(RuntimeObservationWriter &writer, std::int32_t ros_sec,
                     std::uint32_t ros_nanosec) noexcept;
  ~PpObservationScope();
  PpObservationScope(const PpObservationScope &) = delete;
  PpObservationScope &operator=(const PpObservationScope &) = delete;

  PpCallbackObservationV1 &record() noexcept { return record_; }
  void setReturnReason(ReturnReason reason) noexcept;

private:
  RuntimeObservationWriter *writer_{nullptr};
  PpCallbackObservationV1 record_{};
};

} // namespace overtake_transport_contract::c002ay1

#endif // OVERTAKE_TRANSPORT_CONTRACT__C002AY1_RUNTIME_OBSERVER_HPP_
