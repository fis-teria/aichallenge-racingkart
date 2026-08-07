#pragma once

#include "overtake_transport_contract/c002ay0_fixed_record.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <sys/types.h>

namespace overtake_transport_contract::c002ay0 {

constexpr std::uint64_t kFixedBaseIpcMagic = 0x4330303241593049ULL;
constexpr std::uint32_t kFixedBaseIpcAbiVersion = 1U;

struct alignas(64) FixedBaseDataRegion {
  std::uint64_t magic{0U};
  std::uint32_t abi_version{0U};
  std::uint32_t capacity{0U};
  std::uint64_t session_generation{0U};
  std::uint64_t session_nonce{0U};
  std::uint64_t controller_instance_id{0U};
  alignas(8) std::uint64_t accepting{0U};
  alignas(8) std::uint64_t write_index{0U};
  alignas(8) std::uint64_t dropped_count{0U};
  std::array<FixedBaseSlot, kFixedBaseQueueCapacity> slots{};
};

struct alignas(64) FixedBaseAckRegion {
  std::uint64_t magic{0U};
  std::uint32_t abi_version{0U};
  std::uint32_t capacity{0U};
  std::uint64_t session_generation{0U};
  std::uint64_t session_nonce{0U};
  std::uint64_t controller_instance_id{0U};
  alignas(8) std::uint64_t read_index{0U};
  alignas(8) std::uint64_t processed_count{0U};
  alignas(8) std::uint64_t validation_failure_count{0U};
  alignas(8) std::uint64_t serialization_failure_count{0U};
  alignas(8) std::uint64_t torn_count{0U};
  alignas(8) std::uint64_t worker_ready{0U};
};

bool initializeFixedBaseIpc(FixedBaseDataRegion &data, FixedBaseAckRegion &ack,
                            std::size_t capacity,
                            std::uint64_t session_generation,
                            std::uint64_t session_nonce,
                            std::uint64_t controller_instance_id) noexcept;
bool validateFixedBaseIpc(const FixedBaseDataRegion &data,
                          const FixedBaseAckRegion &ack) noexcept;
void disableFixedBaseIpc(FixedBaseDataRegion &data) noexcept;
FixedQueueResult tryPushFixedBaseIpc(FixedBaseDataRegion &data,
                                     const FixedBaseAckRegion &ack,
                                     const FixedBaseRecord &record) noexcept;
FixedQueueResult tryPopFixedBaseIpc(const FixedBaseDataRegion &data,
                                    FixedBaseAckRegion &ack,
                                    FixedBaseRecord &record) noexcept;

enum class WorkerShutdownOutcome : std::uint8_t {
  kNotStarted,
  kAlreadyExited,
  kDrained,
  kInterrupted,
  kTerminated,
  kKilled,
  kPidfdUnavailable,
  kReapUnconfirmed,
  kError,
};

struct WorkerShutdownBudget {
  std::chrono::milliseconds drain{250};
  std::chrono::milliseconds interrupt{250};
  std::chrono::milliseconds terminate{100};
  std::chrono::milliseconds kill{100};

  bool valid() const noexcept;
};

struct FixedBaseWorkerConfig {
  bool enabled{false};
  std::string executable_path;
  std::uint64_t session_generation{0U};
  std::uint64_t session_nonce{0U};
  std::uint64_t controller_instance_id{0U};
  bool source_binding_enabled{false};
  bool use_sim_time{false};
  std::chrono::milliseconds handshake_timeout{250};
  WorkerShutdownBudget shutdown_budget{};
  bool test_force_pidfd_unavailable{false};
  bool test_force_initial_cleanup_unconfirmed{false};
};

class FixedBaseWorkerSession {
public:
  static std::unique_ptr<FixedBaseWorkerSession>
  start(const FixedBaseWorkerConfig &config) noexcept;

  ~FixedBaseWorkerSession();
  FixedBaseWorkerSession(const FixedBaseWorkerSession &) = delete;
  FixedBaseWorkerSession &operator=(const FixedBaseWorkerSession &) = delete;

  bool tryCapture(const FixedBaseRecord &record) noexcept;
  bool ready() const noexcept;
  pid_t workerPid() const noexcept { return worker_pid_; }
  std::uint64_t processedCount() const noexcept;
  std::uint64_t droppedCount() const noexcept;
  WorkerShutdownOutcome shutdown() noexcept;
  WorkerShutdownOutcome shutdownOutcome() const noexcept {
    return shutdown_outcome_;
  }

private:
  FixedBaseWorkerSession() = default;
  struct Impl;
  std::unique_ptr<Impl> impl_;
  pid_t worker_pid_{-1};
  int pidfd_{-1};
  WorkerShutdownBudget shutdown_budget_{};
  WorkerShutdownOutcome shutdown_outcome_{WorkerShutdownOutcome::kNotStarted};
};

bool applyFixedBaseWorkerIsolation() noexcept;
int runFixedBaseShadowWorker(int argc, char **argv);

} // namespace overtake_transport_contract::c002ay0
