#pragma once

#include "state_lattice_overtake_planner/c002ay0_shadow_proposal_record.hpp"

#include "overtake_transport_contract/c002ay0_worker_ipc.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace state_lattice_overtake_planner::c002ay0_shadow {

struct FixedProposalWorkerConfig {
  bool enabled{false};
  std::string executable_path;
  std::uint64_t session_generation{0U};
  std::uint64_t session_nonce{0U};
  FixedProposalStaticConfig static_config{};
  std::chrono::milliseconds handshake_timeout{250};
  overtake_transport_contract::c002ay0::WorkerShutdownBudget shutdown_budget{};
};

// Read-only, diagnostic-only state exported through the planner debug artifact.
// None of these fields grant authority or feed any command/control path.
struct FixedProposalWorkerDiagnostics {
  bool accepting{false};
  bool worker_ready{false};
  std::uint64_t dropped_full_count{0U};
  std::uint64_t invalid_record_count{0U};
  std::uint64_t validation_reject_count{0U};
  std::uint64_t serialization_reject_count{0U};
  std::uint64_t coalesced_count{0U};
  std::uint64_t published_count{0U};
  std::uint64_t worker_process_age_ns{0U};
  std::uint64_t last_publish_queue_age_ns{0U};
};

class FixedProposalWorkerSession {
public:
  static std::unique_ptr<FixedProposalWorkerSession>
  start(const FixedProposalWorkerConfig &config) noexcept;

  ~FixedProposalWorkerSession();
  FixedProposalWorkerSession(const FixedProposalWorkerSession &) = delete;
  FixedProposalWorkerSession &
  operator=(const FixedProposalWorkerSession &) = delete;

  bool tryCapture(const FixedProposalRecord &record) noexcept;
  bool ready() const noexcept;
  std::uint64_t droppedCount() const noexcept;
  std::uint64_t publishedCount() const noexcept;
  std::uint64_t validationRejectCount() const noexcept;
  FixedProposalWorkerDiagnostics diagnostics() const noexcept;
  overtake_transport_contract::c002ay0::WorkerShutdownOutcome
  shutdown() noexcept;

private:
  FixedProposalWorkerSession() = default;
  struct Impl;
  std::unique_ptr<Impl> impl_;
  int data_fd_{-1};
  int worker_pid_{-1};
  overtake_transport_contract::c002ay0::WorkerShutdownBudget shutdown_budget_{};
  overtake_transport_contract::c002ay0::WorkerShutdownOutcome shutdown_outcome_{
      overtake_transport_contract::c002ay0::WorkerShutdownOutcome::kNotStarted};
};

int runFixedProposalShadowWorker(int argc, char **argv);

} // namespace state_lattice_overtake_planner::c002ay0_shadow
