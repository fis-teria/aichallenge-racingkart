#ifndef OVERTAKE_TRANSPORT_CONTRACT__TEST__C002AY1_CADENCE_GATE_HPP_
#define OVERTAKE_TRANSPORT_CONTRACT__TEST__C002AY1_CADENCE_GATE_HPP_

#include <cstddef>
#include <cstdint>

namespace overtake_transport_contract::c002ay1::test_fixture {

constexpr std::uint64_t kProposalIntervalMinNs = 45000000ULL;
constexpr std::uint64_t kProposalIntervalMaxNs = 60000000ULL;
constexpr std::uint64_t kPpProbeGapMaxNs = 12000000ULL;
constexpr std::uint64_t kCallbackBudgetNs = 1000000ULL;
constexpr std::size_t kProposalMeasuredSamples = 2000U;
constexpr std::size_t kPpProbeMeasuredSamples = 10000U;

enum CadenceFailure : std::uint32_t {
  CADENCE_OK = 0U,
  PROPOSAL_INCOMPLETE = 1U << 0U,
  PROPOSAL_CATCH_UP_BURST = 1U << 1U,
  PROPOSAL_TIMER_STALL = 1U << 2U,
  PROPOSAL_CALLBACK_OVERRUN = 1U << 3U,
  PP_PROBE_INCOMPLETE = 1U << 4U,
  PP_PROBE_TIMER_STALL = 1U << 5U,
  PP_PROBE_CALLBACK_OVERRUN = 1U << 6U,
};

struct CadenceGateInput {
  std::size_t proposal_samples{0U};
  std::size_t pp_probe_samples{0U};
  std::uint64_t proposal_gap_min_ns{0U};
  std::uint64_t proposal_gap_max_ns{0U};
  std::uint64_t proposal_callback_p999_ns{0U};
  std::uint64_t pp_probe_gap_max_ns{0U};
  std::uint64_t pp_probe_callback_p999_ns{0U};
};

inline std::uint32_t
evaluateCadenceGate(const CadenceGateInput &input) noexcept {
  std::uint32_t failures = CADENCE_OK;
  if (input.proposal_samples != kProposalMeasuredSamples) {
    failures |= PROPOSAL_INCOMPLETE;
  }
  if (input.proposal_gap_min_ns < kProposalIntervalMinNs) {
    failures |= PROPOSAL_CATCH_UP_BURST;
  }
  if (input.proposal_gap_max_ns > kProposalIntervalMaxNs) {
    failures |= PROPOSAL_TIMER_STALL;
  }
  if (input.proposal_callback_p999_ns > kCallbackBudgetNs) {
    failures |= PROPOSAL_CALLBACK_OVERRUN;
  }
  if (input.pp_probe_samples != kPpProbeMeasuredSamples) {
    failures |= PP_PROBE_INCOMPLETE;
  }
  if (input.pp_probe_gap_max_ns > kPpProbeGapMaxNs) {
    failures |= PP_PROBE_TIMER_STALL;
  }
  if (input.pp_probe_callback_p999_ns > kCallbackBudgetNs) {
    failures |= PP_PROBE_CALLBACK_OVERRUN;
  }
  return failures;
}

} // namespace overtake_transport_contract::c002ay1::test_fixture

#endif // OVERTAKE_TRANSPORT_CONTRACT__TEST__C002AY1_CADENCE_GATE_HPP_
