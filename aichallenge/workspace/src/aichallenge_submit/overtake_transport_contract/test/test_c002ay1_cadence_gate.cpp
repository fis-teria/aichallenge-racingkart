#include "c002ay1_cadence_gate.hpp"

#include <gtest/gtest.h>

namespace {

namespace cadence = overtake_transport_contract::c002ay1::test_fixture;

cadence::CadenceGateInput validInput() {
  cadence::CadenceGateInput input;
  input.proposal_samples = cadence::kProposalMeasuredSamples;
  input.pp_probe_samples = cadence::kPpProbeMeasuredSamples;
  input.proposal_gap_min_ns = cadence::kProposalIntervalMinNs;
  input.proposal_gap_max_ns = cadence::kProposalIntervalMaxNs;
  input.proposal_callback_p999_ns = cadence::kCallbackBudgetNs;
  input.pp_probe_gap_max_ns = cadence::kPpProbeGapMaxNs;
  input.pp_probe_callback_p999_ns = cadence::kCallbackBudgetNs;
  return input;
}

TEST(C002Ay1CadenceGate, ExactBoundariesPass) {
  EXPECT_EQ(cadence::evaluateCadenceGate(validInput()), cadence::CADENCE_OK);
}

TEST(C002Ay1CadenceGate, ProposalBurstAndStallFailIndependently) {
  auto burst = validInput();
  --burst.proposal_gap_min_ns;
  EXPECT_EQ(cadence::evaluateCadenceGate(burst),
            cadence::PROPOSAL_CATCH_UP_BURST);

  auto stall = validInput();
  ++stall.proposal_gap_max_ns;
  EXPECT_EQ(cadence::evaluateCadenceGate(stall), cadence::PROPOSAL_TIMER_STALL);
}

TEST(C002Ay1CadenceGate, PpProbeCannotBeSubstitutedByProposalSamples) {
  auto input = validInput();
  input.pp_probe_samples = input.proposal_samples;
  EXPECT_EQ(cadence::evaluateCadenceGate(input), cadence::PP_PROBE_INCOMPLETE);
}

TEST(C002Ay1CadenceGate, PpProbeStallHasIndependentFailureBit) {
  auto input = validInput();
  ++input.pp_probe_gap_max_ns;
  EXPECT_EQ(cadence::evaluateCadenceGate(input), cadence::PP_PROBE_TIMER_STALL);
}

TEST(C002Ay1CadenceGate, CallbackAndCompletenessFailuresAccumulate) {
  auto input = validInput();
  --input.proposal_samples;
  ++input.proposal_callback_p999_ns;
  ++input.pp_probe_callback_p999_ns;
  const auto expected = cadence::PROPOSAL_INCOMPLETE |
                        cadence::PROPOSAL_CALLBACK_OVERRUN |
                        cadence::PP_PROBE_CALLBACK_OVERRUN;
  EXPECT_EQ(cadence::evaluateCadenceGate(input), expected);
}

} // namespace
