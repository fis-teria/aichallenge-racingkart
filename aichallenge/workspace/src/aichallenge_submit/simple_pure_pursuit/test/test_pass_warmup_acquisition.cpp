#include "simple_pure_pursuit/pass_warmup_acquisition.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace simple_pure_pursuit {

TEST(PassWarmupSteeringAcquisition, SaturatedStepNeverGrantsMotion) {
  const auto result = evaluatePassWarmupSteeringAcquisition(
      -0.095, -0.005, -0.095, -0.005, 0.01, 0.10, 1.0e-4, 0.002);
  EXPECT_TRUE(result.evidence_valid);
  EXPECT_TRUE(result.acquisition_required);
  EXPECT_FALSE(result.measured_steering_converged);
  EXPECT_FALSE(result.motion_ready);
}

TEST(PassWarmupSteeringAcquisition,
     FinalUnboundedCommandRequiresMeasuredConvergence) {
  const auto lagging = evaluatePassWarmupSteeringAcquisition(
      -0.095, -0.095, -0.095, -0.060, 0.01, 0.10, 1.0e-4, 0.002);
  EXPECT_TRUE(lagging.evidence_valid);
  EXPECT_FALSE(lagging.acquisition_required);
  EXPECT_FALSE(lagging.measured_steering_converged);
  EXPECT_FALSE(lagging.motion_ready);

  const auto converged = evaluatePassWarmupSteeringAcquisition(
      -0.095, -0.095, -0.095, -0.094, 0.01, 0.10, 1.0e-4, 0.002);
  EXPECT_TRUE(converged.evidence_valid);
  EXPECT_FALSE(converged.acquisition_required);
  EXPECT_TRUE(converged.measured_steering_converged);
  EXPECT_TRUE(converged.motion_ready);
}

TEST(PassWarmupSteeringAcquisition,
     OutputGainDoesNotMovePhysicalTrackingTarget) {
  const auto result = evaluatePassWarmupSteeringAcquisition(
      -0.274, -0.274, -0.178, -0.182, 0.01, 0.10, 1.0e-4, 0.008);
  EXPECT_TRUE(result.evidence_valid);
  EXPECT_FALSE(result.acquisition_required);
  EXPECT_TRUE(result.measured_steering_converged);
  EXPECT_TRUE(result.motion_ready);
}

TEST(PassWarmupSteeringAcquisition, StaleOrNonfiniteEvidenceFailsClosed) {
  const auto stale = evaluatePassWarmupSteeringAcquisition(
      0.1, 0.1, 0.1, 0.1, 0.11, 0.10, 1.0e-4, 0.002);
  EXPECT_FALSE(stale.evidence_valid);
  EXPECT_FALSE(stale.motion_ready);

  const auto nonfinite = evaluatePassWarmupSteeringAcquisition(
      0.1, 0.1, 0.1, std::numeric_limits<double>::quiet_NaN(), 0.01,
      0.10, 1.0e-4, 0.002);
  EXPECT_FALSE(nonfinite.evidence_valid);
  EXPECT_FALSE(nonfinite.motion_ready);
}

} // namespace simple_pure_pursuit
