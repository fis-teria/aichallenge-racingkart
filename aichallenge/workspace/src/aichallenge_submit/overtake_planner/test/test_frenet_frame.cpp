#include "overtake_planner/frenet_frame.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace
{

overtake_planner::FrenetFrame makeStraightFrame()
{
  std::vector<overtake_planner::ReferencePoint> ref;
  for (int i = 0; i <= 10; ++i) {
    ref.push_back(overtake_planner::ReferencePoint{
      static_cast<double>(i), static_cast<double>(i), 0.0, 0.0, 0.0, 5.0});
  }
  overtake_planner::FrenetFrame frame;
  frame.setReference(ref);
  return frame;
}

}  // namespace

TEST(FrenetFrame, CartesianToFrenetStraightHasExpectedOffsetSign)
{
  const auto frame = makeStraightFrame();

  const auto center = frame.cartesianToFrenet(3.0, 0.0, 0.0);
  const auto left = frame.cartesianToFrenet(3.0, 0.8, 0.0);
  const auto right = frame.cartesianToFrenet(3.0, -0.8, 0.0);

  EXPECT_NEAR(center.d, 0.0, 1.0e-9);
  EXPECT_NEAR(left.d, 0.8, 1.0e-9);
  EXPECT_NEAR(right.d, -0.8, 1.0e-9);
}

TEST(FrenetFrame, DeltaSWrapsForwardAroundClosedLoop)
{
  const auto frame = makeStraightFrame();

  EXPECT_NEAR(frame.deltaS(8.0, 2.0), frame.length() - 6.0, 1.0e-9);
  EXPECT_NEAR(frame.deltaS(2.0, 8.0), 6.0, 1.0e-9);
}

TEST(FrenetFrame, FrenetToCartesianRoundTripIsSmallOnStraight)
{
  const auto frame = makeStraightFrame();

  const auto cart = frame.frenetToCartesian(4.0, 0.5);
  const auto frenet = frame.cartesianToFrenet(cart.x, cart.y, cart.yaw);

  EXPECT_NEAR(frenet.s, 4.0, 0.6);
  EXPECT_NEAR(frenet.d, 0.5, 1.0e-9);
}
