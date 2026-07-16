#include "overtake_planner/frenet_frame.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
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

overtake_planner::FrenetFrame makeSquareFrame()
{
  constexpr double kHalfPi = 1.5707963267948966;
  std::vector<overtake_planner::ReferencePoint> ref{
    {0.0, 0.0, 0.0, 0.0, 0.0, 5.0},
    {10.0, 10.0, 0.0, 0.0, 0.0, 5.0},
    {20.0, 10.0, 10.0, kHalfPi, 0.0, 5.0},
    {30.0, 0.0, 10.0, -kHalfPi, 0.0, 5.0},
  };
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

TEST(FrenetFrame, CartesianToFrenetProjectsOntoReferenceSegment)
{
  const auto frame = makeSquareFrame();

  const auto frenet = frame.cartesianToFrenet(5.4, 0.25, 0.0);

  EXPECT_NEAR(frenet.s, 5.4, 1.0e-9);
  EXPECT_NEAR(frenet.d, 0.25, 1.0e-9);
}

TEST(FrenetFrame, ClosingSegmentInterpolatesAndRoundTrips)
{
  const auto frame = makeSquareFrame();

  const auto point = frame.interpolate(35.0);
  EXPECT_NEAR(point.x, 0.0, 1.0e-9);
  EXPECT_NEAR(point.y, 5.0, 1.0e-9);

  const auto frenet = frame.cartesianToFrenet(point.x, point.y, point.yaw);
  EXPECT_NEAR(frenet.s, 35.0, 1.0e-9);
  EXPECT_NEAR(frenet.d, 0.0, 1.0e-9);
}

TEST(FrenetFrame, CorridorUsesNarrowerAdjacentBoundary)
{
  auto frame = makeSquareFrame();
  frame.setCorridor({
    {0.0, -3.0, 4.0},
    {10.0, -2.0, 3.0},
    {20.0, -4.0, 5.0},
    {30.0, -3.0, 4.0},
  });

  const auto bounds = frame.corridorBounds(5.0, -1.0, 1.0);
  EXPECT_NEAR(bounds.d_min, -2.0, 1.0e-9);
  EXPECT_NEAR(bounds.d_max, 3.0, 1.0e-9);
}

TEST(FrenetFrame, CorridorLoaderRejectsAdjacentDiscontinuity)
{
  auto frame = makeSquareFrame();
  const auto path = std::filesystem::temp_directory_path() /
                    "overtake_planner_adjacent_corridor_test.csv";
  {
    std::ofstream output(path);
    output << "s_m,d_min_m,d_max_m\n"
           << "0,-3,3\n"
           << "10,-3,3\n"
           << "20,4,5\n"
           << "30,4,5\n";
  }

  std::string error;
  EXPECT_FALSE(frame.loadCorridorCsv(path.string(), &error));
  EXPECT_NE(error.find("adjacent corridor bounds do not overlap"),
            std::string::npos);

  std::error_code remove_error;
  std::filesystem::remove(path, remove_error);
}

TEST(FrenetFrame, CorridorLoaderRejectsClosingSeamDiscontinuity)
{
  auto frame = makeSquareFrame();
  const auto path = std::filesystem::temp_directory_path() /
                    "overtake_planner_closing_corridor_test.csv";
  {
    std::ofstream output(path);
    output << "s_m,d_min_m,d_max_m\n"
           << "0,0,1\n"
           << "10,0.5,1.5\n"
           << "20,1,2\n"
           << "30,1.5,2.5\n";
  }

  std::string error;
  EXPECT_FALSE(frame.loadCorridorCsv(path.string(), &error));
  EXPECT_NE(error.find("closing corridor bounds do not overlap"),
            std::string::npos);

  std::error_code remove_error;
  std::filesystem::remove(path, remove_error);
}
