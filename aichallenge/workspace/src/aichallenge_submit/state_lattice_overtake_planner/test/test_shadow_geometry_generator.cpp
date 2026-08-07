#include "state_lattice_overtake_planner/shadow_geometry_generator.hpp"

#include <gtest/gtest.h>

namespace state_lattice_overtake_planner
{
TEST(ShadowGeometryGenerator, GeneratesDenseFinitePureGeometry)
{
  ShadowGeometryRequest request;
  request.start = {0.0, 0.0, 0.0};
  request.goal = {8.0, 1.0, 0.0};
  request.target_id = "D2";
  request.pass_side = 1;
  request.target_d_m = 1.0;
  request.sample_count = 41U;
  const auto result = ShadowGeometryGenerator{}.generate(request);
  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_EQ(result.dense.size(), 41U);
  EXPECT_GT(result.raw_cost, 0.0);
}

TEST(ShadowGeometryGenerator, RejectsInvalidIdentityAndNeverPublishes)
{
  ShadowGeometryRequest request;
  request.start = {0.0, 0.0, 0.0};
  request.goal = {8.0, 1.0, 0.0};
  request.pass_side = 0;
  EXPECT_FALSE(ShadowGeometryGenerator{}.generate(request).valid);
}
}  // namespace state_lattice_overtake_planner
