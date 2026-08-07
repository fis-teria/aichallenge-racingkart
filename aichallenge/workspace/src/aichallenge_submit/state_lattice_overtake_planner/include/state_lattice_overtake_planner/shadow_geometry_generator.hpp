#pragma once

#include "state_lattice_overtake_planner/types.hpp"

#include <string>
#include <vector>

namespace state_lattice_overtake_planner
{

// Shadow-only request.  target/side/target_d are supplied by the owning
// overtake Core; this component never detects or latches an opponent.
struct ShadowGeometryRequest
{
  Pose2d start{};
  double start_curvature{0.0};
  Pose2d goal{};
  double goal_curvature{0.0};
  double tangent_scale{1.0};
  std::string target_id{};
  int pass_side{0};
  double target_d_m{0.0};
  std::size_t sample_count{41U};
};

struct ShadowGeometryResult
{
  bool valid{false};
  std::string reason{"not_generated"};
  std::vector<TrajectoryPoint> dense{};
  double raw_cost{0.0};
};

// Pure Cartesian geometry only: no map, detector, safety evaluation, ROS
// node, publisher, or generation state is reachable from this class.
class ShadowGeometryGenerator
{
public:
  ShadowGeometryResult generate(const ShadowGeometryRequest & request) const;
};

}  // namespace state_lattice_overtake_planner
