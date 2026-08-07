#pragma once

#include "dev3_20260727_225451.hpp"

#include <array>

namespace state_lattice_overtake_planner::test_fixture {

// Last FREE_RUN snapshot before D3 first entered current_pose_hard_collision
// in output/20260728-120212. The collision on the following planner cycle was
// classified as WALL. Raw bags remain outside source control.
inline constexpr PreHardParallelSnapshot kD3PreWallFreeRun{
    {"d3", 89639.4157425458, 43139.4063045593, -0.6679149554, 2.0175698362},
    {{
        {"d1", 89622.15625, 43144.74609375, 0.0, 0.0},
        {"d2", 89635.453125, 43140.8359375, 0.0, 0.0},
    }},
    0.15,
    0.15,
    "",
};

} // namespace state_lattice_overtake_planner::test_fixture
