#pragma once

#include "simple_state_lattice_planner/costmap_builder.hpp"
#include "simple_state_lattice_planner/types.hpp"

#include <string>

namespace simple_state_lattice_planner {

bool loadReferenceCsv(const std::string &path, ReferenceWindow *reference,
                      std::string *error);
bool loadOccupancyGridYaml(const std::string &path, Costmap2D *costmap,
                           std::string *error);

}  // namespace simple_state_lattice_planner
