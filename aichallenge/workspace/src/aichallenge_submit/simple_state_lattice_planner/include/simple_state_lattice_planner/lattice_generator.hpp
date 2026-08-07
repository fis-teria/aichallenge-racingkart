#pragma once

#include "simple_state_lattice_planner/types.hpp"

namespace simple_state_lattice_planner {

LatticeResult generateLatticeCandidates(const ReferenceWindow &reference,
                                        const EgoState &ego,
                                        const LatticeConfig &config,
                                        double snapshot_time_sec);

LatticeResult generateOvertakeLatticeCandidates(
    const ReferenceWindow &reference, const EgoState &ego,
    const LatticeConfig &config, double snapshot_time_sec,
    double merge_start_forward_m);

}  // namespace simple_state_lattice_planner
