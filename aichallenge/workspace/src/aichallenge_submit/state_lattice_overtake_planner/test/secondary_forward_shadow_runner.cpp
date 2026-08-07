#include "fixtures/gate2_generation_602.hpp"

#include "state_lattice_overtake_planner/config.hpp"
#include "state_lattice_overtake_planner/frenet_frame.hpp"
#include "state_lattice_overtake_planner/grid_map.hpp"
#include "state_lattice_overtake_planner/lattice_planner.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

namespace sl = state_lattice_overtake_planner;
namespace fixture = state_lattice_overtake_planner::test_fixture;

namespace state_lattice_overtake_planner
{
struct StateLatticeTestAccess
{
  static bool outputHorizon(
    const LatticePlanner & planner, const EgoState & ego,
    const CandidateTrajectory & candidate,
    const std::vector<OpponentState> & opponents, double target_speed_mps,
    OutputHorizonDiagnostic * diagnostic)
  {
    std::vector<double> d;
    std::vector<double> speed;
    std::vector<double> longitudinal_offsets_m;
    return planner.outputHorizon(
      ego, candidate, opponents, target_speed_mps, &d, &speed,
      &longitudinal_offsets_m, diagnostic);
  }
};
}  // namespace state_lattice_overtake_planner

namespace
{

struct SnapshotFixture
{
  const char * name;
  double ego_x_m;
  double ego_y_m;
  double ego_yaw_rad;
  double ego_speed_mps;
  double ego_yaw_rate_radps;
  double ego_source_stamp_sec;
  double opponent_source_stamp_sec;
  double d2_x_m;
  double d2_y_m;
  double d3_x_m;
  double d3_y_m;
  double opponent_sigma_m;
  double continuation_goal_d_m;
  double continuation_arc_m;
  double continuation_tangent_scale;
  int observed_total_cost;
  int observed_reference_cost;
  int observed_wall_cost;
  int observed_object_cost;
};

SnapshotFixture selectedSnapshot(const std::string & name)
{
  if (name == "control") {
    using F = fixture::Gate2Generation602;
    return {"control", F::ego_x_m, F::ego_y_m, F::ego_yaw_rad,
      F::ego_speed_mps, F::ego_yaw_rate_radps, F::ego_source_stamp_sec,
      F::opponent_source_stamp_sec, F::d2_x_m, F::d2_y_m, F::d3_x_m,
      F::d3_y_m, F::opponent_covariance_m2, F::continuation_goal_d_m,
      F::continuation_arc_m, F::continuation_tangent_scale,
      F::observed_total_cost, F::observed_reference_cost,
      F::observed_wall_cost, F::observed_object_cost};
  }
  if (name == "first-eligible") {
    using F = fixture::Gate2FirstSecondaryEligible;
    return {"first-eligible", F::ego_x_m, F::ego_y_m, F::ego_yaw_rad,
      F::ego_speed_mps, F::ego_yaw_rate_radps, F::ego_source_stamp_sec,
      F::opponent_source_stamp_sec, F::d2_x_m, F::d2_y_m, F::d3_x_m,
      F::d3_y_m, F::opponent_covariance_m2, F::continuation_goal_d_m,
      F::continuation_arc_m, F::continuation_tangent_scale,
      F::observed_total_cost, F::observed_reference_cost,
      F::observed_wall_cost, F::observed_object_cost};
  }
  throw std::runtime_error("unknown snapshot: " + name);
}

sl::PlannerConfig runtimeConfig()
{
  sl::PlannerConfig config;
  config.controller_trackability_profile =
    sl::ControllerTrackabilityProfile::PURE_PURSUIT;
  config.lateral_targets_m = {0.0, 1.0, -1.0, 2.4, -2.4};
  config.tangent_scales = {0.8, 1.0, 1.2};
  config.max_steer_rate_radps = 0.5;
  config.hard_max_steer_rad = 0.64;
  config.planner_max_steer_rad = 0.64;
  return config;
}

sl::OpponentState stationaryOpponent(
  const char * id, double x_m, double y_m, const sl::FrenetFrame & frame,
  const sl::PlannerConfig & config, const SnapshotFixture & snapshot)
{
  sl::OpponentState opponent;
  opponent.id = id;
  opponent.x = x_m;
  opponent.y = y_m;
  opponent.stamp_sec = snapshot.opponent_source_stamp_sec;
  opponent.frenet = frame.project(x_m, y_m, 0.0);
  if (!opponent.frenet.valid) {
    throw std::runtime_error(std::string("failed to project opponent ") + id);
  }
  opponent.yaw = frame.interpolate(opponent.frenet.s).yaw;
  opponent.frenet = frame.project(x_m, y_m, opponent.yaw);
  // The V2X message field is historically named covariance, but the node's
  // public contract consumes it directly as sigma in metres.
  opponent.sigma_x_m = snapshot.opponent_sigma_m;
  opponent.sigma_y_m = opponent.sigma_x_m;
  opponent.uncertainty_x_m = std::clamp(
    config.sigma_multiplier * opponent.sigma_x_m,
    config.sigma_min_margin_m, config.sigma_max_margin_m);
  opponent.uncertainty_y_m = std::clamp(
    config.sigma_multiplier * opponent.sigma_y_m,
    config.sigma_min_margin_m, config.sigma_max_margin_m);
  opponent.valid = opponent.frenet.valid;
  return opponent;
}

sl::CandidateTrajectory makeCandidate(
  const sl::EgoState & ego, double terminal_arc_m, double goal_d_m,
  double tangent_scale, const sl::FrenetFrame & frame,
  const sl::PlannerConfig & config)
{
  const auto reference_goal = frame.interpolate(ego.frenet.s + terminal_arc_m);
  const double denominator = 1.0 - goal_d_m * reference_goal.kappa;
  if (!std::isfinite(denominator) || denominator <= 1.0e-3) {
    throw std::runtime_error("invalid shadow goal curvature denominator");
  }
  const auto cartesian_goal =
    frame.frenetToCartesian(ego.frenet.s + terminal_arc_m, goal_d_m);
  sl::ParametricQuintic polynomial;
  if (!polynomial.configure(
      ego, ego.curvature,
      {cartesian_goal.x, cartesian_goal.y, reference_goal.yaw},
      reference_goal.kappa / denominator, tangent_scale)) {
    throw std::runtime_error("shadow quintic configuration failed");
  }

  sl::CandidateTrajectory candidate;
  candidate.lateral_index = 4U;
  candidate.tangent_index = 1U;
  candidate.goal_d_m = goal_d_m;
  candidate.tangent_scale = tangent_scale;
  candidate.required_arc_m = terminal_arc_m;
  for (int index = 0; index < config.sampling_points; ++index) {
    candidate.representative.push_back(polynomial.sample(
      static_cast<double>(index) / (config.sampling_points - 1)));
  }
  std::vector<sl::TrajectoryPoint> coarse;
  coarse.reserve(101U);
  for (int index = 0; index <= 100; ++index) {
    coarse.push_back(polynomial.sample(static_cast<double>(index) / 100.0));
  }
  candidate.dense.push_back(coarse.front());
  for (std::size_t index = 1U; index < coarse.size(); ++index) {
    const double distance = std::hypot(
      coarse[index].x - coarse[index - 1U].x,
      coarse[index].y - coarse[index - 1U].y);
    const double yaw_change = std::abs(std::remainder(
      coarse[index].yaw - coarse[index - 1U].yaw, 2.0 * M_PI));
    const int pieces = std::max(1, static_cast<int>(std::ceil(std::max(
      distance / config.collision_max_step_m,
      yaw_change / config.collision_max_yaw_step_rad))));
    for (int piece = 1; piece <= pieces; ++piece) {
      const double ratio = static_cast<double>(piece) / pieces;
      const double u = coarse[index - 1U].u +
        (coarse[index].u - coarse[index - 1U].u) * ratio;
      candidate.dense.push_back(polynomial.sample(u));
    }
  }
  return candidate;
}

std::string candidateJson(const sl::CandidateTrajectory & candidate)
{
  std::ostringstream stream;
  stream << "{\"feasible\":" << (candidate.feasible ? "true" : "false")
         << ",\"reason\":\"" << candidate.rejection_reason << "\""
         << ",\"required_arc_m\":" << candidate.required_arc_m
         << ",\"total_cost\":" << candidate.total_cost
         << ",\"reference_cost\":" << candidate.reference_cost
         << ",\"wall_cost\":" << candidate.wall_cost
         << ",\"object_cost\":" << candidate.object_cost
         << ",\"entry_speed_limit_mps\":" << candidate.entry_speed_limit_mps
         << ",\"dynamic_speed_limit_mps\":" << candidate.dynamic_speed_limit_mps
         << "}";
  return stream.str();
}

void writeAtomic(const std::filesystem::path & path, const std::string & payload)
{
  if (path.empty()) {
    return;
  }
  const auto temporary = path.string() + ".tmp." + std::to_string(::getpid());
  {
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) {
      throw std::runtime_error("failed to open result temporary file");
    }
    output << payload << '\n';
    output.flush();
    if (!output) {
      throw std::runtime_error("failed to flush result temporary file");
    }
  }
  std::filesystem::rename(temporary, path);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc < 6 || argc > 7) {
      std::cerr << "usage: secondary_forward_shadow_runner <mpc_source_dir> "
                   "<result.json> <reference_sha256> <map_yaml_sha256> <map_pgm_sha256> "
                   "[control|first-eligible]\n";
      return 2;
    }
    const auto snapshot = selectedSnapshot(argc == 7 ? argv[6] : "control");
    const std::filesystem::path share(argv[1]);
    const auto config = runtimeConfig();
    const auto config_error = sl::validateConfig(config);
    if (!config_error.empty()) {
      throw std::runtime_error(config_error);
    }
    sl::FrenetFrame frame;
    sl::GridMap map;
    std::string error;
    if (!frame.loadCsv(
        (share / "env/final_ver3/traj_mincurv_manual.csv").string(), &error)) {
      throw std::runtime_error(error);
    }
    if (!map.load(
        (share / "env/final_ver3/occupancy_grid_map.yaml").string(), config, &error) ||
      !map.buildReferenceLayer(frame, config)) {
      throw std::runtime_error(error.empty() ? "failed to build reference layer" : error);
    }

    sl::EgoState ego;
    ego.x = snapshot.ego_x_m;
    ego.y = snapshot.ego_y_m;
    ego.yaw = snapshot.ego_yaw_rad;
    ego.speed_mps = snapshot.ego_speed_mps;
    ego.yaw_rate_radps = snapshot.ego_yaw_rate_radps;
    ego.curvature = ego.yaw_rate_radps / ego.speed_mps;
    ego.stamp_sec = snapshot.ego_source_stamp_sec;
    ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
    ego.valid = ego.frenet.valid;
    if (!ego.valid) {
      throw std::runtime_error("failed to project ego");
    }

    const auto d2 = stationaryOpponent(
      "d2", snapshot.d2_x_m, snapshot.d2_y_m, frame, config, snapshot);
    const auto d3 = stationaryOpponent(
      "d3", snapshot.d3_x_m, snapshot.d3_y_m, frame, config, snapshot);
    const std::vector<sl::OpponentState> opponents{d2, d3};
    sl::LatticePlanner planner(config, &frame, &map);

    auto baseline = makeCandidate(
      ego, snapshot.continuation_arc_m, snapshot.continuation_goal_d_m,
      snapshot.continuation_tangent_scale, frame, config);
    planner.evaluateTrajectory(&baseline, opponents, ego.speed_mps);
    sl::OutputHorizonDiagnostic output_horizon_diagnostic;
    const bool output_horizon_evaluated =
      std::string(snapshot.name) == "first-eligible" && baseline.feasible;
    const bool output_horizon_pass = output_horizon_evaluated &&
      sl::StateLatticeTestAccess::outputHorizon(
        planner, ego, baseline, opponents, config.normal_speed_mps,
        &output_horizon_diagnostic);

    const std::size_t nearest = frame.nearestIndex(ego.frenet.s);
    const double midpoint_s = 0.5 * (
      frame.unwrappedIndexS(static_cast<long long>(nearest), nearest, ego.frenet.s) +
      frame.unwrappedIndexS(
        static_cast<long long>(nearest) + config.mpc_wp_id_offset,
        nearest, ego.frenet.s));
    const double furthest_receiver_s = frame.unwrappedIndexS(
      static_cast<long long>(nearest) + config.mpc_wp_id_offset +
      config.nearest_index_uncertainty, nearest, ego.frenet.s);
    const double receiver_alignment_lead_m =
      std::max(0.0, furthest_receiver_s - midpoint_s);
    const double forward_gap_m = frame.forwardDeltaS(ego.frenet.s, d3.frenet.s);
    const double required_separation_m =
      config.wheel_base_m + config.front_overhang_m +
      config.opponent_longitudinal_tracking_margin_m + config.rear_overhang_m +
      d3.uncertainty_x_m + config.opponent_hard_clearance_m +
      config.pass_lateral_extra_margin_m;
    const double capped_arc_m = std::min(
      snapshot.continuation_arc_m,
      std::max(config.minimum_obstacle_transition_distance_m,
        forward_gap_m - required_separation_m - receiver_alignment_lead_m));

    auto shadow = makeCandidate(
      ego, capped_arc_m, snapshot.continuation_goal_d_m,
      snapshot.continuation_tangent_scale, frame, config);
    planner.evaluateTrajectory(&shadow, opponents, ego.speed_mps);

    const bool baseline_parity = baseline.feasible &&
      std::abs(baseline.total_cost - snapshot.observed_total_cost) <= 10 &&
      std::abs(baseline.reference_cost -
        snapshot.observed_reference_cost) <= 10 &&
      std::abs(baseline.wall_cost - snapshot.observed_wall_cost) <= 10 &&
      std::abs(baseline.object_cost -
        snapshot.observed_object_cost) <= 10;
    const bool secondary_forward_feasible = baseline_parity && shadow.feasible &&
      shadow.total_cost < config.stop_cost;
    const bool output_horizon_failed =
      output_horizon_evaluated && !output_horizon_pass;
    const int terminal_exit_code = !baseline_parity ? 3 :
      (output_horizon_failed ? 5 : (secondary_forward_feasible ? 0 : 4));
    const char * terminal_outcome = !baseline_parity ? "INVALID_INPUT_PARITY" :
      (output_horizon_failed ? "OUTPUT_HORIZON_FAILED" :
      (secondary_forward_feasible ? "FEASIBLE" : "NOT_FEASIBLE"));

    std::ostringstream result;
    result << std::fixed << std::setprecision(6)
           << "{\"schema\":\"secondary_forward_shadow/v1\""
           << ",\"observational_only\":true"
           << ",\"live_path_modified\":false"
           << ",\"generation\":602"
           << ",\"snapshot\":\"" << snapshot.name << "\""
           << ",\"authority_claim\":\"NOT_EVALUATED\""
           << ",\"fixture_continuation_target_id\":\"d2\""
           << ",\"secondary_forward_id\":\"d3\""
           << ",\"input_identity\":\"RECONSTRUCTED_BASELINE_PARITY\""
           << ",\"config_profile\":\"gate2-generation-602-reconstructed/v1\""
           << ",\"reference_sha256\":\"" << argv[3] << "\""
           << ",\"map_yaml_sha256\":\"" << argv[4] << "\""
           << ",\"map_pgm_sha256\":\"" << argv[5] << "\""
           << ",\"not_evaluated\":[\"target_arbitration\",\"stale_data_gate\""
           << (output_horizon_evaluated ? "" : ",\"output_horizon\"")
           << ",\"production_command_speed_state\""
           << ",\"actuator_path\"]"
           << ",\"output_horizon_test_access_only\":true"
           << ",\"output_horizon_evidence_scope\":"
              "\"FUNCTION_LEVEL_NORMAL_SPEED_INPUT_ONLY\""
           << ",\"output_horizon_target_speed_mps\":"
           << config.normal_speed_mps
           << ",\"output_horizon_evaluated\":"
           << (output_horizon_evaluated ? "true" : "false")
           << ",\"output_horizon_pass\":"
           << (output_horizon_pass ? "true" : "false")
           << ",\"output_horizon_failure\":\""
           << sl::toString(output_horizon_diagnostic.failure) << "\""
           << ",\"baseline_parity\":" << (baseline_parity ? "true" : "false")
           << ",\"secondary_forward_feasible\":"
           << (secondary_forward_feasible ? "true" : "false")
           << ",\"outcome\":\"" << terminal_outcome << "\""
           << ",\"terminal_exit_code\":" << terminal_exit_code
           << ",\"forward_gap_m\":" << forward_gap_m
           << ",\"required_separation_m\":" << required_separation_m
           << ",\"receiver_alignment_lead_m\":" << receiver_alignment_lead_m
           << ",\"baseline\":" << candidateJson(baseline)
           << ",\"shadow\":" << candidateJson(shadow) << "}";
    const auto payload = result.str();
    writeAtomic(argv[2], payload);
    std::cout << payload << '\n';
    return terminal_exit_code;
  } catch (const std::exception & exception) {
    std::cerr << "secondary-forward shadow runner failed: " << exception.what() << '\n';
    return 2;
  }
}
