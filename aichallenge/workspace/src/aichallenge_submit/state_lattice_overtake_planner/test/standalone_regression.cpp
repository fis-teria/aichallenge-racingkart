#include "state_lattice_overtake_planner/config.hpp"
#include "state_lattice_overtake_planner/frenet_frame.hpp"
#include "state_lattice_overtake_planner/grid_map.hpp"
#include "state_lattice_overtake_planner/lattice_planner.hpp"
#include "state_lattice_overtake_planner/reference_override_contract.hpp"

#include "simple_pure_pursuit/overtake_override_contract.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace sl = state_lattice_overtake_planner;

namespace
{
class Checks
{
public:
  void expect(bool condition, const std::string & message)
  {
    if (condition) {
      ++passed_;
      return;
    }
    ++failed_;
    std::cerr << "FAIL: " << message << '\n';
  }

  int finish() const
  {
    std::cout << "standalone regression: " << passed_ << " passed, "
              << failed_ << " failed\n";
    return failed_ == 0 ? 0 : 1;
  }

private:
  int passed_{0};
  int failed_{0};
};

sl::FrenetFrame sampledSquareFrame()
{
  std::vector<sl::ReferencePoint> points;
  double s = 0.0;
  const auto append = [&points, &s](double x, double y, double yaw) {
      sl::ReferencePoint point;
      point.s = s;
      point.x = x;
      point.y = y;
      point.yaw = yaw;
      point.kappa = 0.0;
      point.speed_mps = 8.0;
      points.push_back(point);
      s += 1.0;
    };
  for (int x = 0; x < 30; ++x) {append(x, 0.0, 0.0);}
  for (int y = 0; y < 30; ++y) {append(30.0, y, M_PI_2);}
  for (int x = 30; x > 0; --x) {append(x, 30.0, M_PI);}
  for (int y = 30; y > 0; --y) {append(0.0, y, -M_PI_2);}
  sl::FrenetFrame frame;
  std::string error;
  if (!frame.setReference(std::move(points), &error)) {
    throw std::runtime_error(error);
  }
  return frame;
}

sl::GridMap openGrid(const sl::PlannerConfig & config)
{
  const auto directory = std::filesystem::temp_directory_path() /
    "state_lattice_overtake_planner_standalone";
  std::filesystem::create_directories(directory);
  const auto pgm_path = directory / "open.pgm";
  const auto yaml_path = directory / "open.yaml";
  {
    std::ofstream pgm(pgm_path, std::ios::binary);
    pgm << "P5\n600 600\n255\n";
    const std::vector<std::uint8_t> pixels(600U * 600U, 255U);
    pgm.write(
      reinterpret_cast<const char *>(pixels.data()),
      static_cast<std::streamsize>(pixels.size()));
  }
  {
    std::ofstream yaml(yaml_path);
    yaml << "image: open.pgm\n"
         << "resolution: 0.1\n"
         << "origin: [-10.0, -10.0, 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";
  }
  sl::GridMap map;
  std::string error;
  if (!map.load(yaml_path.string(), config, &error)) {
    throw std::runtime_error(error);
  }
  return map;
}

sl::EgoState egoAt(sl::FrenetFrame & frame, double x, double speed_mps, double d = 0.0)
{
  const auto cartesian = frame.frenetToCartesian(x, d);
  sl::EgoState ego;
  ego.x = cartesian.x;
  ego.y = cartesian.y;
  ego.yaw = cartesian.yaw;
  ego.speed_mps = speed_mps;
  ego.curvature = cartesian.kappa;
  ego.stamp_sec = 0.0;
  ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
  ego.valid = ego.frenet.valid;
  return ego;
}

sl::OpponentState opponentAt(
  sl::FrenetFrame & frame, const std::string & id, double s, double d = 0.0,
  double speed_mps = 0.0)
{
  const auto cartesian = frame.frenetToCartesian(s, d);
  sl::OpponentState opponent;
  opponent.id = id;
  opponent.x = cartesian.x;
  opponent.y = cartesian.y;
  opponent.yaw = cartesian.yaw;
  opponent.speed_mps = std::abs(speed_mps);
  opponent.vx_mps = std::cos(cartesian.yaw) * speed_mps;
  opponent.vy_mps = std::sin(cartesian.yaw) * speed_mps;
  opponent.stamp_sec = 0.0;
  opponent.frenet = frame.project(opponent.x, opponent.y, opponent.yaw);
  opponent.valid = opponent.frenet.valid;
  return opponent;
}

bool isOvertakeMode(sl::BehaviorMode mode)
{
  return mode == sl::BehaviorMode::OVERTAKE_LEFT ||
         mode == sl::BehaviorMode::OVERTAKE_RIGHT;
}

bool isPrepareMode(sl::BehaviorMode mode)
{
  return mode == sl::BehaviorMode::PREPARE_OVERTAKE_LEFT ||
         mode == sl::BehaviorMode::PREPARE_OVERTAKE_RIGHT;
}

void testConfigAndReferenceValidation(Checks & checks)
{
  sl::PlannerConfig config;
  checks.expect(sl::validateConfig(config).empty(), "default config must be valid");
  config.overtake_permission_lookahead_m =
    std::numeric_limits<double>::quiet_NaN();
  checks.expect(
    !sl::validateConfig(config).empty(),
    "non-finite permission lookahead must be rejected");

  auto frame = sampledSquareFrame();
  auto points = frame.points();
  points.front().kappa = std::numeric_limits<double>::quiet_NaN();
  std::string error;
  sl::FrenetFrame invalid;
  checks.expect(
    !invalid.setReference(points, &error),
    "reference with non-finite curvature must be rejected");
  points = frame.points();
  points.front().speed_mps = -1.0;
  checks.expect(
    !invalid.setReference(points, &error),
    "reference with negative speed must be rejected");
}

void testSpeedCouplingAndContract(
  Checks & checks, sl::FrenetFrame & frame, const sl::GridMap & map)
{
  sl::PlannerConfig config;
  sl::LatticePlanner high_speed(config, &frame, &map);
  auto fast_ego = egoAt(frame, 2.0, 5.0);
  auto target = opponentAt(frame, "d2", 11.0);
  const auto prepare = high_speed.update(fast_ego, {target}, true, 0.0);
  checks.expect(isPrepareMode(prepare.mode), "fast entry must prepare before lateral motion");
  checks.expect(!prepare.safe_lateral, "prepare state must be speed-only");
  checks.expect(
    prepare.speed_cap_mps <= prepare.candidate_speed_limit_mps + 1.0e-9,
    "prepare speed must not exceed candidate entry limit");
  checks.expect(
    prepare.candidate_speed_limit_mps >= config.safe_stop_speed_mps,
    "candidate must expose a usable dynamic speed limit");

  sl::LatticePlanner low_speed(config, &frame, &map);
  auto slow_ego = egoAt(frame, 2.0, 0.2);
  const auto overtake = low_speed.update(slow_ego, {target}, true, 0.0);
  checks.expect(isOvertakeMode(overtake.mode), "slow entry should execute a safe pass");
  checks.expect(overtake.safe_lateral, "executed pass must include a validated lateral horizon");
  checks.expect(overtake.lateral_offsets_m.size() == 20U, "pass horizon must contain 20 offsets");
  checks.expect(overtake.speed_caps_mps.size() == 20U, "pass horizon must contain 20 speeds");

  const auto payload = sl::makeWirePayload(overtake, 42U);
  const auto parsed = simple_pure_pursuit::parseOvertakeOverrideContract(payload.data);
  checks.expect(parsed.has_value(), "pure pursuit must parse fail-closed payload");
  checks.expect(
    parsed.has_value() &&
    parsed->kind == simple_pure_pursuit::OvertakeOverrideContractKind::INACTIVE,
    "default-off publication policy must not emit a V3 downgrade");
}

void testPermissionGate(
  Checks & checks, sl::FrenetFrame & frame, const sl::GridMap & map)
{
  sl::PlannerConfig config;
  config.overtake_permission_profile_enabled = true;
  config.default_overtake_allowed = true;
  config.overtake_permission_lookahead_m = 8.0;
  config.overtake_permission_rules.push_back({"deny_ahead", 15.0, 25.0, false});
  sl::LatticePlanner planner(config, &frame, &map);
  auto ego = egoAt(frame, 10.0, 0.2);
  auto target = opponentAt(frame, "d2", 19.0);
  const auto output = planner.update(ego, {target}, true, 0.0);
  checks.expect(
    output.mode == sl::BehaviorMode::FOLLOW_BLOCKED,
    "lookahead into a prohibited section must block a new pass");
  checks.expect(!output.overtake_permission_allowed, "permission debug flag must be false");
  checks.expect(
    output.overtake_permission_section_name == "deny_ahead",
    "permission debug must identify the blocking section");
  checks.expect(
    output.overtake_permission_reason == "lookahead_section_disallowed",
    "permission debug must identify lookahead denial");
}

void testTargetLossRecovery(
  Checks & checks, sl::FrenetFrame & frame, const sl::GridMap & map)
{
  sl::PlannerConfig config;
  config.target_missing_prediction_grace_sec = 0.20;
  config.target_missing_forget_sec = 0.60;
  config.return_required_cycles = 3;
  config.front_release_cycles = 2;
  sl::LatticePlanner planner(config, &frame, &map);
  auto ego = egoAt(frame, 2.0, 0.2);
  const auto target = opponentAt(frame, "d2", 11.0);
  auto output = planner.update(ego, {target}, true, 0.0);
  checks.expect(planner.active(), "target detection must activate planner");
  checks.expect(isOvertakeMode(output.mode), "initial target should start pass at low speed");

  bool saw_predicted_target = false;
  bool saw_yield_or_recovery = false;
  bool completed = false;
  for (int cycle = 1; cycle <= 30; ++cycle) {
    const double now = 0.1 * static_cast<double>(cycle);
    output = planner.update(ego, {}, true, now);
    saw_predicted_target = saw_predicted_target ||
      output.reason.find("predicted_target") != std::string::npos;
    saw_yield_or_recovery = saw_yield_or_recovery ||
      output.mode == sl::BehaviorMode::YIELD_BEHIND ||
      output.mode == sl::BehaviorMode::ABORT_RECOVERY;
    if (!planner.active()) {
      completed = true;
      break;
    }
  }
  checks.expect(saw_predicted_target, "brief target loss must use bounded prediction");
  checks.expect(saw_yield_or_recovery, "longer target loss must enter an explicit recovery state");
  checks.expect(completed, "missing target recovery must terminate instead of deadlocking");
  checks.expect(
    output.reason == "target_missing_recovery_complete",
    "target-loss completion must be explicit");
  checks.expect(planner.targetId().empty(), "completed recovery must clear target id");
  checks.expect(planner.candidates().empty(), "completed recovery must clear candidate history");
  checks.expect(!planner.selected().has_value(), "completed recovery must clear selected trajectory");
}

void testMpcHealthGuard(
  Checks & checks, sl::FrenetFrame & frame, const sl::GridMap & map)
{
  sl::PlannerConfig config;
  config.mpc_health_release_samples = 3;
  sl::LatticePlanner planner(config, &frame, &map);
  auto ego = egoAt(frame, 2.0, 2.0);
  sl::MpcHealthStatus health;
  health.valid = true;
  health.infeasible_count = 2;
  health.solve_time_ms = 5.0;
  health.age_sec = 0.0;
  health.sample_sequence = 1U;
  auto output = planner.update(ego, {}, true, 0.0, health);
  checks.expect(output.mode == sl::BehaviorMode::SPEED_GUARD, "MPC infeasible must latch speed guard");
  checks.expect(output.mpc_health_guard_active, "MPC guard debug flag must be true");
  checks.expect(output.speed_cap_mps <= config.mpc_health_v_max_mps, "MPC guard cap must be applied");

  health.infeasible_count = 0;
  for (std::uint64_t sequence = 2U; sequence <= 4U; ++sequence) {
    health.sample_sequence = sequence;
    output = planner.update(ego, {}, true, 0.1 * static_cast<double>(sequence), health);
  }
  checks.expect(output.mode == sl::BehaviorMode::FREE_RUN, "healthy sample hysteresis must release MPC guard");
  checks.expect(!output.active, "released MPC guard must return inactive in free run");
}

void testRearPredictionAndReverse(
  Checks & checks, sl::FrenetFrame & frame, const sl::GridMap & map)
{
  sl::PlannerConfig config;
  sl::LatticePlanner planner(config, &frame, &map);
  auto ego = egoAt(frame, 10.0, 1.0, 0.8);
  const auto path = planner.generateRearSafetyPath(ego, 0.0);
  checks.expect(path.size() == 5U, "rear safety corridor must contain five samples");
  checks.expect(
    !path.empty() && std::abs(path.front().time_sec - config.rear_prediction_horizon_sec) < 1.0e-9,
    "rear corridor point beside ego must be evaluated at merge completion time");
  checks.expect(!path.empty() && std::abs(path.back().time_sec) < 1.0e-9,
    "rear endpoint must be evaluated at current time");
  bool monotonic = true;
  for (std::size_t i = 1; i < path.size(); ++i) {
    monotonic = monotonic && path[i].time_sec < path[i - 1U].time_sec;
  }
  checks.expect(monotonic, "rear safety times must monotonically cover the merge interval");
  checks.expect(
    !path.empty() && std::abs(path.back().d) > 0.0,
    "rear corridor must spread toward the lane from which ego is returning");

  auto reverse = egoAt(frame, 10.0, -1.0);
  const auto stop = planner.update(reverse, {}, true, 0.0);
  checks.expect(stop.mode == sl::BehaviorMode::SAFE_STOP, "reverse motion must fail closed by default");
  checks.expect(stop.reason == "reverse_motion_not_allowed", "reverse stop reason must be explicit");
}

void testActualMapCoverage(
  Checks & checks, const std::filesystem::path & repository_root)
{
  const auto reference_path = repository_root /
    "multi_purpose_mpc_ros/env/final_ver3/traj_mincurv_manual.csv";
  const auto map_path = repository_root /
    "multi_purpose_mpc_ros/env/final_ver3/occupancy_grid_map.yaml";
  sl::PlannerConfig config;
  sl::FrenetFrame frame;
  sl::GridMap map;
  std::string error;
  checks.expect(frame.loadCsv(reference_path.string(), &error), "actual reference CSV must load: " + error);
  checks.expect(map.load(map_path.string(), config, &error), "actual occupancy map must load: " + error);
  checks.expect(map.buildReferenceLayer(frame, config), "actual map reference layer must build");
  if (frame.empty() || !map.initialized()) {return;}

  sl::LatticePlanner planner(config, &frame, &map);
  int zero_geometric = 0;
  int nominal_reference_conflicts = 0;
  int hard_margin_reference_conflicts = 0;
  int tracking_margin_reference_conflicts = 0;
  double elapsed_sum_ms = 0.0;
  double maximum_ms = 0.0;
  for (const auto & reference : frame.points()) {
    sl::EgoState ego;
    ego.x = reference.x;
    ego.y = reference.y;
    ego.yaw = reference.yaw;
    ego.speed_mps = 5.0;
    ego.curvature = reference.kappa;
    ego.frenet = frame.project(ego.x, ego.y, ego.yaw);
    ego.valid = ego.frenet.valid;
    sl::TrajectoryPoint reference_pose;
    reference_pose.x = reference.x;
    reference_pose.y = reference.y;
    reference_pose.yaw = reference.yaw;
    if (map.footprintHitsWall(reference_pose, sl::nominalFootprint(config))) {
      ++nominal_reference_conflicts;
    }
    if (map.footprintHitsWall(reference_pose, sl::wallFootprint(config, false))) {
      ++hard_margin_reference_conflicts;
    }
    if (map.footprintHitsWall(reference_pose, sl::wallFootprint(config, true))) {
      ++tracking_margin_reference_conflicts;
    }
    const auto started = std::chrono::steady_clock::now();
    const auto candidates = planner.generateCandidates(ego, {});
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    elapsed_sum_ms += elapsed_ms;
    maximum_ms = std::max(maximum_ms, elapsed_ms);
    const bool geometric = std::any_of(
      candidates.begin(), candidates.end(), [](const auto & candidate) {
        return candidate.feasible || candidate.requires_entry_deceleration;
      });
    if (!geometric) {++zero_geometric;}
  }
  const double average_ms = elapsed_sum_ms / static_cast<double>(frame.points().size());
  std::cout << "actual-map coverage: zero_geometric=" << zero_geometric << "/"
            << frame.points().size() << " average_ms=" << average_ms
            << " maximum_ms=" << maximum_ms
            << " reference_conflicts(nominal/hard/tracking)="
            << nominal_reference_conflicts << "/"
            << hard_margin_reference_conflicts << "/"
            << tracking_margin_reference_conflicts << '\n';
  checks.expect(
    zero_geometric <= 100,
    "actual-map geometric candidate gaps must remain below regression threshold");
  checks.expect(average_ms < 40.0, "actual-map average generation time must remain below 40 ms");
  checks.expect(maximum_ms < 120.0, "actual-map worst generation time must remain bounded");
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    Checks checks;
    testConfigAndReferenceValidation(checks);
    sl::PlannerConfig config;
    auto frame = sampledSquareFrame();
    auto map = openGrid(config);
    checks.expect(map.buildReferenceLayer(frame, config), "synthetic map reference layer must build");
    testSpeedCouplingAndContract(checks, frame, map);
    testPermissionGate(checks, frame, map);
    testTargetLossRecovery(checks, frame, map);
    testMpcHealthGuard(checks, frame, map);
    testRearPredictionAndReverse(checks, frame, map);
    if (argc >= 2) {
      testActualMapCoverage(checks, std::filesystem::path(argv[1]));
    } else {
      std::cout << "actual-map coverage skipped (repository root not supplied)\n";
    }
    return checks.finish();
  } catch (const std::exception & exception) {
    std::cerr << "FATAL: " << exception.what() << '\n';
    return 2;
  }
}
