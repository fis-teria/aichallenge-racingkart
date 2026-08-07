#include "overtake_transport_contract/state_lattice_v2_binding.hpp"
#include "state_lattice_overtake_planner/c002ay0_shadow_proposal.hpp"
#include "state_lattice_overtake_planner/c002ay0_shadow_proposal_capture.hpp"
#include "state_lattice_overtake_planner/c002ay0_shadow_proposal_worker.hpp"
#include "state_lattice_overtake_planner/config.hpp"
#include "state_lattice_overtake_planner/cost_model.hpp"
#include "state_lattice_overtake_planner/frenet_frame.hpp"
#include "state_lattice_overtake_planner/grid_map.hpp"
#include "state_lattice_overtake_planner/instant_controller.hpp"
#include "state_lattice_overtake_planner/lattice_planner.hpp"
#include "state_lattice_overtake_planner/reference_override_contract.hpp"
#include "state_lattice_overtake_planner/state_lattice_v2_final_fence.hpp"
#include "state_lattice_overtake_planner/state_lattice_v2_publication_state.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/authorized_cartesian_trajectory_v2.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/controller_base_trajectory_snapshot.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/state_lattice_control_command.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/state_lattice_v2_base_attestation.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/state_lattice_v2_final_fence.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/state_lattice_v2_quiesce_request.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <v2x_msgs/msg/v2_x_vehicle_position_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <time.h>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace state_lattice_overtake_planner {
namespace {
StateLatticeV2SemanticKey makeStateLatticeV2SemanticKey(
    const multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectory
        &trajectory,
    const std::string &producer_instance_id, const std::string &session_id,
    BehaviorMode planner_mode, SolverHorizonIntent planner_intent) {
  StateLatticeV2SemanticKey key;
  key.v2_schema_version = multi_purpose_mpc_ros_msgs::msg::
      AuthorizedCartesianTrajectoryV2::SCHEMA_V2_NON_AUTHORITATIVE;
  key.producer_instance_id = producer_instance_id;
  key.session_id = session_id;
  key.identity_source_generation = trajectory.base_source_generation;
  key.identity_source_stamp_sec = trajectory.base_source_stamp.sec;
  key.identity_source_stamp_nanosec = trajectory.base_source_stamp.nanosec;
  key.frame_id = trajectory.frame_id;
  key.trajectory_schema_version = trajectory.schema_version;
  key.authority_eligible = trajectory.authority_eligible;
  key.race_arm_epoch = trajectory.plan_sample_key.race_arm_epoch;
  key.planner_instance_id = trajectory.plan_sample_key.planner_instance_id;
  key.target_id = trajectory.plan_sample_key.target_vehicle_id;
  key.pass_direction = trajectory.plan_sample_key.pass_direction;
  key.planner_mode = static_cast<int>(planner_mode);
  key.planner_intent = static_cast<int>(planner_intent);
  key.candidate_type = trajectory.candidate_type;
  key.phase = trajectory.phase;
  key.authorization_state = trajectory.authorization_state;
  key.source_controller_instance_id = trajectory.source_controller_instance_id;
  key.source_controller_sequence = trajectory.source_controller_sequence;
  key.base_lease_id = trajectory.base_lease_id;
  key.base_lease_valid_until_sec = trajectory.base_lease_valid_until.sec;
  key.base_lease_valid_until_nanosec =
      trajectory.base_lease_valid_until.nanosec;
  key.base_source_kind = trajectory.base_source_kind;
  key.base_source_stamp_sec = trajectory.base_source_stamp.sec;
  key.base_source_stamp_nanosec = trajectory.base_source_stamp.nanosec;
  key.base_source_generation = trajectory.base_source_generation;
  key.base_original_point_count = trajectory.base_original_point_count;
  key.base_first_source_index = trajectory.base_first_source_index;
  key.base_last_source_index = trajectory.base_last_source_index;
  key.base_nearest_source_index = trajectory.base_nearest_source_index;
  key.base_source_digest_state = trajectory.base_source_digest_state;
  key.canonical_algorithm_version = trajectory.canonical_algorithm_version;
  key.base_geometry_sha256 = trajectory.base_geometry_sha256;
  key.base_source_sha256 = trajectory.base_source_sha256;
  key.base_snapshot_sha256 = trajectory.base_snapshot_sha256;
  key.geometry_sha256 = trajectory.geometry_sha256;
  key.original_candidate_point_count =
      trajectory.original_candidate_point_count;
  key.total_arc_length_m = trajectory.total_arc_length_m;
  key.required_spatial_horizon_m = trajectory.required_spatial_horizon_m;
  key.join_end_arc_length_m = trajectory.join_end_arc_length_m;
  key.post_join_arc_length_m = trajectory.post_join_arc_length_m;
  key.safety_evaluation_result = trajectory.safety_evaluation_result;
  key.safety_valid_until_sec = trajectory.safety_valid_until.sec;
  key.safety_valid_until_nanosec = trajectory.safety_valid_until.nanosec;
  key.world_safety_snapshot_sha256 = trajectory.world_safety_snapshot_sha256;
  key.safety_evaluator_implementation_sha256 =
      trajectory.safety_evaluator_implementation_sha256;
  key.safety_evaluator_config_sha256 =
      trajectory.safety_evaluator_config_sha256;
  key.safety_proof_sha256 = trajectory.safety_proof_sha256;
  key.controller_implementation_sha256 =
      trajectory.controller_implementation_sha256;
  key.controller_config_sha256 = trajectory.controller_config_sha256;
  key.candidate_start_control_pose_sha256 =
      trajectory.candidate_start_control_pose_sha256;
  return key;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

geometry_msgs::msg::Point markerPoint(double x, double y, double z = 0.0) {
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

void appendRectangle(visualization_msgs::msg::Marker *marker, double min_x,
                     double min_y, double max_x, double max_y, double z) {
  marker->points.push_back(markerPoint(min_x, min_y, z));
  marker->points.push_back(markerPoint(max_x, min_y, z));
  marker->points.push_back(markerPoint(max_x, max_y, z));
  marker->points.push_back(markerPoint(min_x, max_y, z));
  marker->points.push_back(markerPoint(min_x, min_y, z));
}

void appendCapsuleOutline(visualization_msgs::msg::Marker *marker,
                          const Pose2d &start, const Pose2d &end, double radius,
                          double z) {
  constexpr int arc_segments = 16;
  constexpr double pi = 3.14159265358979323846;
  const double heading = std::atan2(end.y - start.y, end.x - start.x);
  if (std::hypot(end.x - start.x, end.y - start.y) < 1.0e-6) {
    for (int i = 0; i <= 2 * arc_segments; ++i) {
      const double angle =
          2.0 * pi * static_cast<double>(i) / (2 * arc_segments);
      marker->points.push_back(markerPoint(start.x + radius * std::cos(angle),
                                           start.y + radius * std::sin(angle),
                                           z));
    }
    return;
  }
  for (int i = 0; i <= arc_segments; ++i) {
    const double angle =
        heading + 0.5 * pi - pi * static_cast<double>(i) / arc_segments;
    marker->points.push_back(markerPoint(end.x + radius * std::cos(angle),
                                         end.y + radius * std::sin(angle), z));
  }
  for (int i = 0; i <= arc_segments; ++i) {
    const double angle =
        heading - 0.5 * pi - pi * static_cast<double>(i) / arc_segments;
    marker->points.push_back(markerPoint(start.x + radius * std::cos(angle),
                                         start.y + radius * std::sin(angle),
                                         z));
  }
  marker->points.push_back(marker->points.front());
}

std::string packagePath(const std::string &package,
                        const std::string &relative) {
  if (!relative.empty() && relative.front() == '/') {
    return relative;
  }
  return ament_index_cpp::get_package_share_directory(package) + "/" + relative;
}

std::string ay0StateLatticeShadowWorkerPath() {
  try {
    return ament_index_cpp::get_package_prefix(
               "state_lattice_overtake_planner") +
           "/lib/state_lattice_overtake_planner/"
           "c002ay0_state_lattice_shadow_worker";
  } catch (...) {
    return {};
  }
}

std::uint64_t monotonicNanoseconds() noexcept {
  timespec stamp{};
  if (clock_gettime(CLOCK_MONOTONIC, &stamp) != 0 || stamp.tv_sec < 0 ||
      stamp.tv_nsec < 0) {
    return 0U;
  }
  return static_cast<std::uint64_t>(stamp.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(stamp.tv_nsec);
}

bool reliablePlannerInputAuditEnabled() noexcept {
  const char *value = std::getenv("C002AY0_TEST_RELIABLE_PLANNER_INPUT_AUDIT");
  return value != nullptr && std::strcmp(value, "1") == 0;
}

std::string resolveOwnVehicleId(const std::string &configured) {
  if (!configured.empty() && configured != "auto") {
    return configured;
  }
  const char *domain = std::getenv("ROS_DOMAIN_ID");
  if (domain == nullptr) {
    return "d1";
  }
  try {
    const int id = std::stoi(domain);
    return id > 0 ? "d" + std::to_string(id) : "d1";
  } catch (...) {
    return "d1";
  }
}

std::optional<double> jsonNumberField(const std::string &json,
                                      const std::string &field) {
  const std::string key = "\"" + field + "\":";
  const auto key_position = json.find(key);
  if (key_position == std::string::npos) {
    return std::nullopt;
  }
  std::size_t position = key_position + key.size();
  while (position < json.size() &&
         std::isspace(static_cast<unsigned char>(json[position])) != 0) {
    ++position;
  }
  if (position >= json.size()) {
    return std::nullopt;
  }
  errno = 0;
  char *end = nullptr;
  const double value = std::strtod(json.c_str() + position, &end);
  if (errno != 0 || end == json.c_str() + position || !std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

std::string trim(std::string value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1U);
}

std::vector<std::string> splitCsvLine(const std::string &line) {
  std::vector<std::string> columns;
  std::stringstream stream(line);
  std::string column;
  while (std::getline(stream, column, ',')) {
    columns.push_back(trim(column));
  }
  return columns;
}

std::optional<std::int64_t> parseInt64(const std::string &value) {
  errno = 0;
  char *end = nullptr;
  const long long parsed = std::strtoll(value.c_str(), &end, 10);
  if (errno != 0 || end == value.c_str() || *end != '\0') {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(parsed);
}

std::optional<bool> parseBool(const std::string &value) {
  std::string lowered;
  lowered.reserve(value.size());
  std::transform(value.begin(), value.end(), std::back_inserter(lowered),
                 [](unsigned char character) {
                   return static_cast<char>(std::tolower(character));
                 });
  if (lowered == "true" || lowered == "1" || lowered == "yes" ||
      lowered == "allow") {
    return true;
  }
  if (lowered == "false" || lowered == "0" || lowered == "no" ||
      lowered == "deny") {
    return false;
  }
  return std::nullopt;
}
} // namespace

class StateLatticeOvertakePlannerNode : public rclcpp::Node {
public:
  StateLatticeOvertakePlannerNode()
      : Node("state_lattice_overtake_planner_node"), tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    loadParameters();
    std::random_device random_device;
    instant_control_producer_instance_id_ =
        (static_cast<std::uint64_t>(random_device()) << 32U) ^
        static_cast<std::uint64_t>(random_device());
    if (instant_control_producer_instance_id_ == 0U) {
      instant_control_producer_instance_id_ = 1U;
    }
    planner_session_id_ = instant_control_producer_instance_id_;
    own_vehicle_id_ = resolveOwnVehicleId(config_.own_vehicle_id);
    // The core resolves side-role ties from a total vehicle-ID order. Give it
    // the same launch/domain-resolved identity used by V2X self filtering
    // before validation and construction; "auto" is never role authority.
    config_.own_vehicle_id = own_vehicle_id_;
    std::string config_error = validateConfig(config_);
    if (config_error.empty()) {
      config_error = validateControllerTrackabilityProfileContract(
          config_.controller_trackability_profile, live_control_output_enabled_,
          instant_control_enabled_);
    }
    if (config_error.empty() && instant_control_enabled_ &&
        (!std::isfinite(instant_controller_config_.lookahead_min_m) ||
         instant_controller_config_.lookahead_min_m <= 0.0 ||
         !std::isfinite(instant_controller_config_.lookahead_gain_sec) ||
         instant_controller_config_.lookahead_gain_sec < 0.0 ||
         !std::isfinite(
             instant_controller_config_.maximum_steering_angle_rad) ||
         instant_controller_config_.maximum_steering_angle_rad <= 0.0 ||
         instant_controller_config_.maximum_steering_angle_rad >
             config_.hard_max_steer_rad ||
         !std::isfinite(
             instant_controller_config_.maximum_steering_rate_radps) ||
         instant_controller_config_.maximum_steering_rate_radps <= 0.0 ||
         instant_controller_config_.maximum_steering_rate_radps >
             config_.max_steer_rate_radps)) {
      config_error = "invalid instant control parameters";
    }
    if (config_error.empty()) {
      applyResolvedControllerTrackabilityEnvelope(
          &config_, instant_controller_config_.maximum_steering_angle_rad,
          instant_controller_config_.maximum_steering_rate_radps);
    }
    if (config_error.empty() && live_control_output_enabled_ &&
        !config_.safety_evaluation_enabled) {
      config_error =
          "live control output requires safety_evaluation_enabled=true";
    }
    if (!config_error.empty()) {
      degraded_reason_ = "invalid_parameter: " + config_error;
      RCLCPP_ERROR(get_logger(), "%s", degraded_reason_.c_str());
    } else {
      loadStaticInputs();
      if (map_.initialized() && !frame_.empty()) {
        ay0_shadow_safety_evidence_.evaluator_implementation_sha256 =
            stateLatticeSafetyEvaluatorImplementationDigest();
        ay0_shadow_safety_evidence_.evaluator_config_sha256 =
            stateLatticeSafetyEvaluatorConfigDigest(config_, map_, frame_);
      }
    }
    planner_ = std::make_unique<LatticePlanner>(config_, &frame_, &map_,
                                                &output_frame_);
    instant_controller_ =
        std::make_unique<InstantController>(instant_controller_config_);

    const auto control_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    const auto static_debug_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    override_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        config_.override_topic, control_qos);
    instant_control_pub_ = create_publisher<
        multi_purpose_mpc_ros_msgs::msg::StateLatticeControlCommand>(
        instant_control_topic_, control_qos);
    if (state_lattice_v2_live_proposal_publish_enabled_ &&
        degraded_reason_.empty() && config_.safety_evaluation_enabled) {
      state_lattice_v2_proposal_pub_ = create_publisher<
          multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectoryV2>(
          "/planning/overtake/state_lattice/v2_proposal",
          rclcpp::QoS(rclcpp::KeepLast(8)).reliable().durability_volatile());
    } else if (state_lattice_v2_live_proposal_publish_enabled_) {
      RCLCPP_WARN(get_logger(), "State Lattice V2 proposal publisher disabled: "
                                "planner safety contract is unavailable");
      state_lattice_v2_live_proposal_publish_enabled_ = false;
    }
    if (state_lattice_v2_live_proposal_publish_enabled_) {
      const auto v2_qos =
          rclcpp::QoS(rclcpp::KeepLast(8)).reliable().durability_volatile();
      state_lattice_v2_base_attestation_sub_ = create_subscription<
          multi_purpose_mpc_ros_msgs::msg::StateLatticeV2BaseAttestation>(
          "/control/overtake/state_lattice/v2_base_attestation", v2_qos,
          [this](const multi_purpose_mpc_ros_msgs::msg::
                     StateLatticeV2BaseAttestation::SharedPtr message) {
            onStateLatticeV2BaseAttestation(*message);
          });
    }
    if (!state_lattice_v2_live_proposal_publish_enabled_) {
      state_lattice_v2_final_fence_enabled_ = false;
    }
    state_lattice_v2_final_fence_state_ =
        std::make_unique<StateLatticeV2FinalFenceState>(
            StateLatticeV2FinalFenceState::Config{
                state_lattice_v2_final_fence_enabled_,
                state_lattice_v2_final_fence_execution_nonce_,
                std::to_string(state_lattice_v2_producer_instance_id_),
                state_lattice_v2_final_fence_expected_session_id_,
                state_lattice_v2_final_fence_sealed_epoch_id_,
                "topic=/planning/overtake/state_lattice/v2_proposal;"
                "type=multi_purpose_mpc_ros_msgs/msg/"
                "AuthorizedCartesianTrajectoryV2;reliability=reliable;"
                "durability=volatile;history=keep_last;depth=8"});
    if (state_lattice_v2_final_fence_enabled_) {
      const auto final_fence_qos =
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
      state_lattice_v2_final_fence_pub_ = create_publisher<
          multi_purpose_mpc_ros_msgs::msg::StateLatticeV2FinalFence>(
          "/test/m4/state_lattice/v2_final_fence", final_fence_qos);
      state_lattice_v2_quiesce_sub_ = create_subscription<
          multi_purpose_mpc_ros_msgs::msg::StateLatticeV2QuiesceRequest>(
          "/test/m4/state_lattice/v2_quiesce", final_fence_qos,
          [this](const multi_purpose_mpc_ros_msgs::msg::
                     StateLatticeV2QuiesceRequest::SharedPtr message) {
            onStateLatticeV2QuiesceRequest(*message);
          });
    }
    if (ay0_shadow_capture_enabled_ && reliablePlannerInputAuditEnabled()) {
      ay0_planner_input_audit_pub_ = create_publisher<std_msgs::msg::String>(
          "/test/c002ay0/state_lattice/planner_input_audit",
          rclcpp::QoS(rclcpp::KeepLast(1024)).reliable().durability_volatile());
    }
    if (ay0_shadow_capture_enabled_ && degraded_reason_.empty()) {
      c002ay0_shadow::FixedProposalWorkerConfig worker_config;
      worker_config.enabled = true;
      worker_config.executable_path = ay0_shadow_worker_path_;
      worker_config.session_generation = ay0_shadow_session_generation_;
      worker_config.session_nonce = ay0_shadow_session_nonce_;
      worker_config.static_config.safety_evaluation_enabled = 1U;
      worker_config.static_config.frame_size = 3U;
      worker_config.static_config.frame[0] = 'm';
      worker_config.static_config.frame[1] = 'a';
      worker_config.static_config.frame[2] = 'p';
      worker_config.static_config.wheel_base_m = config_.wheel_base_m;
      worker_config.static_config.ego_stale_sec = config_.ego_stale_sec;
      worker_config.static_config.evaluator_implementation_sha256 =
          ay0_shadow_safety_evidence_.evaluator_implementation_sha256;
      worker_config.static_config.evaluator_config_sha256 =
          ay0_shadow_safety_evidence_.evaluator_config_sha256;
      ay0_shadow_worker_session_ =
          c002ay0_shadow::FixedProposalWorkerSession::start(worker_config);
      if (ay0_shadow_worker_session_ == nullptr ||
          !ay0_shadow_worker_session_->ready()) {
        ay0_shadow_capture_enabled_ = false;
        RCLCPP_WARN(get_logger(), "State Lattice AY0 shadow worker "
                                  "unavailable; motion path continues");
      }
    }
    mode_pub_ = create_publisher<std_msgs::msg::String>("/debug/overtake/mode",
                                                        control_qos);
    metrics_pub_ = create_publisher<std_msgs::msg::String>(
        "/debug/overtake/metrics", control_qos);
    wall_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/debug/overtake/wall_map", control_qos);
    wall_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/debug/overtake/wall_costmap", static_debug_qos);
    opponent_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/debug/overtake/opponent_costmap", control_qos);
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/debug/overtake/costmap", control_qos);
    candidates_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/debug/overtake/trajectory_candidates", control_qos);
    selected_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/debug/overtake/selected_trajectory", control_qos);
    targets_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/debug/overtake/target_states", control_qos);
    rear_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/debug/overtake/rear_safety_paths", control_qos);
    geometry_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/debug/overtake/planning_geometry", control_qos);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        config_.ego_topic, control_qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
          onOdometry(*message);
        });
    v2x_sub_ = create_subscription<v2x_msgs::msg::V2XVehiclePositionArray>(
        config_.opponent_topic, control_qos,
        [this](
            const v2x_msgs::msg::V2XVehiclePositionArray::SharedPtr message) {
          onV2x(*message);
        });
    if (ay0_shadow_capture_enabled_) {
      const auto shadow_qos =
          rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
      ay0_base_snapshot_sub_ = create_subscription<
          multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot>(
          "/control/overtake/base_trajectory_snapshot", shadow_qos,
          [this](const multi_purpose_mpc_ros_msgs::msg::
                     ControllerBaseTrajectorySnapshot::SharedPtr message) {
            onAy0BaseSnapshot(*message);
          });
    }
    mpc_sub_ = create_subscription<std_msgs::msg::String>(
        config_.mpc_health_topic, control_qos,
        [this](const std_msgs::msg::String::SharedPtr message) {
          onMpcHealth(*message);
        });

    const double planner_rate_hz =
        std::isfinite(config_.planner_rate_hz) && config_.planner_rate_hz > 0.0
            ? config_.planner_rate_hz
            : 20.0;
    const auto period = std::chrono::duration<double>(1.0 / planner_rate_hz);
    planning_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { onPlanningTimer(); });
    const double debug_rate_hz = std::isfinite(config_.debug_costmap_rate_hz) &&
                                         config_.debug_costmap_rate_hz > 0.0
                                     ? config_.debug_costmap_rate_hz
                                     : 5.0;
    const auto debug_period =
        std::chrono::duration<double>(1.0 / debug_rate_hz);
    debug_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(debug_period),
        [this]() { publishCostmap(); });
    if (!config_.safety_evaluation_enabled) {
      RCLCPP_ERROR(
          get_logger(),
          "UNSAFE DEBUG MODE: all environmental safety evaluation is disabled");
    }
    if (instant_control_enabled_ && !config_.safety_evaluation_enabled) {
      RCLCPP_ERROR(get_logger(), "state_lattice instant control requested with "
                                 "safety evaluation disabled; "
                                 "commands will remain ineligible");
    }
    RCLCPP_INFO(
        get_logger(),
        "state lattice overtake planner started: own_vehicle_id=%s degraded=%s "
        "live_control_output=%s instant_control=%s trackability_profile=%s "
        "safety_evaluation=%s",
        own_vehicle_id_.c_str(), degraded_reason_.empty() ? "false" : "true",
        live_control_output_enabled_ ? "true" : "false",
        instant_control_enabled_ ? "true" : "false",
        toString(config_.controller_trackability_profile),
        config_.safety_evaluation_enabled ? "true" : "false");
  }

private:
  void loadParameters() {
    enabled_ = declare_parameter<bool>("enabled", true);
    live_control_output_enabled_ =
        declare_parameter<bool>("live_control_output_enabled", false);
    instant_control_enabled_ =
        declare_parameter<bool>("instant_control_enabled", false);
    state_lattice_v2_live_proposal_publish_enabled_ = declare_parameter<bool>(
        "state_lattice_v2_live_proposal_publish_enabled", false);
    const auto state_lattice_v2_producer_instance_id =
        declare_parameter<std::int64_t>("state_lattice_v2_producer_instance_id",
                                        0);
    state_lattice_v2_producer_instance_id_ =
        state_lattice_v2_producer_instance_id > 0
            ? static_cast<std::uint64_t>(state_lattice_v2_producer_instance_id)
            : 0U;
    state_lattice_v2_base_attestation_accept_enabled_ = declare_parameter<bool>(
        "state_lattice_v2_base_attestation_accept_enabled", false);
    state_lattice_v2_expected_pp_producer_instance_id_ =
        declare_parameter<std::string>(
            "state_lattice_v2_expected_pp_producer_instance_id", "");
    state_lattice_v2_expected_pp_session_id_ = declare_parameter<std::string>(
        "state_lattice_v2_expected_pp_session_id", "");
    state_lattice_v2_final_fence_enabled_ =
        declare_parameter<bool>("state_lattice_v2_final_fence_enabled", false);
    state_lattice_v2_final_fence_execution_nonce_ =
        declare_parameter<std::string>(
            "state_lattice_v2_final_fence_execution_nonce", "");
    state_lattice_v2_final_fence_expected_session_id_ =
        declare_parameter<std::string>(
            "state_lattice_v2_final_fence_expected_session_id", "");
    const auto final_fence_epoch = declare_parameter<std::int64_t>(
        "state_lattice_v2_final_fence_sealed_epoch_id", 0);
    state_lattice_v2_final_fence_sealed_epoch_id_ =
        final_fence_epoch > 0 ? static_cast<std::uint64_t>(final_fence_epoch)
                              : 0U;
    if (state_lattice_v2_final_fence_enabled_ &&
        (!state_lattice_v2_live_proposal_publish_enabled_ ||
         state_lattice_v2_final_fence_execution_nonce_.empty() ||
         state_lattice_v2_final_fence_expected_session_id_.empty() ||
         state_lattice_v2_final_fence_sealed_epoch_id_ == 0U)) {
      state_lattice_v2_final_fence_enabled_ = false;
      RCLCPP_WARN(get_logger(),
                  "State Lattice V2 final fence disabled: explicit test "
                  "epoch binding and live proposal publisher are required");
    }
    if (state_lattice_v2_live_proposal_publish_enabled_ &&
        (state_lattice_v2_producer_instance_id_ == 0U ||
         !state_lattice_v2_base_attestation_accept_enabled_ ||
         state_lattice_v2_expected_pp_producer_instance_id_.empty() ||
         state_lattice_v2_expected_pp_session_id_.empty())) {
      state_lattice_v2_live_proposal_publish_enabled_ = false;
      RCLCPP_WARN(get_logger(),
                  "State Lattice V2 publisher disabled: planner and direct "
                  "PP attestation identities must be explicit");
    }
    config_.controller_trackability_profile =
        parseControllerTrackabilityProfile(declare_parameter<std::string>(
            "controller_trackability_profile", "unknown"));
    instant_control_topic_ = declare_parameter<std::string>(
        "instant_control_topic", "/hybrid_control/state_lattice/control_cmd");
    ay0_shadow_capture_enabled_ =
        declare_parameter<bool>("c002ay0_state_lattice_shadow_enabled", false);
    ay0_shadow_worker_path_ = declare_parameter<std::string>(
        "c002ay0_state_lattice_shadow_worker_path",
        ay0StateLatticeShadowWorkerPath());
    const auto ay0_generation = declare_parameter<std::int64_t>(
        "c002ay0_state_lattice_shadow_session_generation", 0);
    const auto ay0_nonce = declare_parameter<std::int64_t>(
        "c002ay0_state_lattice_shadow_session_nonce", 0);
    ay0_shadow_session_generation_ =
        ay0_generation > 0 ? static_cast<std::uint64_t>(ay0_generation) : 0U;
    ay0_shadow_session_nonce_ =
        ay0_nonce > 0 ? static_cast<std::uint64_t>(ay0_nonce) : 0U;
    config_.safety_evaluation_enabled =
        declare_parameter<bool>("safety_evaluation_enabled", false);
    config_.experimental_exact_spatial_follow_shadow_enabled =
        declare_parameter<bool>(
            "experimental_exact_spatial_follow_shadow_enabled", true);
    experimental_spatial_reference_override_live_publish_enabled_ =
        declare_parameter<bool>(
            "experimental_spatial_reference_override_live_publish_enabled",
            false);
    if (experimental_spatial_reference_override_live_publish_enabled_ &&
        !config_.experimental_exact_spatial_follow_shadow_enabled) {
      experimental_spatial_reference_override_live_publish_enabled_ = false;
      RCLCPP_WARN(
          get_logger(),
          "spatial reference override live publisher disabled: exact-spatial "
          "generation must also be enabled");
    }
    const char *safety_override =
        std::getenv("STATE_LATTICE_SAFETY_EVALUATION_ENABLED");
    if (safety_override != nullptr && !trim(safety_override).empty()) {
      const auto parsed_override = parseBool(trim(safety_override));
      if (parsed_override.has_value()) {
        config_.safety_evaluation_enabled = parsed_override.value();
        set_parameter(rclcpp::Parameter("safety_evaluation_enabled",
                                        config_.safety_evaluation_enabled));
        RCLCPP_WARN(
            get_logger(),
            "STATE_LATTICE_SAFETY_EVALUATION_ENABLED overrides YAML: %s",
            config_.safety_evaluation_enabled ? "true" : "false");
      } else {
        RCLCPP_WARN(get_logger(),
                    "ignoring invalid "
                    "STATE_LATTICE_SAFETY_EVALUATION_ENABLED='%s'; using YAML",
                    safety_override);
      }
    }
    if (ay0_shadow_capture_enabled_ &&
        (!config_.safety_evaluation_enabled || ay0_generation <= 0 ||
         ay0_nonce <= 0 || ay0_shadow_worker_path_.empty())) {
      ay0_shadow_capture_enabled_ = false;
      RCLCPP_WARN(get_logger(), "State Lattice AY0 shadow capture disabled: "
                                "invalid safety/session/path");
    }
    const auto cost_levels = declare_parameter<std::vector<int64_t>>(
        "cost_levels", {6, 9, 12, 15, 18, 60, 70, 80, 90, 100});
    config_.cost_levels.assign(cost_levels.begin(), cost_levels.end());
    config_.wall_distance_thresholds_m = declare_parameter<std::vector<double>>(
        "wall_distance_thresholds_m", config_.wall_distance_thresholds_m);
    config_.object_distance_thresholds_m =
        declare_parameter<std::vector<double>>(
            "object_distance_thresholds_m",
            config_.object_distance_thresholds_m);
    config_.reference_distance_thresholds_m =
        declare_parameter<std::vector<double>>(
            "reference_distance_thresholds_m",
            config_.reference_distance_thresholds_m);
    config_.reference_extra_step_m = declare_parameter<double>(
        "reference_extra_step_m", config_.reference_extra_step_m);
    config_.sigma_multiplier = declare_parameter<double>(
        "opponent_position_sigma_multiplier", config_.sigma_multiplier);
    config_.sigma_min_margin_m = declare_parameter<double>(
        "opponent_position_min_margin_m", config_.sigma_min_margin_m);
    config_.sigma_max_margin_m = declare_parameter<double>(
        "opponent_position_max_margin_m", config_.sigma_max_margin_m);
    config_.reference_package = declare_parameter<std::string>(
        "reference_package", config_.reference_package);
    config_.reference_csv =
        declare_parameter<std::string>("reference_csv", config_.reference_csv);
    config_.output_reference_package = declare_parameter<std::string>(
        "output_reference_package", config_.output_reference_package);
    config_.output_reference_csv = declare_parameter<std::string>(
        "output_reference_csv", config_.output_reference_csv);
    config_.wall_map_package = declare_parameter<std::string>(
        "wall_map_package", config_.wall_map_package);
    config_.wall_map_yaml = declare_parameter<std::string>(
        "wall_map_yaml_relative_path", config_.wall_map_yaml);
    config_.map_frame =
        declare_parameter<std::string>("map_frame_id", config_.map_frame);
    config_.expected_map_resolution_m = declare_parameter<double>(
        "expected_map_resolution", config_.expected_map_resolution_m);
    config_.overtake_permission_profile_enabled =
        declare_parameter<bool>("overtake_permission_profile_enabled",
                                config_.overtake_permission_profile_enabled);
    config_.overtake_permission_package = declare_parameter<std::string>(
        "overtake_permission_package", config_.overtake_permission_package);
    config_.overtake_permission_csv = declare_parameter<std::string>(
        "overtake_permission_csv", config_.overtake_permission_csv);
    config_.default_overtake_allowed = declare_parameter<bool>(
        "default_overtake_allowed", config_.default_overtake_allowed);
    config_.overtake_permission_lookahead_m =
        declare_parameter<double>("overtake_permission_lookahead_m",
                                  config_.overtake_permission_lookahead_m);
    config_.wheel_base_m =
        declare_parameter<double>("wheel_base_m", config_.wheel_base_m);
    config_.front_overhang_m =
        declare_parameter<double>("front_overhang_m", config_.front_overhang_m);
    config_.rear_overhang_m =
        declare_parameter<double>("rear_overhang_m", config_.rear_overhang_m);
    config_.left_extent_m =
        declare_parameter<double>("left_extent_m", config_.left_extent_m);
    config_.right_extent_m =
        declare_parameter<double>("right_extent_m", config_.right_extent_m);
    config_.wall_hard_margin_m = declare_parameter<double>(
        "wall_hard_margin_m", config_.wall_hard_margin_m);
    config_.opponent_hard_clearance_m = declare_parameter<double>(
        "opponent_hard_clearance_m", config_.opponent_hard_clearance_m);
    config_.lateral_targets_m = declare_parameter<std::vector<double>>(
        "lateral_target_offsets_m", config_.lateral_targets_m);
    config_.tangent_scales = declare_parameter<std::vector<double>>(
        "tangent_scale_factors", config_.tangent_scales);
    config_.sampling_points =
        declare_parameter<int>("num_sampling_points", config_.sampling_points);
    config_.base_distance_m =
        declare_parameter<double>("base_dist_param", config_.base_distance_m);
    config_.max_distance_m =
        declare_parameter<double>("max_dist_param", config_.max_distance_m);
    config_.max_distance_speed_mps = declare_parameter<double>(
        "max_dist_speed_mps", config_.max_distance_speed_mps);
    config_.minimum_lateral_transition_distance_m = declare_parameter<double>(
        "minimum_lateral_transition_distance_m",
        config_.minimum_lateral_transition_distance_m);
    config_.lateral_transition_distance_gain =
        declare_parameter<double>("lateral_transition_distance_gain",
                                  config_.lateral_transition_distance_gain);
    config_.adaptive_pass_offset_enabled = declare_parameter<bool>(
        "adaptive_pass_offset_enabled", config_.adaptive_pass_offset_enabled);
    config_.pass_lateral_extra_margin_m = declare_parameter<double>(
        "pass_lateral_extra_margin_m", config_.pass_lateral_extra_margin_m);
    config_.max_adaptive_lateral_offset_m = declare_parameter<double>(
        "max_adaptive_lateral_offset_m", config_.max_adaptive_lateral_offset_m);
    config_.minimum_obstacle_transition_distance_m = declare_parameter<double>(
        "minimum_obstacle_transition_distance_m",
        config_.minimum_obstacle_transition_distance_m);
    config_.front_detection_radius_m = declare_parameter<double>(
        "front_detection_radius_m", config_.front_detection_radius_m);
    config_.frontmost_s_tolerance_m = declare_parameter<double>(
        "frontmost_s_tolerance_m", config_.frontmost_s_tolerance_m);
    config_.front_enter_cycles = declare_parameter<int>(
        "front_detection_enter_cycles", config_.front_enter_cycles);
    config_.front_release_cycles = declare_parameter<int>(
        "front_detection_release_cycles", config_.front_release_cycles);
    config_.early_aware_enabled = declare_parameter<bool>(
        "early_aware_enabled", config_.early_aware_enabled);
    config_.early_aware_base_distance_m = declare_parameter<double>(
        "early_aware_base_distance_m", config_.early_aware_base_distance_m);
    config_.early_aware_speed_horizon_sec = declare_parameter<double>(
        "early_aware_speed_horizon_sec", config_.early_aware_speed_horizon_sec);
    config_.early_aware_max_deceleration_mps2 =
        declare_parameter<double>("early_aware_max_deceleration_mps2",
                                  config_.early_aware_max_deceleration_mps2);
    config_.early_aware_min_distance_m = declare_parameter<double>(
        "early_aware_min_distance_m", config_.early_aware_min_distance_m);
    config_.early_aware_max_distance_m = declare_parameter<double>(
        "early_aware_max_distance_m", config_.early_aware_max_distance_m);
    config_.early_aware_min_tangent_speed_mps =
        declare_parameter<double>("early_aware_min_tangent_speed_mps",
                                  config_.early_aware_min_tangent_speed_mps);
    config_.early_aware_reverse_tangent_tolerance_mps =
        declare_parameter<double>(
            "early_aware_reverse_tangent_tolerance_mps",
            config_.early_aware_reverse_tangent_tolerance_mps);
    config_.early_aware_max_track_heading_error_rad = declare_parameter<double>(
        "early_aware_max_track_heading_error_rad",
        config_.early_aware_max_track_heading_error_rad);
    config_.early_aware_min_forward_delta_s_m =
        declare_parameter<double>("early_aware_min_forward_delta_s_m",
                                  config_.early_aware_min_forward_delta_s_m);
    config_.early_aware_required_fresh_stamps =
        declare_parameter<int>("early_aware_required_fresh_stamps",
                               config_.early_aware_required_fresh_stamps);
    config_.initial_detection_sweep_enabled =
        declare_parameter<bool>("initial_detection_sweep_enabled",
                                config_.initial_detection_sweep_enabled);
    config_.return_required_cycles = declare_parameter<int>(
        "free_run_return_required_cycles", config_.return_required_cycles);
    config_.passed_target_gap_m = declare_parameter<double>(
        "passed_target_gap_m", config_.passed_target_gap_m);
    config_.return_prediction_sec = declare_parameter<double>(
        "return_prediction_horizon_sec", config_.return_prediction_sec);
    config_.return_predicted_gap_m = declare_parameter<double>(
        "return_predicted_min_gap_m", config_.return_predicted_gap_m);
    config_.return_lateral_error_m = declare_parameter<double>(
        "return_lateral_error_m", config_.return_lateral_error_m);
    config_.return_heading_error_rad = declare_parameter<double>(
        "return_heading_error_rad", config_.return_heading_error_rad);
    config_.rear_safety_distance_m = declare_parameter<double>(
        "rear_safety_check_distance_m", config_.rear_safety_distance_m);
    config_.rear_terminal_distance_m = declare_parameter<double>(
        "rear_terminal_distance", config_.rear_terminal_distance_m);
    config_.rear_sampling_angle_rad = declare_parameter<double>(
        "rear_sampling_angle", config_.rear_sampling_angle_rad);
    config_.rear_sampling_points = declare_parameter<int>(
        "rear_safety_num_sampling_points", config_.rear_sampling_points);
    config_.rear_return_cost_threshold = declare_parameter<int>(
        "rear_return_cost_threshold", config_.rear_return_cost_threshold);
    config_.rear_prediction_horizon_sec = declare_parameter<double>(
        "rear_prediction_horizon_sec", config_.rear_prediction_horizon_sec);
    config_.target_missing_prediction_grace_sec =
        declare_parameter<double>("target_missing_prediction_grace_sec",
                                  config_.target_missing_prediction_grace_sec);
    config_.target_missing_forget_sec = declare_parameter<double>(
        "target_missing_forget_sec", config_.target_missing_forget_sec);
    config_.target_missing_uncertainty_growth_mps = declare_parameter<double>(
        "target_missing_uncertainty_growth_mps",
        config_.target_missing_uncertainty_growth_mps);
    config_.target_missing_recovery_speed_mps =
        declare_parameter<double>("target_missing_recovery_speed_mps",
                                  config_.target_missing_recovery_speed_mps);
    config_.follow_desired_extra_gap_m = declare_parameter<double>(
        "follow_desired_extra_gap_m", config_.follow_desired_extra_gap_m);
    config_.follow_gap_gain_per_s = declare_parameter<double>(
        "follow_gap_gain_per_s", config_.follow_gap_gain_per_s);
    config_.follow_closing_speed_gain = declare_parameter<double>(
        "follow_closing_speed_gain", config_.follow_closing_speed_gain);
    config_.preventive_side_role_trigger_clearance_m =
        declare_parameter<double>(
            "preventive_side_role_trigger_clearance_m",
            config_.preventive_side_role_trigger_clearance_m);
    config_.preventive_side_role_tie_band_m =
        declare_parameter<double>("preventive_side_role_tie_band_m",
                                  config_.preventive_side_role_tie_band_m);
    config_.preventive_side_role_max_abs_delta_s_m = declare_parameter<double>(
        "preventive_side_role_max_abs_delta_s_m",
        config_.preventive_side_role_max_abs_delta_s_m);
    config_.preventive_side_role_min_lateral_separation_m =
        declare_parameter<double>(
            "preventive_side_role_min_lateral_separation_m",
            config_.preventive_side_role_min_lateral_separation_m);
    config_.preventive_side_role_min_tangent_progress_mps =
        declare_parameter<double>(
            "preventive_side_role_min_tangent_progress_mps",
            config_.preventive_side_role_min_tangent_progress_mps);
    config_.preventive_side_role_max_track_heading_error_rad =
        declare_parameter<double>(
            "preventive_side_role_max_track_heading_error_rad",
            config_.preventive_side_role_max_track_heading_error_rad);
    config_.preventive_side_role_max_relative_heading_error_rad =
        declare_parameter<double>(
            "preventive_side_role_max_relative_heading_error_rad",
            config_.preventive_side_role_max_relative_heading_error_rad);
    config_.preventive_side_role_separation_epsilon_m =
        declare_parameter<double>(
            "preventive_side_role_separation_epsilon_m",
            config_.preventive_side_role_separation_epsilon_m);
    config_.preventive_side_role_follower_speed_reduction_mps =
        declare_parameter<double>(
            "preventive_side_role_follower_speed_reduction_mps",
            config_.preventive_side_role_follower_speed_reduction_mps);
    config_.preventive_side_role_follower_gap_gain_per_s =
        declare_parameter<double>(
            "preventive_side_role_follower_gap_gain_per_s",
            config_.preventive_side_role_follower_gap_gain_per_s);
    config_.preventive_side_role_follower_min_gap_m = declare_parameter<double>(
        "preventive_side_role_follower_min_gap_m",
        config_.preventive_side_role_follower_min_gap_m);
    config_.preventive_side_role_controller_response_sec =
        declare_parameter<double>(
            "preventive_side_role_controller_response_sec",
            config_.preventive_side_role_controller_response_sec);
    config_.preventive_side_role_deadline_response_margin_sec =
        declare_parameter<double>(
            "preventive_side_role_deadline_response_margin_sec",
            config_.preventive_side_role_deadline_response_margin_sec);
    config_.preventive_side_role_neutral_speed_reduction_mps =
        declare_parameter<double>(
            "preventive_side_role_neutral_speed_reduction_mps",
            config_.preventive_side_role_neutral_speed_reduction_mps);
    config_.preventive_side_role_role_confirm_samples = declare_parameter<int>(
        "preventive_side_role_role_confirm_samples",
        config_.preventive_side_role_role_confirm_samples);
    config_.preventive_side_role_prediction_horizon_sec =
        declare_parameter<double>(
            "preventive_side_role_prediction_horizon_sec",
            config_.preventive_side_role_prediction_horizon_sec);
    config_.preventive_side_role_prediction_step_sec =
        declare_parameter<double>(
            "preventive_side_role_prediction_step_sec",
            config_.preventive_side_role_prediction_step_sec);
    config_.preventive_side_role_yield_peer_speed_mps =
        declare_parameter<double>(
            "preventive_side_role_yield_peer_speed_mps",
            config_.preventive_side_role_yield_peer_speed_mps);
    config_.preventive_side_role_yield_min_samples =
        declare_parameter<int>("preventive_side_role_yield_min_samples",
                               config_.preventive_side_role_yield_min_samples);
    config_.preventive_side_role_yield_min_span_sec = declare_parameter<double>(
        "preventive_side_role_yield_min_span_sec",
        config_.preventive_side_role_yield_min_span_sec);
    config_.preventive_side_role_release_margin_m = declare_parameter<double>(
        "preventive_side_role_release_margin_m",
        config_.preventive_side_role_release_margin_m);
    config_.preventive_side_role_release_prediction_sec =
        declare_parameter<double>(
            "preventive_side_role_release_prediction_sec",
            config_.preventive_side_role_release_prediction_sec);
    config_.preventive_side_role_escape_crawl_speed_mps =
        declare_parameter<double>(
            "preventive_side_role_escape_crawl_speed_mps",
            config_.preventive_side_role_escape_crawl_speed_mps);
    config_.preventive_side_role_exit_clearance_m = declare_parameter<double>(
        "preventive_side_role_exit_clearance_m",
        config_.preventive_side_role_exit_clearance_m);
    config_.preventive_side_role_exit_samples =
        declare_parameter<int>("preventive_side_role_exit_samples",
                               config_.preventive_side_role_exit_samples);
    config_.hard_max_steer_rad = declare_parameter<double>(
        "hard_max_steer_rad", config_.hard_max_steer_rad);
    config_.planner_max_steer_rad = declare_parameter<double>(
        "planner_max_steer_rad", config_.planner_max_steer_rad);
    config_.max_steer_rate_radps = declare_parameter<double>(
        "max_steer_rate_radps", config_.max_steer_rate_radps);
    config_.min_acceleration_mps2 = declare_parameter<double>(
        "min_acceleration_mps2", config_.min_acceleration_mps2);
    config_.max_acceleration_mps2 = declare_parameter<double>(
        "max_acceleration_mps2", config_.max_acceleration_mps2);
    config_.max_acceleration_jerk_mps3 = declare_parameter<double>(
        "max_acceleration_jerk_mps3", config_.max_acceleration_jerk_mps3);
    config_.max_deceleration_jerk_mps3 = declare_parameter<double>(
        "max_deceleration_jerk_mps3", config_.max_deceleration_jerk_mps3);
    config_.lateral_acceleration_limit_mps2 =
        declare_parameter<double>("lateral_acceleration_limit_mps2",
                                  config_.lateral_acceleration_limit_mps2);
    config_.collision_max_step_m = declare_parameter<double>(
        "collision_check_max_step_m", config_.collision_max_step_m);
    config_.collision_max_yaw_step_rad = declare_parameter<double>(
        "collision_check_max_yaw_step_rad", config_.collision_max_yaw_step_rad);
    config_.lateral_tracking_margin_m = declare_parameter<double>(
        "lateral_tracking_margin_m", config_.lateral_tracking_margin_m);
    config_.longitudinal_tracking_margin_m =
        declare_parameter<double>("longitudinal_tracking_margin_m",
                                  config_.longitudinal_tracking_margin_m);
    config_.opponent_lateral_tracking_margin_m =
        declare_parameter<double>("opponent_lateral_tracking_margin_m",
                                  config_.opponent_lateral_tracking_margin_m);
    config_.opponent_longitudinal_tracking_margin_m = declare_parameter<double>(
        "opponent_longitudinal_tracking_margin_m",
        config_.opponent_longitudinal_tracking_margin_m);
    config_.candidate_entry_speed_tolerance_mps =
        declare_parameter<double>("candidate_entry_speed_tolerance_mps",
                                  config_.candidate_entry_speed_tolerance_mps);
    config_.allow_reverse =
        declare_parameter<bool>("allow_reverse", config_.allow_reverse);
    config_.reference_speed_limit_enabled = declare_parameter<bool>(
        "reference_speed_limit_enabled", config_.reference_speed_limit_enabled);
    config_.reference_curvature_sanity_limit_radpm = declare_parameter<double>(
        "reference_curvature_sanity_limit_radpm",
        config_.reference_curvature_sanity_limit_radpm);
    config_.horizon_points =
        declare_parameter<int>("horizon_points", config_.horizon_points);
    config_.mpc_wp_id_offset =
        declare_parameter<int>("mpc_wp_id_offset", config_.mpc_wp_id_offset);
    config_.nearest_index_uncertainty =
        declare_parameter<int>("receiver_nearest_index_uncertainty",
                               config_.nearest_index_uncertainty);
    config_.projection_initial_half_width_m =
        declare_parameter<double>("projection_initial_search_half_width_m",
                                  config_.projection_initial_half_width_m);
    config_.projection_follow_half_width_m =
        declare_parameter<double>("projection_follow_search_half_width_m",
                                  config_.projection_follow_half_width_m);
    config_.projection_max_backward_m = declare_parameter<double>(
        "projection_max_backward_step_m", config_.projection_max_backward_m);
    config_.tie_break_epsilon = declare_parameter<double>(
        "tie_break_epsilon", config_.tie_break_epsilon);
    config_.normal_speed_mps =
        declare_parameter<double>("normal_speed_mps", config_.normal_speed_mps);
    config_.free_run_return_cost = declare_parameter<int>(
        "free_run_return_cost", config_.free_run_return_cost);
    config_.stop_cost = declare_parameter<int>("stop_cost", config_.stop_cost);
    config_.safe_stop_speed_mps = declare_parameter<double>(
        "safe_stop_compat_speed_mps", config_.safe_stop_speed_mps);
    config_.own_vehicle_id = declare_parameter<std::string>(
        "own_vehicle_id", config_.own_vehicle_id);
    config_.ego_topic =
        declare_parameter<std::string>("ego_state_topic", config_.ego_topic);
    config_.opponent_topic = declare_parameter<std::string>(
        "opponent_topic", config_.opponent_topic);
    config_.mpc_health_topic = declare_parameter<std::string>(
        "mpc_health_topic", config_.mpc_health_topic);
    config_.override_topic = declare_parameter<std::string>(
        "reference_override_topic", config_.override_topic);
    config_.planner_rate_hz =
        declare_parameter<double>("planner_rate_hz", config_.planner_rate_hz);
    config_.planning_warn_ms = declare_parameter<double>(
        "planning_warn_time_ms", config_.planning_warn_ms);
    config_.planning_deadline_ms = declare_parameter<double>(
        "planning_deadline_ms", config_.planning_deadline_ms);
    config_.ego_stale_sec =
        declare_parameter<double>("ego_stale_time_sec", config_.ego_stale_sec);
    config_.opponent_stale_sec = declare_parameter<double>(
        "opponent_stale_time_sec", config_.opponent_stale_sec);
    config_.opponent_max_position_jump_m = declare_parameter<double>(
        "opponent_max_position_jump_m", config_.opponent_max_position_jump_m);
    config_.debug_costmap_rate_hz = declare_parameter<double>(
        "debug_costmap_publish_rate_hz", config_.debug_costmap_rate_hz);
    config_.normal_mode_min_hold_sec = declare_parameter<double>(
        "normal_mode_min_hold_sec", config_.normal_mode_min_hold_sec);
    config_.candidate_cost_hysteresis = declare_parameter<int>(
        "candidate_cost_hysteresis", config_.candidate_cost_hysteresis);
    config_.safe_stop_release_cost = declare_parameter<int>(
        "safe_stop_release_cost", config_.safe_stop_release_cost);
    config_.safe_stop_release_cycles = declare_parameter<int>(
        "safe_stop_release_required_cycles", config_.safe_stop_release_cycles);
    config_.overrun_previous_max_age_sec =
        declare_parameter<double>("overrun_last_trajectory_max_age_sec",
                                  config_.overrun_previous_max_age_sec);
    config_.overrun_stop_cycles = declare_parameter<int>(
        "overrun_safe_stop_consecutive_cycles", config_.overrun_stop_cycles);
    config_.mpc_health_speed_guard_enabled =
        declare_parameter<bool>("mpc_health_speed_guard_enabled",
                                config_.mpc_health_speed_guard_enabled);
    config_.mpc_health_infeasible_count_threshold =
        declare_parameter<int>("mpc_health_infeasible_count_threshold",
                               config_.mpc_health_infeasible_count_threshold);
    config_.mpc_health_solve_time_warn_ms = declare_parameter<double>(
        "mpc_health_solve_time_warn_ms", config_.mpc_health_solve_time_warn_ms);
    config_.mpc_health_v_max_mps = declare_parameter<double>(
        "mpc_health_v_max_mps", config_.mpc_health_v_max_mps);
    config_.mpc_health_stale_time_sec = declare_parameter<double>(
        "mpc_health_stale_time_sec", config_.mpc_health_stale_time_sec);
    config_.mpc_health_release_samples = declare_parameter<int>(
        "mpc_health_release_samples", config_.mpc_health_release_samples);
    instant_controller_config_.wheel_base_m = config_.wheel_base_m;
    instant_controller_config_.lookahead_min_m =
        declare_parameter<double>("instant_control_lookahead_min_m", 1.5);
    instant_controller_config_.lookahead_gain_sec =
        declare_parameter<double>("instant_control_lookahead_gain_sec", 0.35);
    instant_controller_config_.maximum_steering_angle_rad =
        declare_parameter<double>("instant_control_max_steering_angle_rad",
                                  config_.planner_max_steer_rad);
    instant_controller_config_.maximum_steering_rate_radps =
        declare_parameter<double>("instant_control_max_steering_rate_radps",
                                  config_.max_steer_rate_radps);
    instant_controller_config_.speed_proportional_gain =
        declare_parameter<double>("instant_control_speed_gain", 0.8);
    instant_controller_config_.minimum_acceleration_mps2 =
        config_.min_acceleration_mps2;
    instant_controller_config_.maximum_acceleration_mps2 =
        config_.max_acceleration_mps2;
  }

  void loadStaticInputs() {
    std::string error;
    try {
      if (!frame_.loadCsv(
              packagePath(config_.reference_package, config_.reference_csv),
              &error)) {
        degraded_reason_ = error;
        return;
      }
      if (!output_frame_.loadCsv(packagePath(config_.output_reference_package,
                                             config_.output_reference_csv),
                                 &error)) {
        degraded_reason_ = error;
        return;
      }
      if (!loadOvertakePermissionRules(&error)) {
        degraded_reason_ = error;
        return;
      }
      if (!map_.load(
              packagePath(config_.wall_map_package, config_.wall_map_yaml),
              config_, &error)) {
        degraded_reason_ = error;
        return;
      }
      if (!map_.buildReferenceLayer(frame_, config_)) {
        degraded_reason_ = "reference path is outside wall map";
      }
    } catch (const std::exception &exception) {
      degraded_reason_ = exception.what();
    }
    if (!degraded_reason_.empty()) {
      RCLCPP_ERROR(get_logger(), "static input degraded: %s",
                   degraded_reason_.c_str());
    }
  }

  bool loadOvertakePermissionRules(std::string *error) {
    config_.overtake_permission_rules.clear();
    if (!config_.overtake_permission_profile_enabled) {
      return true;
    }
    if (config_.overtake_permission_csv.empty()) {
      if (error != nullptr) {
        *error = "overtake permission profile is enabled but csv is empty";
      }
      return false;
    }

    const std::string path = packagePath(config_.overtake_permission_package,
                                         config_.overtake_permission_csv);
    std::ifstream file(path);
    if (!file.is_open()) {
      if (error != nullptr) {
        *error = "cannot open overtake permission csv: " + path;
      }
      return false;
    }

    std::string line;
    std::size_t line_number = 0U;
    while (std::getline(file, line)) {
      ++line_number;
      line = trim(line);
      if (line.empty() || line.front() == '#') {
        continue;
      }
      const auto columns = splitCsvLine(line);
      if (!columns.empty() && columns.front() == "name") {
        continue;
      }
      if (columns.size() != 4U) {
        if (error != nullptr) {
          *error = "invalid overtake permission csv column count at line " +
                   std::to_string(line_number);
        }
        return false;
      }
      const auto start_wp = parseInt64(columns[1]);
      const auto end_wp = parseInt64(columns[2]);
      const auto allow = parseBool(columns[3]);
      if (!start_wp.has_value() || !end_wp.has_value() || !allow.has_value() ||
          start_wp.value() < 0 || end_wp.value() < 0) {
        if (error != nullptr) {
          *error = "invalid overtake permission value at line " +
                   std::to_string(line_number);
        }
        return false;
      }
      const auto start_index = static_cast<std::size_t>(start_wp.value());
      const auto end_index = static_cast<std::size_t>(end_wp.value());
      if (start_index >= frame_.points().size() ||
          end_index >= frame_.points().size()) {
        if (error != nullptr) {
          *error =
              "overtake permission waypoint is outside reference at line " +
              std::to_string(line_number);
        }
        return false;
      }
      OvertakePermissionRule rule;
      rule.name =
          columns[0].empty()
              ? "permission_" +
                    std::to_string(config_.overtake_permission_rules.size())
              : columns[0];
      rule.s_start_m = frame_.points()[start_index].s;
      rule.s_end_m = frame_.points()[end_index].s;
      rule.allow_overtake = allow.value();
      config_.overtake_permission_rules.push_back(std::move(rule));
    }

    if (config_.overtake_permission_rules.empty()) {
      if (error != nullptr) {
        *error = "overtake permission csv contains no valid rules: " + path;
      }
      return false;
    }
    RCLCPP_INFO(get_logger(), "loaded %zu overtake permission rules from %s",
                config_.overtake_permission_rules.size(), path.c_str());
    return true;
  }

  std::optional<Pose2d> transformPose(const geometry_msgs::msg::Pose &pose,
                                      const std_msgs::msg::Header &header) {
    if (header.frame_id.empty()) {
      return std::nullopt;
    }
    if (header.frame_id == config_.map_frame) {
      return Pose2d{pose.position.x, pose.position.y,
                    yawFromQuaternion(pose.orientation)};
    }
    geometry_msgs::msg::PoseStamped source, transformed;
    source.header = header;
    source.pose = pose;
    try {
      transformed = tf_buffer_.transform(source, config_.map_frame,
                                         tf2::durationFromSec(0.02));
      return Pose2d{transformed.pose.position.x, transformed.pose.position.y,
                    yawFromQuaternion(transformed.pose.orientation)};
    } catch (const tf2::TransformException &exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "odom TF failed: %s", exception.what());
      return std::nullopt;
    }
  }

  std::optional<Pose2d> transformPoint(const geometry_msgs::msg::Point &point,
                                       const std_msgs::msg::Header &header) {
    if (header.frame_id.empty()) {
      return std::nullopt;
    }
    if (header.frame_id == config_.map_frame) {
      return Pose2d{point.x, point.y, 0.0};
    }
    geometry_msgs::msg::PointStamped source, transformed;
    source.header = header;
    source.point = point;
    try {
      transformed = tf_buffer_.transform(source, config_.map_frame,
                                         tf2::durationFromSec(0.02));
      return Pose2d{transformed.point.x, transformed.point.y, 0.0};
    } catch (const tf2::TransformException &exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "V2X TF failed: %s", exception.what());
      return std::nullopt;
    }
  }

  void onOdometry(const nav_msgs::msg::Odometry &message) {
    const auto pose = transformPose(message.pose.pose, message.header);
    EgoState state;
    state.stamp_sec = rclcpp::Time(message.header.stamp).seconds();
    state.speed_mps = message.twist.twist.linear.x;
    state.yaw_rate_radps = message.twist.twist.angular.z;
    if (pose.has_value()) {
      state.x = pose->x;
      state.y = pose->y;
      state.yaw = pose->yaw;
      state.frenet = frame_.project(state.x, state.y, state.yaw);
      state.curvature =
          std::abs(state.speed_mps) > 0.1
              ? state.yaw_rate_radps / state.speed_mps
              : (state.frenet.valid ? frame_.interpolate(state.frenet.s).kappa
                                    : 0.0);
      state.valid = state.frenet.valid && std::isfinite(state.speed_mps) &&
                    std::isfinite(state.yaw_rate_radps);
    }
    ego_ = state;
  }

  void onMpcHealth(const std_msgs::msg::String &message) {
    const auto infeasible_count =
        jsonNumberField(message.data, "mpc_infeasible_count");
    const auto solve_time_ms =
        jsonNumberField(message.data, "mpc_solve_time_ms");
    if (!infeasible_count.has_value() && !solve_time_ms.has_value()) {
      return;
    }
    MpcHealthStatus health;
    health.valid = true;
    health.infeasible_count =
        infeasible_count.has_value()
            ? std::max(0,
                       static_cast<int>(std::llround(infeasible_count.value())))
            : 0;
    health.solve_time_ms =
        solve_time_ms.value_or(std::numeric_limits<double>::quiet_NaN());
    health.age_sec = 0.0;
    health.sample_sequence =
        mpc_health_sample_sequence_ == std::numeric_limits<std::uint64_t>::max()
            ? 1U
            : mpc_health_sample_sequence_ + 1U;
    mpc_health_sample_sequence_ = health.sample_sequence;
    mpc_health_ = health;
    last_mpc_health_sec_ = now().seconds();
  }

  MpcHealthStatus currentMpcHealth(double now_sec) const {
    auto health = mpc_health_;
    if (!health.valid || !last_mpc_health_sec_.has_value()) {
      health.valid = false;
      return health;
    }
    health.age_sec = now_sec - last_mpc_health_sec_.value();
    return health;
  }

  void onV2x(const v2x_msgs::msg::V2XVehiclePositionArray &message) {
    const double now_sec = now().seconds();
    const bool recovered = !v2x_received_ || now_sec - v2x_receive_sec_ >
                                                 config_.opponent_stale_sec;
    v2x_received_ = true;
    v2x_receive_sec_ = now_sec;
    v2x_snapshot_valid_ = true;
    std::vector<OpponentState> snapshot;
    for (const auto &vehicle : message.vehicles) {
      if (vehicle.vehicle_id == own_vehicle_id_) {
        continue;
      }
      const auto pose = transformPoint(vehicle.position, vehicle.header);
      const auto uncertainty = uncertaintyMargin(vehicle.covariance.x,
                                                 vehicle.covariance.y, config_);
      const double stamp = rclcpp::Time(vehicle.header.stamp).seconds();
      if (!pose.has_value() || !uncertainty.valid || !std::isfinite(stamp)) {
        v2x_snapshot_valid_ = false;
        continue;
      }
      OpponentState opponent;
      opponent.id = vehicle.vehicle_id;
      opponent.stamp_sec = stamp;
      opponent.x = pose->x;
      opponent.y = pose->y;
      opponent.sigma_x_m = vehicle.covariance.x;
      opponent.sigma_y_m = vehicle.covariance.y;
      opponent.uncertainty_x_m = uncertainty.x_m;
      opponent.uncertainty_y_m = uncertainty.y_m;
      const auto previous = opponent_history_.find(opponent.id);
      if (previous != opponent_history_.end()) {
        const double dt = stamp - previous->second.stamp_sec;
        const double jump = std::hypot(opponent.x - previous->second.x,
                                       opponent.y - previous->second.y);
        if (dt < -1.0e-6 || jump > config_.opponent_max_position_jump_m) {
          v2x_snapshot_valid_ = false;
          continue;
        }
        if (dt > 1.0e-6) {
          opponent.vx_mps = (opponent.x - previous->second.x) / dt;
          opponent.vy_mps = (opponent.y - previous->second.y) / dt;
        } else {
          opponent.vx_mps = previous->second.vx_mps;
          opponent.vy_mps = previous->second.vy_mps;
        }
        opponent.speed_mps = std::hypot(opponent.vx_mps, opponent.vy_mps);
      }
      opponent.frenet = frame_.project(opponent.x, opponent.y, 0.0);
      opponent.yaw = opponent.speed_mps > 0.3
                         ? std::atan2(opponent.vy_mps, opponent.vx_mps)
                         : (opponent.frenet.valid
                                ? frame_.interpolate(opponent.frenet.s).yaw
                                : 0.0);
      opponent.frenet = frame_.project(opponent.x, opponent.y, opponent.yaw);
      opponent.valid = opponent.frenet.valid;
      if (!opponent.valid) {
        v2x_snapshot_valid_ = false;
        continue;
      }
      snapshot.push_back(opponent);
      opponent_history_[opponent.id] = opponent;
    }
    for (auto it = opponent_history_.begin(); it != opponent_history_.end();) {
      const bool present = std::any_of(snapshot.begin(), snapshot.end(),
                                       [&it](const OpponentState &opponent) {
                                         return opponent.id == it->first;
                                       });
      if (!present &&
          now_sec - it->second.stamp_sec > 2.0 * config_.opponent_stale_sec) {
        it = opponent_history_.erase(it);
      } else {
        ++it;
      }
    }
    opponents_ = std::move(snapshot);
    if (recovered && planner_) {
      planner_->detector().requestInitialSweep();
    }
  }

  PlannerOutput safeStop(const std::string &reason) const {
    PlannerOutput output;
    output.active = true;
    output.mode = BehaviorMode::SAFE_STOP;
    output.emergency_stop = true;
    output.speed_cap_mps = config_.safe_stop_speed_mps;
    output.reason = reason;
    return output;
  }

  bool inputsFresh(double now_sec) const {
    if (!config_.safety_evaluation_enabled) {
      return degraded_reason_.empty() && ego_.valid;
    }
    if (!degraded_reason_.empty() || !ego_.valid || !v2x_received_ ||
        !v2x_snapshot_valid_) {
      return false;
    }
    if (now_sec - ego_.stamp_sec > config_.ego_stale_sec ||
        now_sec - ego_.stamp_sec < -0.05 ||
        now_sec - v2x_receive_sec_ > config_.opponent_stale_sec) {
      return false;
    }
    return std::all_of(opponents_.begin(), opponents_.end(),
                       [this, now_sec](const OpponentState &opponent) {
                         return opponent.valid &&
                                now_sec - opponent.stamp_sec <=
                                    config_.opponent_stale_sec &&
                                now_sec - opponent.stamp_sec >= -0.05;
                       });
  }

  void onPlanningTimer() {
    ++speed_evidence_attempt_ordinal_;
    const std::uint64_t speed_evidence_attempt_ordinal =
        speed_evidence_attempt_ordinal_;
    const std::uint64_t planning_timer_entry_monotonic_ns =
        ay0_planner_input_audit_pub_ != nullptr ? monotonicNanoseconds() : 0U;
    const double now_sec = now().seconds();
    const bool inputs_fresh = inputsFresh(now_sec);
    const auto started = std::chrono::steady_clock::now();
    PlannerOutput output;
    std::unique_ptr<LatticePlanner> trial_planner;
    PlanningCycleMetrics planner_cycle_metrics;
    double trial_copy_ms = 0.0;
    double planner_update_ms = 0.0;
    std::uint64_t planner_update_begin_monotonic_ns = 0U;
    TrialSpeedEvidence trial_speed_evidence{};
    std::optional<SpeedEvidenceIdentity> pending_speed_evidence_identity;

    if (!enabled_) {
      ++speed_evidence_state_epoch_;
      beginSpeedEvidenceEpoch(speed_evidence_commit_state_);
      planner_->resetManeuverState();
      output.reason = "disabled";
      previous_safe_v3_.reset();
      previous_safe_output_.reset();
    } else if (!degraded_reason_.empty()) {
      planner_->clearPassContinuationLatch();
      output = safeStop(degraded_reason_);
    } else if (!v2x_received_ && config_.safety_evaluation_enabled) {
      // AWSIM does not publish an empty V2X snapshot in single-vehicle mode.
      // Keep the overtake override inactive until the first snapshot instead
      // of deadlocking the baseline controller at the safe-stop speed. Once a
      // snapshot has been received, inputsFresh() remains fail-closed if V2X
      // later becomes stale.
      ++speed_evidence_state_epoch_;
      beginSpeedEvidenceEpoch(speed_evidence_commit_state_);
      planner_->resetManeuverState();
      output.reason = "waiting_for_initial_v2x";
      previous_safe_v3_.reset();
      previous_safe_output_.reset();
    } else {
      // Planning is transactional. State is committed only when the cycle
      // finishes within the deadline, so a reused trajectory cannot leave the
      // planner one state transition ahead of the controller.
      const auto trial_copy_started = std::chrono::steady_clock::now();
      trial_planner = std::make_unique<LatticePlanner>(*planner_);
      trial_copy_ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - trial_copy_started)
                          .count();
      const auto planner_update_started = std::chrono::steady_clock::now();
      if (ay0_planner_input_audit_pub_ != nullptr) {
        planner_update_begin_monotonic_ns = monotonicNanoseconds();
      }
      output = trial_planner->update(ego_, opponents_, inputs_fresh, now_sec,
                                     currentMpcHealth(now_sec),
                                     &trial_speed_evidence);
      planner_update_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - planner_update_started)
              .count();
      planner_cycle_metrics = trial_planner->lastPlanningCycleMetrics();
    }

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - started)
                                  .count();
    if (elapsed_ms > config_.planning_warn_ms) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "planning cycle %.2f ms", elapsed_ms);
    }
    if (config_.safety_evaluation_enabled && trial_planner &&
        elapsed_ms > config_.planning_deadline_ms) {
      // The trial planner is not committed on this branch. Never bind its
      // diagnostic snapshot to a reused or fail-closed wire generation.
      discardUncommittedPassClearanceDiagnostic(&planner_cycle_metrics);
      ++consecutive_overruns_;
      const bool previous_fresh = previous_safe_v3_.has_value() &&
                                  previous_safe_output_.has_value() &&
                                  now_sec - previous_safe_time_sec_ <=
                                      config_.overrun_previous_max_age_sec &&
                                  inputs_fresh;
      const bool previous_horizon_safe =
          previous_fresh &&
          !previous_safe_output_->preventive_side_role_active &&
          planner_->validateOutputHorizon(
              ego_, opponents_, previous_safe_output_->lateral_offsets_m,
              previous_safe_output_->speed_caps_mps,
              previous_safe_output_->longitudinal_offsets_m, now_sec);
      const bool previous_still_safe =
          previous_fresh &&
          deadlinePreviousOutputReusable(previous_fresh, previous_horizon_safe,
                                         *previous_safe_output_);
      // A deadline cycle cannot prove that the latched pass identity advanced
      // transactionally. Reusing a currently revalidated output is allowed,
      // but the next normal cycle must start without continuation authority.
      planner_->clearPassContinuationLatch();
      if (consecutive_overruns_ < config_.overrun_stop_cycles &&
          previous_still_safe) {
        PlannerOutput reused_output = previous_safe_output_.value();
        reused_output.pass_continuation_latched = false;
        reused_output.pass_continuation_active = false;
        reused_output.pass_continuation_suspended = false;
        reused_output.pass_continuation_target_id.clear();
        reused_output.pass_continuation_side = 0;
        reused_output.pass_continuation_lateral_index = -1;
        reused_output.pass_continuation_tangent_index = -1;
        reused_output.pass_continuation_goal_d_m =
            std::numeric_limits<double>::quiet_NaN();
        reused_output.pass_continuation_required_arc_m =
            std::numeric_limits<double>::quiet_NaN();
        reused_output.pass_continuation_target_observation_stamp_sec =
            std::numeric_limits<double>::quiet_NaN();
        discardSpeedEvidenceTrial(speed_evidence_commit_state_);
        override_pub_->publish(previous_safe_v3_.value());
        publishInstantControl(reused_output, now_sec);
        publishDebug(reused_output, elapsed_ms, "deadline_reused_previous",
                     planner_cycle_metrics, trial_copy_ms, planner_update_ms);
        return;
      }
      output = makePlanningDeadlineStop(config_.safe_stop_speed_mps);
      // trial_planner is deliberately discarded here.
      discardSpeedEvidenceTrial(speed_evidence_commit_state_);
    } else {
      consecutive_overruns_ = 0;
      if (trial_planner) {
        planner_ = std::move(trial_planner);
        pending_speed_evidence_identity =
            prepareSpeedEvidenceCommitAfterPlannerMove(
                speed_evidence_commit_state_, planner_session_id_,
                speed_evidence_state_epoch_, speed_evidence_attempt_ordinal);
      }
    }

    const bool poc_base_identity_available =
        state_lattice_v2_live_proposal_publish_enabled_ &&
        state_lattice_v2_base_attestation_.has_value() &&
        overtake_transport_contract::state_lattice_v2::validateBaseAttestation(
            state_lattice_v2_base_attestation_.value(),
            state_lattice_v2_expected_pp_producer_instance_id_,
            state_lattice_v2_expected_pp_session_id_, now()) ==
            overtake_transport_contract::state_lattice_v2::RejectReason::kNone;
    // V2 keeps its exact base-attestation gate.  The deliberately narrower
    // V4 AWSIM PoC removes the wire copy's shadow marker only through its
    // explicit live-publication switch.  Repository launch/runner policy
    // limits that switch to simulation; direct parameter activation is not a
    // supported production or real-vehicle entry point.
    PlannerOutput wire_output = prepareWireOutputForPublication(
        output, poc_base_identity_available,
        experimental_spatial_reference_override_live_publish_enabled_);
    const WirePublicationPolicy wire_policy{
        config_.experimental_exact_spatial_follow_shadow_enabled,
        experimental_spatial_reference_override_live_publish_enabled_};
    WirePayload prospective =
        makeWirePayload(wire_output, generation_, wire_policy);
    const bool semantic_generation_advanced =
        !last_payload_.has_value() ||
        !semanticallyEqual(last_payload_.value(), prospective);
    if (semantic_generation_advanced) {
      generation_ = nextGeneration(generation_);
      prospective = makeWirePayload(wire_output, generation_, wire_policy);
    }
    last_payload_ = prospective;
    std_msgs::msg::Float32MultiArray message;
    message.data = prospective.data;
    override_pub_->publish(message);
    if ((prospective.kind == WireKind::LATERAL_AND_SPEED_V3 ||
         prospective.kind == WireKind::SPATIAL_LATERAL_AND_SPEED_V4) &&
        output.safe_lateral) {
      previous_safe_v3_ = message;
      previous_safe_output_ = output;
      previous_safe_time_sec_ = now_sec;
    } else {
      // Never resurrect a lateral command after a normal cycle deliberately
      // switched to speed-only FOLLOW/PREPARE/YIELD/STOP behavior.
      previous_safe_v3_.reset();
      previous_safe_output_.reset();
    }
    publishInstantControl(output, now_sec);
    publishStateLatticeV2Proposal(output);
    captureAy0ShadowProposal(output, planning_timer_entry_monotonic_ns,
                             planner_update_begin_monotonic_ns,
                             pending_speed_evidence_identity.has_value()
                                 ? &trial_speed_evidence
                                 : nullptr);
    std::optional<CommittedSpeedEvidence> pending_committed_speed_evidence;
    if (pending_speed_evidence_identity.has_value()) {
      pending_speed_evidence_identity->wire_semantic_generation = generation_;
      pending_committed_speed_evidence = sealCommittedSpeedEvidenceAfterCommit(
          trial_speed_evidence, pending_speed_evidence_identity.value());
    }
    // Handoff is deliberately after every safety-critical publication.
    // Missing exact payload binding remains a sealed INVALID_CAPTURE and can
    // never make the one-shot pair mailbox READY.
    if (pending_committed_speed_evidence.has_value()) {
      finishSpeedEvidenceCommit(speed_evidence_commit_state_,
                                pending_committed_speed_evidence.value());
    }
    publishDebug(output, elapsed_ms, output.reason, planner_cycle_metrics,
                 trial_copy_ms, planner_update_ms);
  }

  void onAy0BaseSnapshot(
      const multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot
          &snapshot) {
    const std::uint64_t callback_entry_monotonic_ns =
        ay0_planner_input_audit_pub_ != nullptr ? monotonicNanoseconds() : 0U;
    if (!ay0_shadow_capture_enabled_) {
      return;
    }
    if (!c002ay0_shadow::copyFixedBaseRecord(
            snapshot, ay0_shadow_session_generation_, ay0_shadow_session_nonce_,
            &ay0_base_snapshot_record_scratch_,
            &ay0_base_snapshot_provenance_scratch_)) {
      ay0_base_snapshot_record_.reset();
      ay0_base_snapshot_provenance_.reset();
      ++ay0_shadow_base_reject_count_;
      return;
    }
    ay0_base_snapshot_record_ = ay0_base_snapshot_record_scratch_;
    ay0_base_snapshot_provenance_ = ay0_base_snapshot_provenance_scratch_;
    if (ay0_planner_input_audit_pub_ != nullptr) {
      ay0_base_snapshot_callback_entry_monotonic_ns_ =
          callback_entry_monotonic_ns;
    }
  }

  void onStateLatticeV2BaseAttestation(
      const multi_purpose_mpc_ros_msgs::msg::StateLatticeV2BaseAttestation
          &attestation) {
    if (!state_lattice_v2_live_proposal_publish_enabled_) {
      return;
    }
    if (state_lattice_v2_base_attestation_receive_count_ !=
        std::numeric_limits<std::uint64_t>::max()) {
      ++state_lattice_v2_base_attestation_receive_count_;
    }
    const builtin_interfaces::msg::Time now_ros = now();
    const auto validation =
        overtake_transport_contract::state_lattice_v2::validateBaseAttestation(
            attestation, state_lattice_v2_expected_pp_producer_instance_id_,
            state_lattice_v2_expected_pp_session_id_, now_ros);
    const auto &base = attestation.base;
    const bool exact_source =
        base.base_source_digest_state == base.BASE_SOURCE_DIGEST_COMPLETE &&
        (base.base_source_kind == base.SOURCE_MPC_HORIZON ||
         base.base_source_kind == base.SOURCE_REFERENCE_TRAJECTORY);
    state_lattice_v2_last_base_attestation_validation_reason_ =
        static_cast<std::uint8_t>(validation);
    state_lattice_v2_last_base_attestation_exact_source_ = exact_source;
    if (validation != overtake_transport_contract::state_lattice_v2::
                          RejectReason::kNone ||
        !exact_source) {
      state_lattice_v2_base_attestation_.reset();
      if (state_lattice_v2_base_attestation_reject_count_ !=
          std::numeric_limits<std::uint64_t>::max()) {
        ++state_lattice_v2_base_attestation_reject_count_;
      }
      return;
    }
    state_lattice_v2_base_attestation_ = attestation;
    if (state_lattice_v2_base_attestation_accept_count_ !=
        std::numeric_limits<std::uint64_t>::max()) {
      ++state_lattice_v2_base_attestation_accept_count_;
    }
  }

  void onStateLatticeV2QuiesceRequest(
      const multi_purpose_mpc_ros_msgs::msg::StateLatticeV2QuiesceRequest
          &message) {
    if (!state_lattice_v2_final_fence_enabled_ ||
        state_lattice_v2_final_fence_state_ == nullptr) {
      return;
    }
    FinalFenceRequest request;
    request.schema_version = message.schema_version;
    request.execution_nonce = message.execution_nonce;
    request.expected_producer_instance_id =
        message.expected_producer_instance_id;
    request.expected_session_id = message.expected_session_id;
    request.sealed_epoch_id = message.sealed_epoch_id;
    request.request_id = message.request_id;
    request.canonical_sha256 = message.canonical_sha256;
    const auto result = state_lattice_v2_final_fence_state_->requestQuiesce(
        request, monotonicNanoseconds());
    if (result == FinalFenceRequestResult::kAccepted) {
      publishStateLatticeV2FinalFenceIfReady();
    }
  }

  void publishStateLatticeV2FinalFenceIfReady() {
    if (!state_lattice_v2_final_fence_enabled_ ||
        state_lattice_v2_final_fence_state_ == nullptr ||
        state_lattice_v2_final_fence_pub_ == nullptr) {
      return;
    }
    const auto snapshot = state_lattice_v2_final_fence_state_->prepareFence(
        monotonicNanoseconds());
    if (!snapshot.has_value()) {
      return;
    }
    multi_purpose_mpc_ros_msgs::msg::StateLatticeV2FinalFence message;
    message.schema_version = snapshot->schema_version;
    message.execution_nonce = snapshot->execution_nonce;
    message.producer_instance_id = snapshot->producer_instance_id;
    message.session_id = snapshot->session_id;
    message.sealed_epoch_id = snapshot->sealed_epoch_id;
    message.fence_id = snapshot->fence_id;
    message.request_id = snapshot->request_id;
    message.request_canonical_sha256 = snapshot->request_canonical_sha256;
    message.final_committed_ordinal = snapshot->final_committed_ordinal;
    message.successful_emission_count = snapshot->successful_emission_count;
    message.final_identity_present = snapshot->final_identity_present;
    if (snapshot->final_identity_present) {
      message.final_identity.producer_instance_id =
          snapshot->final_identity.producer_instance_id;
      message.final_identity.session_id = snapshot->final_identity.session_id;
      message.final_identity.proposal_sequence =
          snapshot->final_identity.proposal_sequence;
      message.final_identity.plan_generation =
          snapshot->final_identity.plan_generation;
      message.final_identity.source_generation =
          snapshot->final_identity.source_generation;
      message.final_identity.source_stamp.sec =
          snapshot->final_identity.source_stamp_sec;
      message.final_identity.source_stamp.nanosec =
          snapshot->final_identity.source_stamp_nanosec;
      message.final_identity.frame_id = snapshot->final_identity.frame_id;
      message.final_identity.canonical_sha256 =
          snapshot->final_identity.canonical_sha256;
      message.final_identity.publish_monotonic_ns =
          snapshot->final_identity.publish_monotonic_ns;
    }
    message.proposal_qos_fingerprint = snapshot->proposal_qos_fingerprint;
    message.quiesced_monotonic_ns = snapshot->quiesced_monotonic_ns;
    message.fence_publish_monotonic_ns = snapshot->fence_publish_monotonic_ns;
    message.canonical_sha256 = snapshot->canonical_sha256;
    try {
      state_lattice_v2_final_fence_pub_->publish(message);
    } catch (...) {
      // FENCED is process-terminal even on ambiguous fence publication. The
      // independent sticky fault prevents evidence credit and no retry occurs.
      state_lattice_v2_final_fence_state_->markFencePublicationAmbiguous();
    }
  }

  void publishStateLatticeV2Proposal(const PlannerOutput &output) {
    const char *gate_reason = nullptr;
    if (state_lattice_v2_publication_halted_ ||
        state_lattice_v2_publication_state_.halted()) {
      state_lattice_v2_publication_halted_ = true;
      gate_reason = "publication_halted";
    } else if (!state_lattice_v2_live_proposal_publish_enabled_) {
      gate_reason = "disabled";
    } else if (state_lattice_v2_proposal_pub_ == nullptr) {
      gate_reason = "publisher_missing";
    } else if (!output.active) {
      gate_reason = "output_inactive";
    } else if (output.emergency_stop) {
      gate_reason = "emergency_stop";
    } else if (!state_lattice_v2_base_attestation_.has_value()) {
      gate_reason = "base_attestation_missing";
    } else if (output.target_id.empty()) {
      gate_reason = "target_missing";
    } else if (!output.safe_lateral) {
      gate_reason = "unsafe_lateral";
    }
    if (gate_reason != nullptr) {
      state_lattice_v2_last_publish_reason_ = gate_reason;
      return;
    }
    const auto &selected = planner_->selected();
    const bool selected_mode_matches =
        selected.has_value() &&
        ((output.mode == BehaviorMode::OVERTAKE_LEFT &&
          selected->goal_d_m > 0.05) ||
         (output.mode == BehaviorMode::OVERTAKE_RIGHT &&
          selected->goal_d_m < -0.05) ||
         (output.mode == BehaviorMode::SIDE_BY_SIDE_KEEP &&
          std::abs(selected->goal_d_m) > 0.05));
    if (!selected.has_value() || !selected->feasible ||
        !selected_mode_matches || selected->dense.empty() ||
        std::abs(selected->goal_d_m - ego_.frenet.d) <= 0.05) {
      state_lattice_v2_last_publish_reason_ = "selected_candidate_mismatch";
      return;
    }

    const auto next_preparation = state_lattice_v2_publication_state_.prepare(
        StateLatticeV2SemanticKey{});
    if (!next_preparation.intent.has_value()) {
      state_lattice_v2_last_publish_reason_ =
          next_preparation.result ==
                  StateLatticeV2PublicationPrepareResult::kSequenceExhausted
              ? "sequence_exhausted"
              : "plan_generation_exhausted";
      return;
    }
    Ay0ShadowProposalIdentity identity;
    identity.planner_instance_id = state_lattice_v2_producer_instance_id_;
    identity.attempt_id = next_preparation.intent->proposal_sequence;
    identity.connector_transaction_id = identity.attempt_id;
    identity.authority_token = identity.attempt_id;
    identity.safety_snapshot_id = next_preparation.intent->proposal_sequence;
    identity.plan_generation = next_preparation.intent->plan_generation;
    identity.candidate_revision = next_preparation.intent->plan_generation;
    identity.target_id = output.target_id;
    const auto plan_stamp = now();
    const auto proposal = buildAy0ShadowProposal(
        state_lattice_v2_base_attestation_->base, selected.value(), ego_,
        opponents_, config_, ay0_shadow_safety_evidence_, plan_stamp, identity);
    if (!proposal.valid()) {
      ++state_lattice_v2_publish_reject_count_;
      state_lattice_v2_last_publish_reason_ = proposal.reason;
      return;
    }
    const auto producer_instance_id =
        std::to_string(identity.planner_instance_id);
    const auto session_id =
        std::to_string(proposal.trajectory.plan_sample_key.race_arm_epoch);
    const auto semantic_key =
        makeStateLatticeV2SemanticKey(proposal.trajectory, producer_instance_id,
                                      session_id, output.mode, output.intent);
    auto final_fence_admission =
        state_lattice_v2_final_fence_enabled_
            ? state_lattice_v2_final_fence_state_->tryAdmit(
                  producer_instance_id, session_id,
                  proposal.trajectory.plan_sample_key.race_arm_epoch)
            : std::optional<StateLatticeV2FinalFenceState::Admission>{};
    if (state_lattice_v2_final_fence_enabled_ &&
        !final_fence_admission.has_value() &&
        state_lattice_v2_final_fence_state_->phase() !=
            FinalFencePublicationPhase::kOpen) {
      state_lattice_v2_last_publish_reason_ = "diagnostic_epoch_quiesced";
      return;
    }
    const auto publication_preparation =
        state_lattice_v2_publication_state_.prepare(semantic_key);
    if (!publication_preparation.intent.has_value()) {
      state_lattice_v2_last_publish_reason_ =
          publication_preparation.result ==
                  StateLatticeV2PublicationPrepareResult::kDuplicate
              ? "semantic_generation_unchanged"
          : publication_preparation.result ==
                  StateLatticeV2PublicationPrepareResult::kSequenceExhausted
              ? "sequence_exhausted"
              : "plan_generation_exhausted";
      if (final_fence_admission.has_value()) {
        state_lattice_v2_final_fence_state_->finishUncommitted(
            std::move(final_fence_admission.value()));
        publishStateLatticeV2FinalFenceIfReady();
      }
      return;
    }
    const auto &publication_intent = publication_preparation.intent.value();

    multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectoryV2 message;
    message.schema_version = message.SCHEMA_V2_NON_AUTHORITATIVE;
    message.header.stamp = proposal.trajectory.plan_stamp;
    message.header.frame_id = proposal.trajectory.frame_id;
    message.identity.producer_instance_id = producer_instance_id;
    message.identity.session_id = session_id;
    message.identity.proposal_sequence = publication_intent.proposal_sequence;
    message.identity.plan_generation = publication_intent.plan_generation;
    message.identity.source_generation =
        proposal.trajectory.base_source_generation;
    message.identity.source_stamp = proposal.trajectory.base_source_stamp;
    message.identity.frame_id = proposal.trajectory.frame_id;
    message.identity.canonical_sha256 = proposal.trajectory.payload_sha256;
    message.identity.publish_monotonic_ns = monotonicNanoseconds();
    message.proposal = proposal.trajectory;
    try {
      state_lattice_v2_proposal_pub_->publish(message);
    } catch (const std::exception &exception) {
      if (final_fence_admission.has_value()) {
        state_lattice_v2_final_fence_state_->finishAmbiguous(
            std::move(final_fence_admission.value()));
      }
      state_lattice_v2_publication_halted_ = true;
      state_lattice_v2_publication_state_.halt();
      state_lattice_v2_last_publish_reason_ = "publication_delivery_ambiguous";
      RCLCPP_ERROR(
          get_logger(),
          "State Lattice V2 publication halted after publish error: %s",
          exception.what());
      return;
    } catch (...) {
      if (final_fence_admission.has_value()) {
        state_lattice_v2_final_fence_state_->finishAmbiguous(
            std::move(final_fence_admission.value()));
      }
      state_lattice_v2_publication_halted_ = true;
      state_lattice_v2_publication_state_.halt();
      state_lattice_v2_last_publish_reason_ = "publication_delivery_ambiguous";
      RCLCPP_ERROR(
          get_logger(),
          "State Lattice V2 publication halted after unknown publish error");
      return;
    }
    if (!state_lattice_v2_publication_state_.commit(publication_intent)) {
      if (final_fence_admission.has_value()) {
        state_lattice_v2_final_fence_state_->finishAmbiguous(
            std::move(final_fence_admission.value()));
      }
      state_lattice_v2_publication_halted_ = true;
      state_lattice_v2_publication_state_.halt();
      state_lattice_v2_last_publish_reason_ = "publication_delivery_ambiguous";
      RCLCPP_ERROR(
          get_logger(),
          "State Lattice V2 publication halted after post-publish state "
          "commit mismatch");
      return;
    }
    if (final_fence_admission.has_value()) {
      FinalFenceIdentity committed_identity;
      committed_identity.producer_instance_id =
          message.identity.producer_instance_id;
      committed_identity.session_id = message.identity.session_id;
      committed_identity.proposal_sequence = message.identity.proposal_sequence;
      committed_identity.plan_generation = message.identity.plan_generation;
      committed_identity.source_generation = message.identity.source_generation;
      committed_identity.source_stamp_sec = message.identity.source_stamp.sec;
      committed_identity.source_stamp_nanosec =
          message.identity.source_stamp.nanosec;
      committed_identity.frame_id = message.identity.frame_id;
      committed_identity.canonical_sha256 = message.identity.canonical_sha256;
      committed_identity.publish_monotonic_ns =
          message.identity.publish_monotonic_ns;
      (void)state_lattice_v2_final_fence_state_->finishCommitted(
          std::move(final_fence_admission.value()),
          publication_intent.proposal_sequence, committed_identity);
      publishStateLatticeV2FinalFenceIfReady();
    }
    state_lattice_v2_last_publish_reason_ = "published";
  }

  void captureAy0ShadowProposal(const PlannerOutput &output,
                                std::uint64_t planning_timer_entry_monotonic_ns,
                                std::uint64_t planner_update_begin_monotonic_ns,
                                TrialSpeedEvidence *speed_evidence) {
    if (!ay0_shadow_capture_enabled_ || ay0_shadow_worker_session_ == nullptr ||
        !ay0_shadow_worker_session_->ready() ||
        !ay0_base_snapshot_record_.has_value() || output.target_id.empty() ||
        !ay0_base_snapshot_provenance_.has_value() || !output.safe_lateral) {
      return;
    }
    const auto &selected = planner_->selected();
    if (!selected.has_value() || !selected->feasible ||
        selected->dense.empty() ||
        std::abs(selected->goal_d_m - ego_.frenet.d) <= 0.05) {
      return;
    }
    Ay0ShadowProposalIdentity identity;
    identity.planner_instance_id = instant_control_producer_instance_id_;
    identity.attempt_id = ++ay0_shadow_attempt_id_;
    identity.connector_transaction_id = identity.attempt_id;
    identity.authority_token = identity.attempt_id;
    identity.safety_snapshot_id = ++ay0_shadow_safety_snapshot_id_;
    identity.plan_generation = generation_ == 0U ? 1U : generation_;
    identity.candidate_revision = ++ay0_shadow_candidate_revision_;
    identity.target_id = output.target_id;
    const auto capture = c002ay0_shadow::buildFixedProposalRecord(
        ay0_base_snapshot_record_.value(),
        ay0_base_snapshot_provenance_.value(), selected.value(), ego_,
        opponents_, ay0_shadow_safety_evidence_, now(), identity,
        ay0_shadow_session_generation_, ay0_shadow_session_nonce_,
        &ay0_shadow_record_scratch_);
    if (capture != c002ay0_shadow::FixedProposalCaptureResult::kBuilt) {
      ++ay0_shadow_capture_reject_count_;
      return;
    }
    if (!ay0_shadow_worker_session_->tryCapture(ay0_shadow_record_scratch_)) {
      ++ay0_shadow_enqueue_reject_count_;
      return;
    }
    if (speed_evidence != nullptr) {
      const auto &record = ay0_shadow_record_scratch_;
      auto &binding = speed_evidence->payload_binding;
      binding.session_generation = record.session_generation;
      binding.session_nonce = record.session_nonce;
      binding.planner_instance_id = record.planner_instance_id;
      binding.attempt_id = record.attempt_id;
      binding.connector_transaction_id = record.connector_transaction_id;
      binding.authority_token = record.authority_token;
      binding.safety_snapshot_id = record.safety_snapshot_id;
      binding.plan_generation = record.plan_generation;
      binding.candidate_revision = record.candidate_revision;
      binding.race_arm_epoch = record.base.race_arm_epoch;
      binding.controller_instance_id = record.base.controller_instance_id;
      binding.controller_sequence = record.base.controller_sequence;
      binding.base_lease_id = record.base.base_lease_id;
      binding.base_source_stamp_sec = record.base.base_source_stamp.sec;
      binding.base_source_stamp_nanosec = record.base.base_source_stamp.nanosec;
      binding.base_source_generation = record.base.base_source_generation;
      speed_evidence->present_fields |= SPEED_EVIDENCE_PAYLOAD_BINDING;
    }
    publishAy0PlannerInputAudit(ay0_shadow_record_scratch_,
                                planning_timer_entry_monotonic_ns,
                                planner_update_begin_monotonic_ns);
  }

  void
  publishAy0PlannerInputAudit(const c002ay0_shadow::FixedProposalRecord &record,
                              std::uint64_t planning_timer_entry_monotonic_ns,
                              std::uint64_t planner_update_begin_monotonic_ns) {
    if (ay0_planner_input_audit_pub_ == nullptr ||
        ay0_base_snapshot_callback_entry_monotonic_ns_ == 0U) {
      return;
    }
    // Test-only sidecar: derive every identifier from the already-built fixed
    // record, so this cannot alter proposal construction or authority.
    std_msgs::msg::String audit;
    audit.data =
        "{\"schema_version\":1,\"proposal_race_arm_epoch\":" +
        std::to_string(record.base.race_arm_epoch) +
        ",\"proposal_planner_instance_id\":" +
        std::to_string(record.planner_instance_id) +
        ",\"proposal_attempt_id\":" + std::to_string(record.attempt_id) +
        ",\"proposal_connector_transaction_id\":" +
        std::to_string(record.connector_transaction_id) +
        ",\"proposal_plan_generation\":" +
        std::to_string(record.plan_generation) +
        ",\"proposal_candidate_revision\":" +
        std::to_string(record.candidate_revision) +
        ",\"proposal_authority_token\":" +
        std::to_string(record.authority_token) +
        ",\"proposal_safety_snapshot_id\":" +
        std::to_string(record.safety_snapshot_id) +
        ",\"proposal_controller_instance_id\":" +
        std::to_string(record.base.controller_instance_id) +
        ",\"proposal_controller_sequence\":" +
        std::to_string(record.base.controller_sequence) +
        ",\"proposal_base_lease_id\":" +
        std::to_string(record.base.base_lease_id) +
        ",\"proposal_source_stamp_sec\":" +
        std::to_string(record.base.base_source_stamp.sec) +
        ",\"proposal_source_stamp_nanosec\":" +
        std::to_string(record.base.base_source_stamp.nanosec) +
        ",\"proposal_source_generation\":" +
        std::to_string(record.base.base_source_generation) +
        ",\"base_snapshot_callback_entry_monotonic_ns\":" +
        std::to_string(ay0_base_snapshot_callback_entry_monotonic_ns_) +
        ",\"planning_timer_entry_monotonic_ns\":" +
        std::to_string(planning_timer_entry_monotonic_ns) +
        ",\"planner_update_begin_monotonic_ns\":" +
        std::to_string(planner_update_begin_monotonic_ns) +
        ",\"capture_monotonic_ns\":" +
        std::to_string(record.capture_monotonic_ns) + "}";
    ay0_planner_input_audit_pub_->publish(audit);
  }

  void publishInstantControl(const PlannerOutput &output, double now_sec) {
    if (!live_control_output_enabled_ || !instant_control_enabled_) {
      return;
    }

    using Message = multi_purpose_mpc_ros_msgs::msg::StateLatticeControlCommand;
    Message message;
    message.header.stamp = now();
    message.header.frame_id = config_.map_frame;
    message.schema_version = Message::SCHEMA_V1;
    message.producer_instance_id = instant_control_producer_instance_id_;
    message.command_sequence = ++instant_control_command_sequence_;
    message.plan_generation = generation_;
    message.active = output.active;
    message.safety_evaluation_enabled = config_.safety_evaluation_enabled;
    message.inputs_fresh = inputsFresh(now_sec);
    message.costmap_valid = map_.initialized() && degraded_reason_.empty();
    message.stop_required = output.emergency_stop;

    InstantControlResult control;
    const auto selected = planner_->selected();
    if (message.active && selected.has_value() && selected->feasible) {
      control = instant_controller_->update(ego_, selected->dense, now_sec);
    } else {
      instant_controller_->reset();
      control.reason =
          message.active ? "selected_trajectory_missing" : "inactive";
    }
    message.trajectory_valid = control.valid;
    message.trajectory_safe = bool(control.valid && selected.has_value() &&
                                   selected->feasible && output.safe_lateral);
    message.stop_required =
        bool(message.stop_required ||
             (message.active &&
              (!message.safety_evaluation_enabled || !message.inputs_fresh ||
               !message.costmap_valid || !message.trajectory_valid ||
               !message.trajectory_safe)));

    const auto stamp = message.header.stamp;
    message.command.stamp = stamp;
    message.command.longitudinal.stamp = stamp;
    message.command.lateral.stamp = stamp;
    message.command.longitudinal.speed =
        static_cast<float>(control.valid ? control.speed_mps : 0.0);
    message.command.longitudinal.acceleration =
        static_cast<float>(control.valid ? control.acceleration_mps2
                                         : config_.min_acceleration_mps2);
    message.command.longitudinal.jerk = 0.0F;
    message.command.lateral.steering_tire_angle =
        static_cast<float>(control.valid ? control.steering_angle_rad : 0.0);
    message.command.lateral.steering_tire_rotation_rate =
        static_cast<float>(control.valid ? control.steering_rate_radps : 0.0);
    message.reason = control.reason.substr(0U, 128U);
    instant_control_pub_->publish(message);
  }

  void publishDebug(const PlannerOutput &output, double elapsed_ms,
                    const std::string &reason,
                    const PlanningCycleMetrics &planner_cycle_metrics,
                    double trial_copy_ms, double planner_update_ms) {
    std_msgs::msg::String mode;
    mode.data = toString(output.mode);
    mode_pub_->publish(mode);
    std_msgs::msg::String metrics;
    std::ostringstream json;
    const auto append_finite = [&json](double value) {
      if (std::isfinite(value)) {
        json << value;
      } else {
        json << "null";
      }
    };
    const auto append_json_escaped = [&json](const char *value) {
      static constexpr char kHex[] = "0123456789abcdef";
      for (const auto *cursor = reinterpret_cast<const unsigned char *>(value);
           *cursor != '\0'; ++cursor) {
        if (*cursor == '"' || *cursor == '\\') {
          json << '\\' << static_cast<char>(*cursor);
        } else if (*cursor < 0x20U) {
          json << "\\u00" << kHex[*cursor >> 4U] << kHex[*cursor & 0x0fU];
        } else {
          json << static_cast<char>(*cursor);
        }
      }
    };
    json << std::fixed << std::setprecision(3) << "{\"mode\":\""
         << toString(output.mode) << "\",\"reason\":\"" << reason
         << "\",\"planning_ms\":" << elapsed_ms
         << ",\"trial_copy_ms\":" << trial_copy_ms
         << ",\"planner_update_ms\":" << planner_update_ms
         << ",\"candidate_generation_ms\":"
         << planner_cycle_metrics.candidate_generation_ms
         << ",\"output_horizon_ms\":" << planner_cycle_metrics.output_horizon_ms
         << ",\"candidate_dense_point_count\":"
         << planner_cycle_metrics.candidate_dense_point_count
         << ",\"output_horizon_call_count\":"
         << planner_cycle_metrics.output_horizon_call_count
         << ",\"pass_clearance\":{";
    const auto &pass_clearance =
        planner_cycle_metrics.pass_clearance_diagnostic;
    json << "\"target_id\":\"";
    append_json_escaped(pass_clearance.target_id.data());
    json << "\",\"target_d_m\":";
    append_finite(pass_clearance.target_d_m);
    json << ",\"target_uncertainty_y_m\":";
    append_finite(pass_clearance.target_uncertainty_y_m);
    json << ",\"target_observation_stamp_sec\":";
    append_finite(pass_clearance.target_observation_stamp_sec);
    json << ",\"candidate_goal_d_m\":";
    append_finite(pass_clearance.candidate_goal_d_m);
    json << ",\"side\":" << pass_clearance.side << ",\"actual_separation_m\":";
    append_finite(pass_clearance.actual_separation_m);
    json << ",\"required_separation_m\":";
    append_finite(pass_clearance.required_separation_m);
    json << ",\"margin_m\":";
    append_finite(pass_clearance.margin_m);
    json << ",\"observed_predicate\":"
         << (pass_clearance.observed_predicate ? "true" : "false")
         << ",\"evaluated\":" << (pass_clearance.evaluated ? "true" : "false")
         << ",\"inputs_valid\":"
         << (pass_clearance.inputs_valid ? "true" : "false")
         << ",\"wire_generation\":";
    if (pass_clearance.evaluated) {
      json << generation_;
    } else {
      json << "null";
    }
    json << "}"
         << ",\"minimum_cost\":" << output.minimum_cost
         << ",\"rear_cost\":" << output.rear_cost
         << ",\"rear_hard_safe\":" << (output.rear_hard_safe ? "true" : "false")
         << ",\"speed_cap_mps\":" << output.speed_cap_mps
         << ",\"candidate_speed_limit_mps\":"
         << output.candidate_speed_limit_mps
         << ",\"safe_lateral\":" << (output.safe_lateral ? "true" : "false")
         << ",\"target_missing\":" << (output.target_missing ? "true" : "false")
         << ",\"preventive_side_role_active\":"
         << (output.preventive_side_role_active ? "true" : "false")
         << ",\"preventive_side_role_leader\":"
         << (output.preventive_side_role_leader ? "true" : "false")
         << ",\"preventive_side_role_neutral\":"
         << (output.preventive_side_role_neutral ? "true" : "false")
         << ",\"preventive_side_role_phase\":\""
         << toString(output.preventive_side_role_phase)
         << "\",\"preventive_side_role_action\":\""
         << toString(output.preventive_side_role_action)
         << "\",\"preventive_side_role_generation\":"
         << output.preventive_side_role_generation
         << "\",\"preventive_side_role_yield_sample_count\":"
         << output.preventive_side_role_yield_sample_count
         << ",\"preventive_side_role_yield_sample_span_sec\":"
         << output.preventive_side_role_yield_sample_span_sec
         << ",\"preventive_side_role_peer_tangent_speed_mps\":";
    append_finite(output.preventive_side_role_peer_tangent_speed_mps);
    json << ",\"preventive_side_role_peer_id\":\"";
    append_json_escaped(output.preventive_side_role_peer_id.c_str());
    json << "\",\"preventive_side_role_clearance_m\":";
    append_finite(output.preventive_side_role_clearance_m);
    const auto &role_eligibility =
        output.preventive_side_role_eligibility_diagnostic;
    json << ",\"preventive_side_role_eligibility_failure\":\""
         << toString(role_eligibility.failure)
         << "\",\"preventive_side_role_eligibility_peer_id\":\"";
    append_json_escaped(role_eligibility.peer_id.c_str());
    json << "\",\"preventive_side_role_matching_peer_ids\":"
         << role_eligibility.matching_peer_ids
         << ",\"preventive_side_role_eligibility_clearance_m\":";
    append_finite(role_eligibility.clearance_m);
    json << ",\"preventive_side_role_delta_s_m\":";
    append_finite(role_eligibility.delta_s_m);
    json << ",\"preventive_side_role_lateral_separation_m\":";
    append_finite(role_eligibility.lateral_separation_m);
    json << ",\"preventive_side_role_ego_tangent_progress_mps\":";
    append_finite(role_eligibility.ego_tangent_progress_mps);
    json << ",\"preventive_side_role_peer_tangent_progress_mps\":";
    append_finite(role_eligibility.peer_tangent_progress_mps);
    json << ",\"preventive_side_role_ego_track_heading_error_rad\":";
    append_finite(role_eligibility.ego_track_heading_error_rad);
    json << ",\"preventive_side_role_peer_track_heading_error_rad\":";
    append_finite(role_eligibility.peer_track_heading_error_rad);
    json << ",\"preventive_side_role_relative_heading_error_rad\":";
    append_finite(role_eligibility.relative_heading_error_rad);
    json << ",\"preventive_side_role_predicted_time_to_hard_sec\":";
    append_finite(role_eligibility.predicted_time_to_hard_sec);
    json << ",\"preventive_side_role_predicted_min_clearance_m\":";
    append_finite(role_eligibility.predicted_min_clearance_m);
    json << ",\"preventive_side_role_synchronized_stamp_sec\":";
    append_finite(role_eligibility.synchronized_stamp_sec);
    json << ",\"preventive_side_role_synchronized_ego_stamp_sec\":";
    append_finite(role_eligibility.synchronized_ego_stamp_sec);
    json << ",\"preventive_side_role_synchronized_peer_stamp_sec\":";
    append_finite(role_eligibility.synchronized_peer_stamp_sec);
    json << ",\"preventive_side_role_cartesian_longitudinal_m\":";
    append_finite(role_eligibility.cartesian_longitudinal_m);
    json << ",\"preventive_side_role_cartesian_lateral_m\":";
    append_finite(role_eligibility.cartesian_lateral_m);
    json << ",\"preventive_side_role_decision_deadline_sec\":";
    append_finite(role_eligibility.decision_deadline_sec);
    json << ",\"preventive_side_role_entry_reason\":\"";
    append_json_escaped(role_eligibility.entry_reason.c_str());
    json << "\"";
    json << ",\"preventive_side_role_ego_relation\":\""
         << toString(role_eligibility.ego_relation)
         << "\",\"preventive_side_role_peer_relation\":\""
         << toString(role_eligibility.peer_relation) << "\"";
    const auto &role_candidate =
        output.preventive_side_role_candidate_diagnostic;
    json << ",\"preventive_side_role_candidate_failure\":\""
         << toString(role_candidate.failure)
         << "\",\"preventive_side_role_candidate_peer_id\":\"";
    append_json_escaped(role_candidate.peer_id.c_str());
    json << "\",\"preventive_side_role_generated_candidates\":"
         << role_candidate.generated_candidates
         << ",\"preventive_side_role_base_feasible_candidates\":"
         << role_candidate.base_feasible_candidates
         << ",\"preventive_side_role_separation_feasible_candidates\":"
         << role_candidate.separation_feasible_candidates
         << ",\"preventive_side_role_violating_sample_index\":"
         << role_candidate.violating_sample_index
         << ",\"preventive_side_role_violating_interpolated_sample\":"
         << (role_candidate.violating_interpolated_sample ? "true" : "false")
         << ",\"preventive_side_role_initial_lateral_gap_m\":";
    append_finite(role_candidate.initial_lateral_gap_m);
    json << ",\"preventive_side_role_observed_lateral_gap_m\":";
    append_finite(role_candidate.observed_lateral_gap_m);
    json << ",\"preventive_side_role_initial_clearance_m\":";
    append_finite(role_candidate.initial_clearance_m);
    json << ",\"preventive_side_role_observed_clearance_m\":";
    append_finite(role_candidate.observed_clearance_m);
    json << ",\"preventive_side_role_precheck_minimum_opponent_clearance_m\":";
    append_finite(role_candidate.minimum_opponent_clearance_m);
    json << ",\"preventive_side_role_precheck_minimum_wall_clearance_m\":";
    append_finite(role_candidate.minimum_wall_clearance_m);
    json << ",\"preventive_side_role_precheck_observational_only\":"
         << (role_candidate.precheck_clearance_observational_only ? "true"
                                                                  : "false");
    json << ",\"preventive_side_role_evaluation_generation\":"
         << role_candidate.evaluation_generation
         << ",\"preventive_side_role_first_reject_reason\":\"";
    append_json_escaped(role_candidate.first_reject_reason.c_str());
    json << "\",\"preventive_side_role_required_gap_m\":";
    append_finite(output.preventive_side_role_required_gap_m);
    json << ",\"preventive_side_role_current_gap_m\":";
    append_finite(output.preventive_side_role_current_gap_m);
    json << ",\"preventive_side_role_requested_speed_cap_mps\":";
    append_finite(output.preventive_side_role_requested_speed_cap_mps);
    json << ",\"preventive_side_role_applied_speed_cap_mps\":";
    append_finite(output.preventive_side_role_applied_speed_cap_mps);
    json << ",\"preventive_side_role_included_opponent_ids\":\"";
    append_json_escaped(
        output.preventive_side_role_included_opponent_ids.c_str());
    json << "\"";
    json << ",\"pass_continuation_latched\":"
         << (output.pass_continuation_latched ? "true" : "false")
         << ",\"pass_continuation_active\":"
         << (output.pass_continuation_active ? "true" : "false")
         << ",\"pass_continuation_suspended\":"
         << (output.pass_continuation_suspended ? "true" : "false")
         << ",\"pass_continuation_target_id\":\"";
    append_json_escaped(output.pass_continuation_target_id.c_str());
    json << "\",\"pass_continuation_side\":" << output.pass_continuation_side
         << ",\"pass_continuation_lateral_index\":"
         << output.pass_continuation_lateral_index
         << ",\"pass_continuation_tangent_index\":"
         << output.pass_continuation_tangent_index
         << ",\"pass_continuation_goal_d_m\":";
    append_finite(output.pass_continuation_goal_d_m);
    json << ",\"pass_continuation_required_arc_m\":";
    append_finite(output.pass_continuation_required_arc_m);
    json << ",\"pass_continuation_target_observation_stamp_sec\":";
    append_finite(output.pass_continuation_target_observation_stamp_sec);
    json << ",\"mpc_health_guard_active\":"
         << (output.mpc_health_guard_active ? "true" : "false")
         << ",\"safety_evaluation_enabled\":"
         << (config_.safety_evaluation_enabled ? "true" : "false")
         << ",\"controller_trackability_profile\":\""
         << toString(config_.controller_trackability_profile) << "\""
         << ",\"ay0_shadow_capture_enabled\":"
         << (ay0_shadow_capture_enabled_ ? "true" : "false")
         << ",\"ay0_shadow_base_reject_count\":"
         << ay0_shadow_base_reject_count_
         << ",\"ay0_shadow_capture_reject_count\":"
         << ay0_shadow_capture_reject_count_
         << ",\"ay0_shadow_enqueue_reject_count\":"
         << ay0_shadow_enqueue_reject_count_
         << ",\"state_lattice_v2_publish_reject_count\":"
         << state_lattice_v2_publish_reject_count_
         << ",\"state_lattice_v2_base_attestation_present\":"
         << (state_lattice_v2_base_attestation_.has_value() ? "true" : "false")
         << ",\"state_lattice_v2_base_attestation_receive_count\":"
         << state_lattice_v2_base_attestation_receive_count_
         << ",\"state_lattice_v2_base_attestation_accept_count\":"
         << state_lattice_v2_base_attestation_accept_count_
         << ",\"state_lattice_v2_base_attestation_reject_count\":"
         << state_lattice_v2_base_attestation_reject_count_
         << ",\"state_lattice_v2_last_base_attestation_validation_reason\":"
         << static_cast<unsigned int>(
                state_lattice_v2_last_base_attestation_validation_reason_)
         << ",\"state_lattice_v2_last_base_attestation_exact_source\":"
         << (state_lattice_v2_last_base_attestation_exact_source_ ? "true"
                                                                  : "false")
         << ",\"state_lattice_v2_last_publish_reason\":\"";
    append_json_escaped(state_lattice_v2_last_publish_reason_.c_str());
    json << "\",\"ay0_shadow_worker\":{";
    const auto ay0_shadow_diagnostics =
        ay0_shadow_worker_session_ == nullptr
            ? c002ay0_shadow::FixedProposalWorkerDiagnostics{}
            : ay0_shadow_worker_session_->diagnostics();
    json << "\"accepting\":"
         << (ay0_shadow_diagnostics.accepting ? "true" : "false")
         << ",\"ready\":"
         << (ay0_shadow_diagnostics.worker_ready ? "true" : "false")
         << ",\"dropped_full_count\":"
         << ay0_shadow_diagnostics.dropped_full_count
         << ",\"invalid_record_count\":"
         << ay0_shadow_diagnostics.invalid_record_count
         << ",\"validation_reject_count\":"
         << ay0_shadow_diagnostics.validation_reject_count
         << ",\"serialization_reject_count\":"
         << ay0_shadow_diagnostics.serialization_reject_count
         << ",\"coalesced_count\":" << ay0_shadow_diagnostics.coalesced_count
         << ",\"published_count\":" << ay0_shadow_diagnostics.published_count
         << ",\"worker_process_age_ns\":"
         << ay0_shadow_diagnostics.worker_process_age_ns
         << ",\"last_publish_queue_age_ns\":"
         << ay0_shadow_diagnostics.last_publish_queue_age_ns << "}"
         << ",\"overtake_permission_allowed\":"
         << (output.overtake_permission_allowed ? "true" : "false")
         << ",\"overtake_permission_section_name\":\""
         << output.overtake_permission_section_name
         << "\",\"overtake_permission_reason\":\""
         << output.overtake_permission_reason << "\""
         << ",\"generated_candidates\":" << output.generated_candidates
         << ",\"feasible_candidates\":" << output.feasible_candidates
         << ",\"rejected_wall_candidates\":" << output.rejected_wall_candidates
         << ",\"rejected_opponent_candidates\":"
         << output.rejected_opponent_candidates
         << ",\"rejected_curvature_candidates\":"
         << output.rejected_curvature_candidates
         << ",\"rejected_trackability_candidates\":"
         << output.rejected_trackability_candidates
         << ",\"rejected_other_candidates\":"
         << output.rejected_other_candidates << ",\"candidate_diagnostics\":[";
    for (std::size_t i = 0U;
         i < planner_cycle_metrics.candidate_diagnostic_count; ++i) {
      if (i != 0U) {
        json << ",";
      }
      const auto &candidate = planner_cycle_metrics.candidate_diagnostics[i];
      json << "{\"lateral_index\":" << candidate.lateral_index
           << ",\"tangent_index\":" << candidate.tangent_index
           << ",\"goal_d_m\":";
      append_finite(candidate.goal_d_m);
      json << ",\"tangent_scale\":";
      append_finite(candidate.tangent_scale);
      json << ",\"required_arc_m\":";
      append_finite(candidate.required_arc_m);
      json << ",\"total_cost\":" << candidate.total_cost
           << ",\"reference_cost\":" << candidate.reference_cost
           << ",\"wall_cost\":" << candidate.wall_cost
           << ",\"object_cost\":" << candidate.object_cost
           << ",\"representative_wall_diagnostic_observational_only\":true"
           << ",\"representative_wall_diagnostic_valid\":"
           << (candidate.representative_wall_diagnostic_valid ? "true"
                                                              : "false")
           << ",\"representative_minimum_nominal_wall_clearance_proxy_m\":";
      append_finite(
          candidate.representative_minimum_nominal_wall_clearance_proxy_m);
      json
          << ",\"representative_minimum_nominal_wall_clearance_point_index\":"
          << candidate.representative_minimum_nominal_wall_clearance_point_index
          << ",\"representative_maximum_nominal_wall_level\":"
          << candidate.representative_maximum_nominal_wall_level
          << ",\"representative_maximum_nominal_wall_level_point_index\":"
          << candidate.representative_maximum_nominal_wall_level_point_index
          << ",\"representative_object_diagnostics_observational_only\":true"
          << ",\"representative_object_diagnostics\":[";
      for (std::size_t object_index = 0U;
           object_index < candidate.representative_object_diagnostic_count;
           ++object_index) {
        if (object_index != 0U) {
          json << ",";
        }
        const auto &object =
            candidate.representative_object_diagnostics[object_index];
        json << "{\"representative_point_index\":" << object_index
             << ",\"winner_valid\":" << (object.winner_valid ? "true" : "false")
             << ",\"prediction_valid\":"
             << (object.prediction_valid ? "true" : "false")
             << ",\"object_level\":" << object.object_level
             << ",\"clearance_m\":";
        append_finite(object.clearance_m);
        json << ",\"prediction_horizon_sec\":";
        append_finite(object.prediction_horizon_sec);
        json << ",\"opponent_id\":\"";
        append_json_escaped(object.opponent_id.data());
        json << "\",\"opponent_id_truncated\":"
             << (object.opponent_id_truncated ? "true" : "false") << "}";
      }
      json << "]"
           << ",\"opponent_clearance_recovery_m\":";
      append_finite(candidate.opponent_clearance_recovery_m);
      json << ",\"start_pose\":{\"x_m\":";
      append_finite(candidate.start_x_m);
      json << ",\"y_m\":";
      append_finite(candidate.start_y_m);
      json << ",\"yaw_rad\":";
      append_finite(candidate.start_yaw_rad);
      json << ",\"s_m\":";
      append_finite(candidate.start_s_m);
      json << ",\"d_m\":";
      append_finite(candidate.start_d_m);
      json << "},\"first_reject_reason\":\"";
      append_json_escaped(candidate.first_reject_reason.data());
      json << "\"}";
    }
    json << "]"
         << ",\"front_detection_radius_m\":" << output.front_detection_radius_m
         << ",\"front_detection_transition_distance_m\":"
         << output.front_detection_transition_distance_m;
    const auto &front_detection = output.front_detection_diagnostic;
    json << ",\"front_detection\":{\"evaluated\":"
         << (front_detection.evaluated ? "true" : "false")
         << ",\"selected_target_id\":\"";
    append_json_escaped(front_detection.selected_target_id.data());
    json << "\",\"enter_cycles\":" << front_detection.enter_cycles
         << ",\"clear_cycles\":" << front_detection.clear_cycles
         << ",\"detected_latched\":"
         << (front_detection.detected_latched ? "true" : "false")
         << ",\"dropped_opponent_count\":"
         << front_detection.dropped_opponent_count << ",\"opponents\":[";
    for (std::size_t i = 0U; i < front_detection.opponent_count; ++i) {
      if (i != 0U) {
        json << ",";
      }
      const auto &entry = front_detection.opponents[i];
      json << "{\"id\":\"";
      append_json_escaped(entry.opponent_id.data());
      json << "\",\"target_class\":\"" << toString(entry.target_class)
           << "\",\"first_false\":\"" << toString(entry.first_false)
           << "\",\"forward_gap_m\":";
      append_finite(entry.forward_gap_m);
      json << ",\"rear_gap_m\":";
      append_finite(entry.rear_gap_m);
      json << ",\"detection_radius_m\":";
      append_finite(entry.detection_radius_m);
      json << ",\"minimum_sweep_distance_m\":";
      append_finite(entry.minimum_sweep_distance_m);
      json << "}";
    }
    const auto &early_aware = output.early_aware_diagnostic;
    json << "]}"
         << ",\"early_aware\":{\"evaluated\":"
         << (early_aware.evaluated ? "true" : "false")
         << ",\"active\":" << (early_aware.active ? "true" : "false")
         << ",\"state\":\"" << toString(early_aware.state)
         << "\",\"target_id\":\"";
    append_json_escaped(early_aware.target_id.data());
    json << "\",\"distinct_fresh_stamp_count\":"
         << early_aware.distinct_fresh_stamp_count
         << ",\"target_observation_stamp_sec\":";
    append_finite(early_aware.target_observation_stamp_sec);
    json << ",\"forward_delta_s_m\":";
    append_finite(early_aware.forward_delta_s_m);
    json << ",\"dynamic_distance_m\":";
    append_finite(early_aware.dynamic_distance_m);
    json << ",\"ego_tangent_speed_mps\":";
    append_finite(early_aware.ego_tangent_speed_mps);
    json << ",\"target_tangent_speed_mps\":";
    append_finite(early_aware.target_tangent_speed_mps);
    json << ",\"projected_closing_speed_mps\":";
    append_finite(early_aware.projected_closing_speed_mps);
    json << ",\"reason\":\"";
    append_json_escaped(early_aware.reason.c_str());
    json << "\"}"
         << ",\"current_pose_collision_kind\":\""
         << toString(output.current_collision_kind) << "\""
         << ",\"current_pose_opponent_relation\":\""
         << toString(output.current_opponent_relation) << "\""
         << ",\"current_pose_rear_only_exempt\":"
         << (output.current_pose_rear_only_exempt ? "true" : "false")
         << ",\"current_pose_rear_only_exempt_opponent_id\":\""
         << output.current_pose_rear_only_exempt_opponent_id << "\""
         << ",\"current_pose_rear_only_exempt_clearance_m\":";
    append_finite(output.current_pose_rear_only_exempt_clearance_m);
    json << ",\"blocking_opponent_id\":\"" << output.blocking_opponent_id
         << "\""
         << ",\"blocking_clearance_m\":";
    if (std::isfinite(output.blocking_clearance_m)) {
      json << output.blocking_clearance_m;
    } else {
      json << "null";
    }
    json << ",\"blocking_required_clearance_m\":"
         << output.blocking_required_clearance_m
         << ",\"output_horizon_first_false\":\""
         << toString(output.output_horizon_diagnostic.failure) << "\""
         << ",\"output_horizon_nearest_shift\":"
         << output.output_horizon_diagnostic.nearest_shift
         << ",\"output_horizon_layout_offset\":"
         << output.output_horizon_diagnostic.layout_offset
         << ",\"output_horizon_waypoint_index\":"
         << output.output_horizon_diagnostic.waypoint_index
         << ",\"output_horizon_interpolation_piece\":"
         << output.output_horizon_diagnostic.interpolation_piece
         << ",\"output_horizon_reference_index\":"
         << output.output_horizon_diagnostic.reference_index
         << ",\"output_horizon_opponent_id\":\""
         << output.output_horizon_diagnostic.opponent_id << "\""
         << ",\"output_horizon_candidate_goal_d_m\":";
    append_finite(output.output_horizon_diagnostic.candidate_goal_d_m);
    json << ",\"output_horizon_candidate_tangent_scale\":";
    append_finite(output.output_horizon_diagnostic.candidate_tangent_scale);
    json << ",\"output_horizon_s_m\":";
    append_finite(output.output_horizon_diagnostic.s_m);
    json << ",\"output_horizon_d_m\":";
    append_finite(output.output_horizon_diagnostic.d_m);
    json << ",\"output_horizon_x_m\":";
    append_finite(output.output_horizon_diagnostic.x_m);
    json << ",\"output_horizon_y_m\":";
    append_finite(output.output_horizon_diagnostic.y_m);
    json << ",\"output_horizon_curvature_radpm\":";
    append_finite(output.output_horizon_diagnostic.curvature_radpm);
    json << ",\"output_horizon_observed_value\":";
    append_finite(output.output_horizon_diagnostic.observed_value);
    json << ",\"output_horizon_limit_value\":";
    append_finite(output.output_horizon_diagnostic.limit_value);
    json << ",\"return_ready_cycles\":" << output.return_ready_cycles
         << ",\"generation\":" << generation_
         << ",\"speed_evidence_mailbox_state\":"
         << static_cast<unsigned>(speed_evidence_commit_state_.mailbox.state())
         << ",\"speed_evidence_drop_count\":"
         << speed_evidence_commit_state_.mailbox.dropCount();
    const bool speed_evidence_pair_ready =
        speed_evidence_commit_state_.mailbox.state() ==
        SpeedEvidenceMailboxState::READY;
    if (speed_evidence_pair_ready) {
      const auto double_bits = [](double value) noexcept {
        std::uint64_t bits = 0U;
        static_assert(sizeof(bits) == sizeof(value));
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
      };
      const auto append_record = [&json, &double_bits](
                                     const char *name,
                                     const CommittedSpeedEvidence &record) {
        const auto &identity = record.identity;
        const auto &trial = record.trial;
        const auto &binding = trial.payload_binding;
        json << ",\"" << name << "\":{";
        json << "\"session\":" << identity.planner_session_id
             << ",\"epoch\":" << identity.state_epoch
             << ",\"attempt\":" << identity.attempt_ordinal
             << ",\"commit\":" << identity.commit_ordinal
             << ",\"predecessor_commit\":"
             << identity.predecessor_commit_ordinal
             << ",\"previous_committed_attempt\":"
             << identity.previous_committed_attempt_ordinal
             << ",\"wire_generation\":" << identity.wire_semantic_generation
             << ",\"record_sequence\":" << identity.producer_record_sequence
             << ",\"drop_count\":" << identity.cumulative_drop_count
             << ",\"sealed_after_commit\":"
             << static_cast<unsigned>(record.sealed_after_commit)
             << ",\"capture_state\":"
             << static_cast<unsigned>(trial.capture_state)
             << ",\"failure\":" << static_cast<unsigned>(trial.failure)
             << ",\"branch\":" << static_cast<unsigned>(trial.branch)
             << ",\"present_fields\":" << trial.present_fields
             << ",\"logical_stop\":"
             << static_cast<unsigned>(trial.logical_stop)
             << ",\"target_missing_recovery\":"
             << static_cast<unsigned>(trial.target_missing_recovery)
             << ",\"state_changed\":"
             << static_cast<unsigned>(trial.state_changed)
             << ",\"effective_dt_bits\":" << double_bits(trial.effective_dt_sec)
             << ",\"requested_speed_bits\":"
             << double_bits(trial.requested_speed_mps)
             << ",\"lower_speed_limit_bits\":"
             << double_bits(trial.lower_speed_limit_mps)
             << ",\"upper_speed_limit_bits\":"
             << double_bits(trial.upper_speed_limit_mps)
             << ",\"pre_command_speed_bits\":"
             << double_bits(trial.pre_state.command_speed_mps)
             << ",\"pre_acceleration_bits\":"
             << double_bits(trial.pre_state.acceleration_mps2)
             << ",\"pre_initialized\":"
             << static_cast<unsigned>(trial.pre_state.initialized)
             << ",\"pre_safe_stop_latched\":"
             << static_cast<unsigned>(trial.pre_state.safe_stop_latched)
             << ",\"pre_safe_stop_release_count\":"
             << trial.pre_state.safe_stop_release_count
             << ",\"post_command_speed_bits\":"
             << double_bits(trial.post_state.command_speed_mps)
             << ",\"post_acceleration_bits\":"
             << double_bits(trial.post_state.acceleration_mps2)
             << ",\"post_initialized\":"
             << static_cast<unsigned>(trial.post_state.initialized)
             << ",\"post_safe_stop_latched\":"
             << static_cast<unsigned>(trial.post_state.safe_stop_latched)
             << ",\"post_safe_stop_release_count\":"
             << trial.post_state.safe_stop_release_count
             << ",\"session_generation\":" << binding.session_generation
             << ",\"session_nonce\":" << binding.session_nonce
             << ",\"plan_generation\":" << binding.plan_generation
             << ",\"candidate_revision\":" << binding.candidate_revision
             << ",\"planner_instance_id\":" << binding.planner_instance_id
             << ",\"proposal_attempt_id\":" << binding.attempt_id
             << ",\"connector_transaction_id\":"
             << binding.connector_transaction_id
             << ",\"authority_token\":" << binding.authority_token
             << ",\"safety_snapshot_id\":" << binding.safety_snapshot_id
             << ",\"race_arm_epoch\":" << binding.race_arm_epoch
             << ",\"controller_instance_id\":" << binding.controller_instance_id
             << ",\"controller_sequence\":" << binding.controller_sequence
             << ",\"base_lease_id\":" << binding.base_lease_id
             << ",\"base_source_stamp_sec\":" << binding.base_source_stamp_sec
             << ",\"base_source_stamp_nanosec\":"
             << binding.base_source_stamp_nanosec
             << ",\"base_source_generation\":" << binding.base_source_generation
             << "}";
      };
      append_record("speed_evidence_predecessor",
                    speed_evidence_commit_state_.mailbox.predecessor());
      append_record("speed_evidence_target",
                    speed_evidence_commit_state_.mailbox.target());
    }
    json << "}";
    metrics.data = json.str();
    metrics_pub_->publish(metrics);
    if (speed_evidence_pair_ready) {
      // The ROS publish is the existing process-external, rosbag-recorded
      // artifact seam. Consume only after the complete pair was published.
      speed_evidence_commit_state_.mailbox.consume();
    }
    publishPaths(output);
  }

  void publishPaths(const PlannerOutput &output) {
    const auto stamp = now();
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_candidates;
    clear_candidates.header.frame_id = config_.map_frame;
    clear_candidates.header.stamp = stamp;
    clear_candidates.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(std::move(clear_candidates));
    const bool has_current_candidates = output.generated_candidates > 0;
    if (has_current_candidates) {
      int id = 0;
      for (const auto &candidate : planner_->candidates()) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = config_.map_frame;
        marker.header.stamp = stamp;
        marker.ns = "state_lattice_candidates";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.04;
        marker.color.a = 0.8F;
        marker.color.g = candidate.feasible ? 1.0F : 0.0F;
        marker.color.r = candidate.feasible ? 0.0F : 1.0F;
        for (const auto &point : candidate.dense) {
          geometry_msgs::msg::Point p;
          p.x = point.x;
          p.y = point.y;
          marker.points.push_back(p);
        }
        markers.markers.push_back(std::move(marker));
      }
    }
    candidates_pub_->publish(markers);
    nav_msgs::msg::Path selected_path;
    selected_path.header.frame_id = config_.map_frame;
    selected_path.header.stamp = stamp;
    if (has_current_candidates && planner_->selected().has_value()) {
      for (const auto &point : planner_->selected()->dense) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = selected_path.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.orientation = quaternionFromYaw(point.yaw);
        selected_path.poses.push_back(pose);
      }
    }
    selected_pub_->publish(selected_path);
    visualization_msgs::msg::MarkerArray targets;
    visualization_msgs::msg::Marker terminal_marker;
    terminal_marker.header = selected_path.header;
    terminal_marker.ns = "front_detection_terminals";
    terminal_marker.id = 0;
    terminal_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    terminal_marker.action = visualization_msgs::msg::Marker::ADD;
    terminal_marker.scale.x = 0.25;
    terminal_marker.scale.y = 0.25;
    terminal_marker.scale.z = 0.25;
    terminal_marker.color.a = 1.0F;
    terminal_marker.color.b = 1.0F;
    for (const auto &point : planner_->detector().currentTerminals()) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      terminal_marker.points.push_back(p);
    }
    targets.markers.push_back(std::move(terminal_marker));
    targets_pub_->publish(targets);
    publishPlanningGeometry(stamp);
    visualization_msgs::msg::MarkerArray rear;
    visualization_msgs::msg::Marker rear_marker;
    rear_marker.header = selected_path.header;
    rear_marker.ns = "rear_safety";
    rear_marker.id = 0;
    rear_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    rear_marker.action = visualization_msgs::msg::Marker::ADD;
    rear_marker.scale.x = 0.08;
    rear_marker.color.a = 1.0F;
    rear_marker.color.r = 1.0F;
    rear_marker.color.b = 1.0F;
    for (const auto &point : planner_->rearSafetyPath()) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      rear_marker.points.push_back(p);
    }
    rear.markers.push_back(std::move(rear_marker));
    rear_pub_->publish(rear);
  }

  void publishPlanningGeometry(const rclcpp::Time &stamp) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = config_.map_frame;
    clear.header.stamp = stamp;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(std::move(clear));
    if (map_.initialized()) {
      visualization_msgs::msg::Marker bounds;
      bounds.header.frame_id = config_.map_frame;
      bounds.header.stamp = stamp;
      bounds.ns = "costmap_bounds";
      bounds.id = 0;
      bounds.type = visualization_msgs::msg::Marker::LINE_STRIP;
      bounds.action = visualization_msgs::msg::Marker::ADD;
      bounds.scale.x = 0.08;
      bounds.color.r = 1.0F;
      bounds.color.g = 0.55F;
      bounds.color.a = 1.0F;
      appendRectangle(&bounds, map_.originX(), map_.originY(),
                      map_.originX() + map_.width() * map_.resolution(),
                      map_.originY() + map_.height() * map_.resolution(), 0.08);
      markers.markers.push_back(std::move(bounds));

      visualization_msgs::msg::Marker label;
      label.header.frame_id = config_.map_frame;
      label.header.stamp = stamp;
      label.ns = "costmap_bounds";
      label.id = 1;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position = markerPoint(map_.originX(), map_.originY(), 0.35);
      label.scale.z = 0.45;
      label.color.r = 1.0F;
      label.color.g = 0.7F;
      label.color.b = 0.2F;
      label.color.a = 1.0F;
      std::ostringstream text;
      text << std::fixed << std::setprecision(1) << "costmap "
           << map_.width() * map_.resolution() << " x "
           << map_.height() * map_.resolution() << " m, res "
           << std::setprecision(2) << map_.resolution() << " m";
      label.text = text.str();
      markers.markers.push_back(std::move(label));
    }

    const auto &starts = planner_->detector().sweepStarts();
    const auto &terminals = planner_->detector().currentTerminals();
    const double detection_radius = planner_->detector().detectionRadiusM();
    const std::size_t sweep_count = std::min(starts.size(), terminals.size());
    for (std::size_t i = 0; i < sweep_count; ++i) {
      visualization_msgs::msg::Marker area;
      area.header.frame_id = config_.map_frame;
      area.header.stamp = stamp;
      area.ns = "front_detection_area";
      area.id = static_cast<int>(i);
      area.type = visualization_msgs::msg::Marker::LINE_STRIP;
      area.action = visualization_msgs::msg::Marker::ADD;
      area.scale.x = 2.0 * detection_radius;
      area.color.r = 0.1F;
      area.color.g = 0.8F;
      area.color.b = 1.0F;
      area.color.a = 0.12F;
      area.points.push_back(markerPoint(starts[i].x, starts[i].y, 0.03));
      area.points.push_back(markerPoint(terminals[i].x, terminals[i].y, 0.03));
      markers.markers.push_back(std::move(area));

      visualization_msgs::msg::Marker outline;
      outline.header.frame_id = config_.map_frame;
      outline.header.stamp = stamp;
      outline.ns = "front_detection_outline";
      outline.id = static_cast<int>(i);
      outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
      outline.action = visualization_msgs::msg::Marker::ADD;
      outline.scale.x = 0.04;
      outline.color.r = 0.1F;
      outline.color.g = 0.9F;
      outline.color.b = 1.0F;
      outline.color.a = 0.9F;
      appendCapsuleOutline(&outline, starts[i], terminals[i], detection_radius,
                           0.06);
      markers.markers.push_back(std::move(outline));
    }

    if (planner_->active() && map_.initialized()) {
      int marker_id = 0;
      for (const auto &opponent : opponents_) {
        if (!opponent.valid) {
          continue;
        }
        const double radius =
            3.0 + std::max(opponent.uncertainty_x_m, opponent.uncertainty_y_m);
        visualization_msgs::msg::Marker update_bounds;
        update_bounds.header.frame_id = config_.map_frame;
        update_bounds.header.stamp = stamp;
        update_bounds.ns = "dynamic_cost_update_bounds";
        update_bounds.id = marker_id++;
        update_bounds.type = visualization_msgs::msg::Marker::LINE_STRIP;
        update_bounds.action = visualization_msgs::msg::Marker::ADD;
        update_bounds.scale.x = 0.05;
        update_bounds.color.r = 1.0F;
        update_bounds.color.g = 0.1F;
        update_bounds.color.b = 0.7F;
        update_bounds.color.a = 0.9F;
        appendRectangle(&update_bounds, opponent.x - radius,
                        opponent.y - radius, opponent.x + radius,
                        opponent.y + radius, 0.10);
        markers.markers.push_back(std::move(update_bounds));
      }
    }
    geometry_pub_->publish(markers);
  }

  void publishCostmap() {
    if (!map_.initialized() ||
        map_.referenceLevels().size() != map_.wallLevels().size()) {
      return;
    }
    nav_msgs::msg::OccupancyGrid message;
    const auto stamp = now();
    message.header.stamp = stamp;
    message.header.frame_id = config_.map_frame;
    message.info.resolution = static_cast<float>(map_.resolution());
    message.info.width = static_cast<std::uint32_t>(map_.width());
    message.info.height = static_cast<std::uint32_t>(map_.height());
    message.info.origin.position.x = map_.originX();
    message.info.origin.position.y = map_.originY();
    message.info.origin.orientation.w = 1.0;
    message.data.resize(map_.width() * map_.height());

    nav_msgs::msg::OccupancyGrid wall_message = message;
    wall_message.data.resize(map_.occupiedCells().size());
    std::transform(map_.occupiedCells().begin(), map_.occupiedCells().end(),
                   wall_message.data.begin(), [](const std::uint8_t occupied) {
                     return static_cast<std::int8_t>(occupied != 0U ? 100 : 0);
                   });
    wall_map_pub_->publish(wall_message);

    if (!last_wall_costmap_generation_.has_value() ||
        last_wall_costmap_generation_.value() != map_.staticGeneration()) {
      nav_msgs::msg::OccupancyGrid wall_cost_message = message;
      wall_cost_message.data.resize(message.data.size(), 0);
      for (std::size_t i = 0; i < wall_cost_message.data.size(); ++i) {
        const int level = static_cast<int>(map_.wallLevels()[i]);
        if (level > 0) {
          wall_cost_message.data[i] = static_cast<std::int8_t>(
              config_.cost_levels[static_cast<std::size_t>(level)]);
        }
      }
      wall_costmap_pub_->publish(wall_cost_message);
      last_wall_costmap_generation_ = map_.staticGeneration();
    }

    for (std::size_t i = 0; i < message.data.size(); ++i) {
      const int level =
          mergeCostLevels(map_.wallLevels()[i], map_.referenceLevels()[i]);
      message.data[i] = static_cast<std::int8_t>(
          config_.cost_levels[static_cast<std::size_t>(level)]);
    }
    std::optional<nav_msgs::msg::OccupancyGrid> opponent_cost_message;
    if (opponent_costmap_pub_->get_subscription_count() > 0U) {
      opponent_cost_message = message;
      std::fill(opponent_cost_message->data.begin(),
                opponent_cost_message->data.end(), 0);
    }
    if (planner_->active() && inputsFresh(stamp.seconds())) {
      for (const auto &opponent : opponents_) {
        if (!opponent.valid) {
          continue;
        }
        const double radius =
            3.0 + std::max(opponent.uncertainty_x_m, opponent.uncertainty_y_m);
        int min_x = 0, min_y = 0, max_x = 0, max_y = 0;
        map_.worldToCell(opponent.x - radius, opponent.y - radius, &min_x,
                         &min_y);
        map_.worldToCell(opponent.x + radius, opponent.y + radius, &max_x,
                         &max_y);
        min_x = std::clamp(min_x, 0, static_cast<int>(map_.width()) - 1);
        max_x = std::clamp(max_x, 0, static_cast<int>(map_.width()) - 1);
        min_y = std::clamp(min_y, 0, static_cast<int>(map_.height()) - 1);
        max_y = std::clamp(max_y, 0, static_cast<int>(map_.height()) - 1);
        for (int y = min_y; y <= max_y; ++y) {
          for (int x = min_x; x <= max_x; ++x) {
            const auto center = map_.cellCenter(x, y);
            const double dx = std::max(0.0, std::abs(center.x - opponent.x) -
                                                opponent.uncertainty_x_m);
            const double dy = std::max(0.0, std::abs(center.y - opponent.y) -
                                                opponent.uncertainty_y_m);
            const int object_level =
                objectCostLevel(std::hypot(dx, dy), config_);
            const auto index = static_cast<std::size_t>(y) * map_.width() +
                               static_cast<std::size_t>(x);
            if (object_level > 0 && opponent_cost_message.has_value()) {
              const auto isolated_level_it = std::find(
                  config_.cost_levels.begin(), config_.cost_levels.end(),
                  opponent_cost_message->data[index]);
              const int isolated_level =
                  isolated_level_it == config_.cost_levels.end()
                      ? 0
                      : static_cast<int>(std::distance(
                            config_.cost_levels.begin(), isolated_level_it));
              opponent_cost_message->data[index] = static_cast<std::int8_t>(
                  config_.cost_levels[static_cast<std::size_t>(
                      mergeCostLevels(isolated_level, object_level))]);
            }
            const auto current_level_it =
                std::find(config_.cost_levels.begin(),
                          config_.cost_levels.end(), message.data[index]);
            const int current_level =
                current_level_it == config_.cost_levels.end()
                    ? 0
                    : static_cast<int>(std::distance(
                          config_.cost_levels.begin(), current_level_it));
            message.data[index] = static_cast<std::int8_t>(
                config_.cost_levels[static_cast<std::size_t>(
                    mergeCostLevels(current_level, object_level))]);
          }
        }
      }
    }
    if (opponent_cost_message.has_value()) {
      opponent_costmap_pub_->publish(opponent_cost_message.value());
    }
    costmap_pub_->publish(message);
  }

  PlannerConfig config_;
  bool enabled_{true};
  bool live_control_output_enabled_{false};
  bool instant_control_enabled_{false};
  bool experimental_spatial_reference_override_live_publish_enabled_{false};
  bool state_lattice_v2_live_proposal_publish_enabled_{false};
  std::uint64_t state_lattice_v2_producer_instance_id_{0U};
  bool state_lattice_v2_base_attestation_accept_enabled_{false};
  std::string state_lattice_v2_expected_pp_producer_instance_id_;
  std::string state_lattice_v2_expected_pp_session_id_;
  bool state_lattice_v2_final_fence_enabled_{false};
  std::string state_lattice_v2_final_fence_execution_nonce_;
  std::string state_lattice_v2_final_fence_expected_session_id_;
  std::uint64_t state_lattice_v2_final_fence_sealed_epoch_id_{0U};
  std::string instant_control_topic_;
  bool ay0_shadow_capture_enabled_{false};
  std::string ay0_shadow_worker_path_;
  std::uint64_t ay0_shadow_session_generation_{0U};
  std::uint64_t ay0_shadow_session_nonce_{0U};
  Ay0ShadowSafetyEvidence ay0_shadow_safety_evidence_;
  std::optional<overtake_transport_contract::c002ay0::FixedBaseRecord>
      ay0_base_snapshot_record_;
  std::optional<c002ay0_shadow::FixedIncomingBaseProvenance>
      ay0_base_snapshot_provenance_;
  std::uint64_t ay0_base_snapshot_callback_entry_monotonic_ns_{0U};
  overtake_transport_contract::c002ay0::FixedBaseRecord
      ay0_base_snapshot_record_scratch_{};
  c002ay0_shadow::FixedIncomingBaseProvenance
      ay0_base_snapshot_provenance_scratch_{};
  c002ay0_shadow::FixedProposalRecord ay0_shadow_record_scratch_{};
  std::unique_ptr<c002ay0_shadow::FixedProposalWorkerSession>
      ay0_shadow_worker_session_;
  std::uint64_t ay0_shadow_attempt_id_{0U};
  std::uint64_t ay0_shadow_safety_snapshot_id_{0U};
  std::uint32_t ay0_shadow_candidate_revision_{0U};
  std::uint64_t ay0_shadow_base_reject_count_{0U};
  std::uint64_t ay0_shadow_capture_reject_count_{0U};
  std::uint64_t ay0_shadow_enqueue_reject_count_{0U};
  std::optional<multi_purpose_mpc_ros_msgs::msg::StateLatticeV2BaseAttestation>
      state_lattice_v2_base_attestation_;
  std::uint64_t state_lattice_v2_base_attestation_reject_count_{0U};
  std::uint64_t state_lattice_v2_base_attestation_receive_count_{0U};
  std::uint64_t state_lattice_v2_base_attestation_accept_count_{0U};
  std::uint8_t state_lattice_v2_last_base_attestation_validation_reason_{255U};
  bool state_lattice_v2_last_base_attestation_exact_source_{false};
  // No callback group is specified, so all callbacks use this node's default
  // mutually exclusive group. The planning timer is the sole writer of this
  // state; halt prevents any ambiguous delivery from reusing its identity.
  bool state_lattice_v2_publication_halted_{false};
  StateLatticeV2PublicationState state_lattice_v2_publication_state_;
  std::unique_ptr<StateLatticeV2FinalFenceState>
      state_lattice_v2_final_fence_state_;
  std::uint64_t state_lattice_v2_publish_reject_count_{0U};
  std::string state_lattice_v2_last_publish_reason_{"not_attempted"};
  InstantControllerConfig instant_controller_config_;
  std::unique_ptr<InstantController> instant_controller_;
  std::uint64_t instant_control_producer_instance_id_{0U};
  std::uint64_t instant_control_command_sequence_{0U};
  std::string own_vehicle_id_;
  std::string degraded_reason_;
  FrenetFrame frame_;
  FrenetFrame output_frame_;
  GridMap map_;
  std::unique_ptr<LatticePlanner> planner_;
  EgoState ego_;
  std::vector<OpponentState> opponents_;
  std::unordered_map<std::string, OpponentState> opponent_history_;
  bool v2x_received_{false};
  bool v2x_snapshot_valid_{false};
  double v2x_receive_sec_{0.0};
  MpcHealthStatus mpc_health_;
  std::optional<double> last_mpc_health_sec_;
  std::uint64_t mpc_health_sample_sequence_{0U};
  std::uint32_t generation_{0U};
  std::uint64_t planner_session_id_{0U};
  std::uint64_t speed_evidence_state_epoch_{1U};
  std::uint64_t speed_evidence_attempt_ordinal_{0U};
  SpeedEvidenceCommitState speed_evidence_commit_state_;
  std::optional<WirePayload> last_payload_;
  std::optional<std_msgs::msg::Float32MultiArray> previous_safe_v3_;
  std::optional<PlannerOutput> previous_safe_output_;
  std::optional<std::uint64_t> last_wall_costmap_generation_;
  double previous_safe_time_sec_{0.0};
  int consecutive_overruns_{0};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr override_pub_;
  rclcpp::Publisher<
      multi_purpose_mpc_ros_msgs::msg::StateLatticeControlCommand>::SharedPtr
      instant_control_pub_;
  rclcpp::Publisher<
      multi_purpose_mpc_ros_msgs::msg::AuthorizedCartesianTrajectoryV2>::
      SharedPtr state_lattice_v2_proposal_pub_;
  rclcpp::Subscription<
      multi_purpose_mpc_ros_msgs::msg::StateLatticeV2BaseAttestation>::SharedPtr
      state_lattice_v2_base_attestation_sub_;
  rclcpp::Publisher<multi_purpose_mpc_ros_msgs::msg::StateLatticeV2FinalFence>::
      SharedPtr state_lattice_v2_final_fence_pub_;
  rclcpp::Subscription<
      multi_purpose_mpc_ros_msgs::msg::StateLatticeV2QuiesceRequest>::SharedPtr
      state_lattice_v2_quiesce_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      ay0_planner_input_audit_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr wall_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr wall_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      opponent_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      candidates_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      targets_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rear_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      geometry_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<v2x_msgs::msg::V2XVehiclePositionArray>::SharedPtr
      v2x_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mpc_sub_;
  rclcpp::Subscription<
      multi_purpose_mpc_ros_msgs::msg::ControllerBaseTrajectorySnapshot>::
      SharedPtr ay0_base_snapshot_sub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
};

} // namespace state_lattice_overtake_planner

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<
          state_lattice_overtake_planner::StateLatticeOvertakePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
