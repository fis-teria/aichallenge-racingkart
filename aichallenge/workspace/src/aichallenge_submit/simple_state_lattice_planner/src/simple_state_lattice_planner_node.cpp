#include "simple_state_lattice_planner/instant_controller.hpp"
#include "simple_state_lattice_planner/planner_core.hpp"
#include "simple_state_lattice_planner/static_input_loader.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/state_lattice_control_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <v2x_msgs/msg/v2_x_vehicle_position_array.hpp>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>
#include <sstream>
#include <string>

namespace sl = simple_state_lattice_planner;

namespace {

double stampSec(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) + 1.0e-9 * stamp.nanosec;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

const char *stateName(sl::PlannerState state) {
  return state == sl::PlannerState::FREE_RUN ? "FREE_RUN" : "OVERTAKE";
}

const char *reasonName(sl::OutputReason reason) {
  switch (reason) {
    case sl::OutputReason::CENTER_CLEAR: return "CENTER_CLEAR";
    case sl::OutputReason::CENTER_BLOCKED_SIDE_SELECTED: return "CENTER_BLOCKED_SIDE_SELECTED";
    case sl::OutputReason::OVERTAKE_CONTINUING: return "OVERTAKE_CONTINUING";
    case sl::OutputReason::OPPONENT_PASSED_CENTER_CLEAR: return "OPPONENT_PASSED_CENTER_CLEAR";
    case sl::OutputReason::NO_VALID_CANDIDATE: return "NO_VALID_CANDIDATE";
    case sl::OutputReason::OPPONENT_UNAVAILABLE: return "OPPONENT_UNAVAILABLE";
    default: return "INPUT_INVALID";
  }
}

}  // namespace

class SimpleStateLatticePlannerNode final : public rclcpp::Node {
 public:
  SimpleStateLatticePlannerNode()
      : Node("simple_state_lattice_planner"), planner_(plannerConfig()),
        controller_(controllerConfig()) {
    use_live_output_ = declare_parameter<bool>("live_control_output_enabled", false);
    safety_enabled_ = declare_parameter<bool>("safety_evaluation_enabled", false);
    producer_id_ = static_cast<std::uint64_t>(
        declare_parameter<std::int64_t>("producer_instance_id", 2026080301));
    maximum_input_age_sec_ = declare_parameter<double>("maximum_input_age_sec", 0.20);
    maximum_future_sec_ = declare_parameter<double>("maximum_future_offset_sec", 0.05);
    own_vehicle_id_ = declare_parameter<std::string>("own_vehicle_id", "auto");
    if (own_vehicle_id_ == "auto") {
      const char *environment_id = std::getenv("VEHICLE_ID");
      own_vehicle_id_ = environment_id == nullptr ? "d1" : environment_id;
    }
    const auto reference_package = declare_parameter<std::string>(
        "reference_package", "multi_purpose_mpc_ros");
    const auto reference_relative = declare_parameter<std::string>(
        "reference_csv", "env/final_ver3/traj_mincurv_manual.csv");
    const auto map_package = declare_parameter<std::string>(
        "wall_map_package", "multi_purpose_mpc_ros");
    const auto map_relative = declare_parameter<std::string>(
        "wall_map_yaml", "env/final_ver3/occupancy_grid_map.yaml");
    const auto input_odom = declare_parameter<std::string>(
        "input_odom", "/localization/kinematic_state");
    const auto input_v2x = declare_parameter<std::string>(
        "input_v2x", "/v2x/vehicle_positions");
    const auto output_command = declare_parameter<std::string>(
        "output_control_cmd", "/hybrid_control/state_lattice/control_cmd");

    std::string error;
    try {
      const auto reference_path = ament_index_cpp::get_package_share_directory(reference_package) + "/" + reference_relative;
      const auto map_path = ament_index_cpp::get_package_share_directory(map_package) + "/" + map_relative;
      if (!sl::loadReferenceCsv(reference_path, &static_reference_, &error)) {
        throw std::runtime_error(error);
      }
      sl::Costmap2D raw_map;
      if (!sl::loadOccupancyGridYaml(map_path, &raw_map, &error)) {
        throw std::runtime_error(error);
      }
      const auto inflated = sl::buildCostmap(raw_map, std::nullopt, costmap_config_);
      if (!inflated.valid) throw std::runtime_error(inflated.reason);
      inflated_static_map_ = inflated.costmap;
      static_inputs_valid_ = true;
    } catch (const std::exception &exception) {
      static_error_ = exception.what();
      RCLCPP_ERROR(get_logger(), "static inputs invalid: %s", static_error_.c_str());
    }

    command_pub_ = create_publisher<multi_purpose_mpc_ros_msgs::msg::StateLatticeControlCommand>(output_command, rclcpp::QoS(10));
    mode_pub_ = create_publisher<std_msgs::msg::String>("/debug/overtake/mode", rclcpp::QoS(10));
    metrics_pub_ = create_publisher<std_msgs::msg::String>("/debug/overtake/metrics", rclcpp::QoS(10));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/debug/overtake/selected_path", rclcpp::QoS(10));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_odom, rclcpp::QoS(10), [this](nav_msgs::msg::Odometry::SharedPtr message) { latest_odom_ = std::move(message); });
    v2x_sub_ = create_subscription<v2x_msgs::msg::V2XVehiclePositionArray>(
        input_v2x, rclcpp::QoS(10), [this](v2x_msgs::msg::V2XVehiclePositionArray::SharedPtr message) { latest_v2x_ = std::move(message); });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { cycle(); });
    RCLCPP_INFO(get_logger(), "started: live=%s safety=%s own=%s static=%s",
                use_live_output_ ? "true" : "false", safety_enabled_ ? "true" : "false",
                own_vehicle_id_.c_str(), static_inputs_valid_ ? "valid" : "invalid");
  }

 private:
  static sl::PlannerConfig plannerConfig() {
    sl::PlannerConfig config;
    config.lattice.target_speed_mps = 2.0;
    return config;
  }

  static sl::InstantControllerConfig controllerConfig() {
    return sl::InstantControllerConfig{};
  }

  bool fresh(double source_sec, double now_sec) const {
    return std::isfinite(source_sec) && source_sec > 0.0 &&
           now_sec - source_sec <= maximum_input_age_sec_ &&
           source_sec - now_sec <= maximum_future_sec_;
  }

  std::optional<sl::OpponentState> opponent(std::uint64_t snapshot_id,
                                             double now_sec, bool *valid) const {
    *valid = false;
    if (!latest_v2x_ || latest_v2x_->header.frame_id != "map" ||
        !fresh(stampSec(latest_v2x_->header.stamp), now_sec)) return std::nullopt;
    std::optional<sl::OpponentState> result;
    for (const auto &vehicle : latest_v2x_->vehicles) {
      if (vehicle.vehicle_id == own_vehicle_id_) continue;
      if (result.has_value()) return std::nullopt;
      const double source_sec = stampSec(vehicle.header.stamp);
      if (vehicle.header.frame_id != "map" || !fresh(source_sec, now_sec) ||
          !std::isfinite(vehicle.position.x) || !std::isfinite(vehicle.position.y)) return std::nullopt;
      result = sl::OpponentState{snapshot_id, source_sec, vehicle.position.x,
                                 vehicle.position.y, 0.0, 0.0};
    }
    // A fresh empty array is a valid FREE_RUN observation. OVERTAKE still
    // fails closed inside PlannerCore if its tracked opponent disappears.
    *valid = true;
    return result;
  }

  void cycle() {
    const auto start = std::chrono::steady_clock::now();
    const auto ros_now = now();
    const double now_sec = ros_now.seconds();
    const std::uint64_t snapshot_id = ++snapshot_id_;
    bool inputs_fresh = static_inputs_valid_ && latest_odom_ != nullptr &&
        latest_odom_->header.frame_id == "map" &&
        fresh(stampSec(latest_odom_->header.stamp), now_sec);
    bool opponent_valid = false;
    auto current_opponent = opponent(snapshot_id, now_sec, &opponent_valid);
    inputs_fresh = inputs_fresh && opponent_valid;

    sl::PlannerOutput output;
    sl::InstantControlResult control;
    bool costmap_valid = false;
    sl::EgoState ego;
    if (inputs_fresh) {
      ego.snapshot_id = snapshot_id;
      ego.frame_id = "map";
      ego.stamp_sec = stampSec(latest_odom_->header.stamp);
      ego.x_m = latest_odom_->pose.pose.position.x;
      ego.y_m = latest_odom_->pose.pose.position.y;
      ego.yaw_rad = yawFromQuaternion(latest_odom_->pose.pose.orientation);
      ego.speed_mps = latest_odom_->twist.twist.linear.x;
      inputs_fresh = std::isfinite(ego.x_m) && std::isfinite(ego.y_m) &&
                     std::isfinite(ego.yaw_rad) && std::isfinite(ego.speed_mps) &&
                     ego.speed_mps >= 0.0;
    }
    if (inputs_fresh) {
      auto reference = static_reference_;
      reference.snapshot_id = snapshot_id;
      reference.stamp_sec = now_sec;
      auto static_map = inflated_static_map_;
      static_map.snapshot_id = snapshot_id;
      static_map.stamp_sec = now_sec;
      const auto dynamic_map = sl::addOpponentToInflatedCostmap(static_map, current_opponent, costmap_config_);
      costmap_valid = dynamic_map.valid;
      if (costmap_valid) {
        sl::PlannerInput input{snapshot_id, ego, current_opponent, reference, dynamic_map.costmap};
        output = planner_.plan(input, now_sec);
        const sl::Candidate *selected = nullptr;
        if (output.selected_id.has_value()) {
          for (const auto &candidate : output.candidates) {
            if (candidate.id == *output.selected_id) { selected = &candidate; break; }
          }
        }
        sl::InstantControlInput control_input;
        control_input.ego = &ego;
        control_input.selected_candidate = selected;
        control_input.planner_snapshot_id = snapshot_id;
        control_input.inputs_fresh = true;
        control_input.planner_output_fresh = selected != nullptr && selected->valid &&
                                             selected->reject_reason == sl::RejectReason::NONE;
        control_input.monotonic_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(start.time_since_epoch()).count();
        control = controller_.update(control_input);
      }
    }
    if (!inputs_fresh || !costmap_valid) {
      planner_.reset();
      controller_.reset();
      output.snapshot_id = snapshot_id;
      output.reason = sl::OutputReason::INPUT_INVALID;
      control.reason = inputs_fresh ? "costmap_invalid" : "inputs_stale_or_missing";
      control.acceleration_mps2 = -0.9;
    }
    publishCommand(ros_now, output, control, inputs_fresh, costmap_valid);
    publishDebug(ros_now, output, control, current_opponent, start);
  }

  void publishCommand(const rclcpp::Time &stamp, const sl::PlannerOutput &output,
                      const sl::InstantControlResult &control, bool inputs_fresh,
                      bool costmap_valid) {
    using Message = multi_purpose_mpc_ros_msgs::msg::StateLatticeControlCommand;
    Message message;
    message.header.stamp = stamp;
    message.header.frame_id = "map";
    message.schema_version = Message::SCHEMA_V1;
    message.producer_instance_id = producer_id_;
    message.command_sequence = ++command_sequence_;
    plan_generation_ = plan_generation_ >= 16777215U ? 1U : plan_generation_ + 1U;
    message.plan_generation = plan_generation_;
    message.active = use_live_output_;
    message.safety_evaluation_enabled = safety_enabled_;
    message.inputs_fresh = inputs_fresh;
    message.costmap_valid = costmap_valid;
    const auto selected = std::find_if(
        output.candidates.begin(), output.candidates.end(),
        [&output](const sl::Candidate &candidate) {
          return output.selected_id.has_value() && candidate.id == *output.selected_id;
        });
    const bool evaluator_accepted = selected != output.candidates.end() &&
                                    selected->valid &&
                                    selected->reject_reason == sl::RejectReason::NONE;
    message.trajectory_valid = control.valid && output.selected_id.has_value();
    message.trajectory_safe = message.trajectory_valid && evaluator_accepted;
    message.stop_required = message.active &&
        (!message.safety_evaluation_enabled || !message.inputs_fresh ||
         !message.costmap_valid || !message.trajectory_valid ||
         !message.trajectory_safe || control.stop_required);
    message.command.stamp = stamp;
    message.command.longitudinal.stamp = stamp;
    message.command.lateral.stamp = stamp;
    message.command.longitudinal.speed = message.stop_required ? 0.0F : static_cast<float>(control.speed_mps);
    message.command.longitudinal.acceleration = message.stop_required ? -0.9F : static_cast<float>(control.acceleration_mps2);
    message.command.longitudinal.jerk = 0.0F;
    message.command.lateral.steering_tire_angle = message.stop_required ? 0.0F : static_cast<float>(control.steering_angle_rad);
    message.command.lateral.steering_tire_rotation_rate = message.stop_required ? 0.0F : static_cast<float>(control.steering_rate_radps);
    message.reason = (message.stop_required ? control.reason : reasonName(output.reason)).substr(0U, 128U);
    command_pub_->publish(message);
  }

  void publishDebug(const rclcpp::Time &stamp, const sl::PlannerOutput &output,
                    const sl::InstantControlResult &control,
                    const std::optional<sl::OpponentState> &opponent,
                    std::chrono::steady_clock::time_point start) {
    std_msgs::msg::String mode;
    mode.data = stateName(output.state);
    mode_pub_->publish(mode);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    std_msgs::msg::String metrics;
    std::ostringstream json;
    json << "{\"snapshot_id\":" << output.snapshot_id
         << ",\"state\":\"" << stateName(output.state)
         << "\",\"reason\":\"" << reasonName(output.reason)
         << "\",\"opponent_detected\":" << (opponent.has_value() ? "true" : "false")
         << ",\"selected_id\":";
    if (output.selected_id.has_value()) json << *output.selected_id; else json << "null";
    json << ",\"controller_valid\":" << (control.valid ? "true" : "false")
         << ",\"cycle_ms\":" << elapsed_ms << "}";
    metrics.data = json.str();
    metrics_pub_->publish(metrics);
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = "map";
    for (const auto &point : output.trajectory) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x_m;
      pose.pose.position.y = point.y_m;
      pose.pose.orientation.z = std::sin(0.5 * point.yaw_rad);
      pose.pose.orientation.w = std::cos(0.5 * point.yaw_rad);
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  sl::CostmapBuilderConfig costmap_config_;
  sl::ReferenceWindow static_reference_;
  sl::Costmap2D inflated_static_map_;
  sl::PlannerCore planner_;
  sl::InstantController controller_;
  bool use_live_output_{false};
  bool safety_enabled_{false};
  bool static_inputs_valid_{false};
  std::string static_error_;
  std::string own_vehicle_id_;
  std::uint64_t producer_id_{0U};
  std::uint64_t snapshot_id_{0U};
  std::uint64_t command_sequence_{0U};
  std::uint32_t plan_generation_{0U};
  double maximum_input_age_sec_{0.20};
  double maximum_future_sec_{0.05};
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  v2x_msgs::msg::V2XVehiclePositionArray::SharedPtr latest_v2x_;
  rclcpp::Publisher<multi_purpose_mpc_ros_msgs::msg::StateLatticeControlCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<v2x_msgs::msg::V2XVehiclePositionArray>::SharedPtr v2x_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleStateLatticePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
