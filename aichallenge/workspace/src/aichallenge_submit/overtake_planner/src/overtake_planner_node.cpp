#include "overtake_planner/overtake_planner_core.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <v2x_msgs/msg/v2_x_vehicle_position_array.hpp>

#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace overtake_planner
{

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string resolveReferencePath(
  const std::string & package_name,
  const std::string & csv_path)
{
  if (csv_path.empty() || csv_path.front() == '/') {
    return csv_path;
  }
  return ament_index_cpp::get_package_share_directory(package_name) + "/" + csv_path;
}

}  // namespace

class OvertakePlannerNode : public rclcpp::Node
{
public:
  OvertakePlannerNode()
  : Node("overtake_planner_node")
  {
    const auto reference_package =
      declare_parameter<std::string>("reference_package", "multi_purpose_mpc_ros");
    const auto reference_csv =
      declare_parameter<std::string>("reference_csv", "env/final_ver3/traj_mincurv_manual.csv");
    own_vehicle_id_ = declare_parameter<std::string>("own_vehicle_id", "d1");
    ignore_near_ego_m_ = declare_parameter<double>("ignore_near_ego_m", 1.0);
    position_jump_threshold_m_ = declare_parameter<double>("position_jump_threshold_m", 5.0);
    ego_stale_time_sec_ = declare_parameter<double>("ego_stale_time_sec", 0.50);

    PlannerConfig config;
    config.enabled = declare_parameter<bool>("enabled", true);
    config.horizon_points = static_cast<std::size_t>(declare_parameter<int>("horizon_points", 30));
    config.horizon_dt_sec = declare_parameter<double>("horizon_dt_sec", 0.025);
    config.lookahead_s_m = declare_parameter<double>("lookahead_s_m", 35.0);
    config.follow_trigger_s_m = declare_parameter<double>("follow_trigger_s_m", 12.0);
    config.same_corridor_width_m = declare_parameter<double>("same_corridor_width_m", 0.90);
    config.dv_block_threshold_mps = declare_parameter<double>("dv_block_threshold_mps", 0.20);
    config.opponent_stale_time_sec = declare_parameter<double>("opponent_stale_time_sec", 0.50);
    config.side_by_side_s_m = declare_parameter<double>("side_by_side_s_m", 4.0);
    config.left_offset_m = declare_parameter<double>("left_offset_m", 0.80);
    config.right_offset_m = declare_parameter<double>("right_offset_m", -0.80);
    config.prepare_distance_m = declare_parameter<double>("prepare_distance_m", 8.0);
    config.merge_distance_m = declare_parameter<double>("merge_distance_m", 12.0);
    config.follow_speed_margin_mps = declare_parameter<double>("follow_speed_margin_mps", 0.20);
    config.recovery_v_max_mps = declare_parameter<double>("recovery_v_max_mps", 5.0);
    config.v_passthrough_mps = declare_parameter<double>("v_passthrough_mps", 50.0);
    config.d_min_m = declare_parameter<double>("d_min_m", -1.35);
    config.d_max_m = declare_parameter<double>("d_max_m", 1.35);
    config.min_wall_margin_m = declare_parameter<double>("min_wall_margin_m", 0.25);
    config.safety_ellipse_a_m = declare_parameter<double>("safety_ellipse_a_m", 3.0);
    config.safety_ellipse_b_m = declare_parameter<double>("safety_ellipse_b_m", 1.2);
    config.min_ellipse_h = declare_parameter<double>("min_ellipse_h", 0.20);
    config.pass_safe_required_cycles =
      declare_parameter<double>("pass_safe_required_cycles", 5.0);
    config.merge_front_gap_m = declare_parameter<double>("merge_front_gap_m", 6.0);
    config.abort_timeout_sec = declare_parameter<double>("abort_timeout_sec", 5.0);
    config.min_mode_hold_time_sec = declare_parameter<double>("min_mode_hold_time_sec", 0.60);
    config.keep_mode_bonus = declare_parameter<double>("keep_mode_bonus", 25.0);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);

    FrenetFrame frame;
    std::string error;
    const auto reference_path = resolveReferencePath(reference_package, reference_csv);
    if (!frame.loadCsv(reference_path, &error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(), "loaded overtake reference: %s (%zu points, %.2f m)",
        reference_path.c_str(), frame.reference().size(), frame.length());
    }
    frame_ = frame;
    core_ = std::make_unique<OvertakePlannerCore>(frame_, config);

    override_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/overtake/reference_override", rclcpp::QoS(1));
    mode_pub_ = create_publisher<std_msgs::msg::String>("/debug/overtake/mode", rclcpp::QoS(1));
    metrics_pub_ = create_publisher<std_msgs::msg::String>(
      "/debug/overtake/metrics", rclcpp::QoS(1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", rclcpp::QoS(1),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odom_ = *msg;
      });
    v2x_sub_ = create_subscription<v2x_msgs::msg::V2XVehiclePositionArray>(
      "/v2x/vehicle_positions", rclcpp::QoS(1),
      [this](const v2x_msgs::msg::V2XVehiclePositionArray::ConstSharedPtr msg) {
        updateOpponents(*msg);
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_)),
      [this]() { onTimer(); });
  }

private:
  struct TrackSample
  {
    double stamp_sec{0.0};
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
    bool has_prev{false};
  };

  double stampToSec(const builtin_interfaces::msg::Time & stamp) const
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
  }

  void updateOpponents(const v2x_msgs::msg::V2XVehiclePositionArray & msg)
  {
    for (const auto & vehicle : msg.vehicles) {
      const double stamp_sec = stampToSec(vehicle.header.stamp);
      auto & sample = samples_[vehicle.vehicle_id];
      const double x = vehicle.position.x;
      const double y = vehicle.position.y;
      if (sample.has_prev) {
        const double dt = stamp_sec - sample.stamp_sec;
        const double jump = std::hypot(x - sample.x, y - sample.y);
        if (dt > 0.0 && jump <= position_jump_threshold_m_) {
          sample.vx = (x - sample.x) / dt;
          sample.vy = (y - sample.y) / dt;
        } else {
          sample.vx = 0.0;
          sample.vy = 0.0;
        }
      }
      sample.stamp_sec = stamp_sec;
      sample.x = x;
      sample.y = y;
      sample.has_prev = true;
    }
  }

  std::vector<OpponentState> collectOpponents(const EgoState & ego, double now_sec) const
  {
    std::vector<OpponentState> opponents;
    if (!ego.valid || frame_.empty()) {
      return opponents;
    }
    for (const auto & item : samples_) {
      const auto & id = item.first;
      const auto & sample = item.second;
      if (!sample.has_prev || id == own_vehicle_id_) {
        continue;
      }
      const double ego_dist = std::hypot(sample.x - ego.x, sample.y - ego.y);
      if (ego_dist < ignore_near_ego_m_) {
        continue;
      }
      OpponentState opp;
      opp.id = id;
      opp.stamp_sec = sample.stamp_sec;
      opp.x = sample.x;
      opp.y = sample.y;
      opp.vx = sample.vx;
      opp.vy = sample.vy;
      opp.v = std::hypot(sample.vx, sample.vy);
      opp.frenet = frame_.cartesianToFrenet(opp.x, opp.y, 0.0);
      opp.valid = now_sec - opp.stamp_sec <= 2.0;
      opponents.push_back(opp);
    }
    return opponents;
  }

  void publishOverride(const PlannerOutput & output)
  {
    std_msgs::msg::Float32MultiArray msg;
    const int mode_id = static_cast<int>(output.mode);
    const int n = output.active_override ? static_cast<int>(output.lateral_offsets.size()) : 0;
    msg.data.push_back(1.0F);
    msg.data.push_back(static_cast<float>(mode_id));
    msg.data.push_back(static_cast<float>(n));
    for (int i = 0; i < n; ++i) {
      msg.data.push_back(static_cast<float>(output.lateral_offsets[static_cast<std::size_t>(i)]));
    }
    for (int i = 0; i < n; ++i) {
      msg.data.push_back(static_cast<float>(output.speed_caps[static_cast<std::size_t>(i)]));
    }
    override_pub_->publish(msg);
  }

  void publishDebug(const PlannerOutput & output)
  {
    std_msgs::msg::String mode_msg;
    mode_msg.data = toString(output.mode);
    mode_pub_->publish(mode_msg);

    std_msgs::msg::String metrics_msg;
    std::ostringstream oss;
    oss << "{"
        << "\"mode\":\"" << toString(output.mode) << "\","
        << "\"selected\":\"" << toString(output.selected) << "\","
        << "\"blocked\":" << (output.blocked_info.blocked ? "true" : "false") << ","
        << "\"side_by_side\":" << (output.blocked_info.side_by_side ? "true" : "false") << ","
        << "\"front_delta_s\":" << output.blocked_info.front_delta_s << ","
        << "\"front_delta_d\":" << output.blocked_info.front_delta_d << ","
        << "\"front_rel_v\":" << output.blocked_info.front_rel_v << ","
        << "\"active_override\":" << (output.active_override ? "true" : "false")
        << "}";
    metrics_msg.data = oss.str();
    metrics_pub_->publish(metrics_msg);
  }

  void onTimer()
  {
    const double now_sec = now().seconds();
    EgoState ego;
    if (odom_.has_value() && !frame_.empty()) {
      const auto & odom = *odom_;
      ego.stamp_sec = stampToSec(odom.header.stamp);
      if (now_sec - ego.stamp_sec <= ego_stale_time_sec_) {
        ego.x = odom.pose.pose.position.x;
        ego.y = odom.pose.pose.position.y;
        ego.yaw = yawFromQuaternion(odom.pose.pose.orientation);
        ego.v = odom.twist.twist.linear.x;
        ego.frenet = frame_.cartesianToFrenet(ego.x, ego.y, ego.yaw);
        ego.valid = true;
      }
    }

    const auto opponents = collectOpponents(ego, now_sec);
    const auto output = core_->update(now_sec, ego, opponents);
    publishOverride(output);
    publishDebug(output);
  }

  FrenetFrame frame_;
  std::unique_ptr<OvertakePlannerCore> core_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::unordered_map<std::string, TrackSample> samples_;
  std::string own_vehicle_id_;
  double ignore_near_ego_m_{1.0};
  double position_jump_threshold_m_{5.0};
  double ego_stale_time_sec_{0.50};
  double control_rate_hz_{20.0};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr override_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<v2x_msgs::msg::V2XVehiclePositionArray>::SharedPtr v2x_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace overtake_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<overtake_planner::OvertakePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
