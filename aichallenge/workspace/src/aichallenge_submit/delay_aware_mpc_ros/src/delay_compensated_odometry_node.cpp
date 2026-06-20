#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

namespace delay_aware_mpc_ros {
namespace {

double normalize_angle(const double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

std::string json_escape(const std::string &value) {
  std::ostringstream out;
  for (const char c : value) {
    if (c == '"' || c == '\\') {
      out << '\\' << c;
    } else if (c == '\n') {
      out << "\\n";
    } else {
      out << c;
    }
  }
  return out.str();
}

} // namespace

struct PoseState {
  double x{};
  double y{};
  double yaw{};
  double velocity{};
  double yaw_rate{};
};

struct CommandSample {
  double time_sec{};
  double steering_rad{};
};

struct DelayPrediction {
  PoseState input;
  PoseState predicted;
  double estimated_current_steering_rad{};
  double applied_steering_rad{};
  double delay_sec{};
  int prediction_steps{};
  std::string mode;
  bool shifted{};
  std::string steering_source;
};

class DelayCompensatedOdometryNode : public rclcpp::Node {
public:
  DelayCompensatedOdometryNode() : Node("delay_compensated_odometry") {
    enabled_ = declare_parameter<bool>("enabled", true);
    mode_ =
        declare_parameter<std::string>("mode", "state_shift_with_steer_lag");
    steering_delay_sec_ = declare_parameter<double>("steering_delay_sec", 0.20);
    prediction_dt_ = declare_parameter<double>("prediction_dt", 0.02);
    steering_time_constant_sec_ =
        declare_parameter<double>("steering_time_constant_sec", 0.30);
    wheelbase_ = declare_parameter<double>("wheelbase", 1.087);
    use_reference_time_shift_ =
        declare_parameter<bool>("use_reference_time_shift", true);
    use_steering_status_ = declare_parameter<bool>("use_steering_status", true);
    steering_status_timeout_sec_ =
        declare_parameter<double>("steering_status_timeout_sec", 0.20);
    use_yaw_rate_fallback_ =
        declare_parameter<bool>("use_yaw_rate_fallback", true);
    use_command_history_fallback_ =
        declare_parameter<bool>("use_command_history_fallback", true);
    min_velocity_for_yaw_prediction_ =
        declare_parameter<double>("min_velocity_for_yaw_prediction", 0.20);
    debug_publish_period_sec_ =
        declare_parameter<double>("debug_publish_period_sec", 0.25);

    normalize_config();

    delayed_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "/delay_aware_mpc/localization/kinematic_state", rclcpp::QoS(1));
    delayed_pose_pub_ = create_publisher<geometry_msgs::msg::Pose2D>(
        "/delay_aware_mpc/delayed_pose", rclcpp::QoS(1));
    debug_pub_ = create_publisher<std_msgs::msg::String>(
        "/delay_aware_mpc/debug", rclcpp::QoS(1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/localization/kinematic_state", rclcpp::QoS(1),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          on_odometry(msg);
        });
    command_sub_ = create_subscription<
        autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd_raw", rclcpp::QoS(1),
        [this](const autoware_auto_control_msgs::msg::AckermannControlCommand::
                   ConstSharedPtr msg) { on_control_command(msg); });
    steering_sub_ =
        create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
            "/vehicle/status/steering_status", rclcpp::QoS(1),
            [this](const autoware_auto_vehicle_msgs::msg::SteeringReport::
                       ConstSharedPtr msg) { on_steering_report(msg); });

    RCLCPP_INFO(
        get_logger(),
        "delay-aware odometry mode=%s delay=%.3fs dt=%.3fs wheelbase=%.3fm",
        mode_.c_str(), steering_delay_sec_, prediction_dt_, wheelbase_);
  }

private:
  bool active() const {
    return enabled_ && mode_ != "baseline" && use_reference_time_shift_ &&
           steering_delay_sec_ > 0.0;
  }

  void normalize_config() {
    if (mode_ != "baseline" && mode_ != "state_shift" &&
        mode_ != "state_shift_with_steer_lag" && mode_ != "delay_augmented") {
      RCLCPP_WARN(
          get_logger(),
          "Unsupported mode '%s'; falling back to state_shift_with_steer_lag",
          mode_.c_str());
      mode_ = "state_shift_with_steer_lag";
    }
    steering_delay_sec_ = std::max(0.0, steering_delay_sec_);
    prediction_dt_ = std::max(1.0e-3, prediction_dt_);
    steering_time_constant_sec_ = std::max(1.0e-3, steering_time_constant_sec_);
    wheelbase_ = std::max(1.0e-3, wheelbase_);
    steering_status_timeout_sec_ = std::max(0.0, steering_status_timeout_sec_);
    min_velocity_for_yaw_prediction_ =
        std::max(1.0e-3, min_velocity_for_yaw_prediction_);
    debug_publish_period_sec_ = std::max(0.0, debug_publish_period_sec_);
  }

  double now_sec() {
    return static_cast<double>(get_clock()->now().nanoseconds()) / 1.0e9;
  }

  void on_control_command(const autoware_auto_control_msgs::msg::
                              AckermannControlCommand::ConstSharedPtr msg) {
    const double t = now_sec();
    const double steer = static_cast<double>(msg->lateral.steering_tire_angle);
    command_history_.push_back(CommandSample{t, steer});
    last_command_steering_rad_ = steer;
    trim_command_history(t);
  }

  void on_steering_report(
      const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr
          msg) {
    latest_steering_status_ =
        CommandSample{now_sec(), static_cast<double>(msg->steering_tire_angle)};
  }

  void on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    const double t = now_sec();
    const PoseState input{msg->pose.pose.position.x, msg->pose.pose.position.y,
                          yaw_from_quaternion(msg->pose.pose.orientation),
                          msg->twist.twist.linear.x,
                          msg->twist.twist.angular.z};
    const DelayPrediction prediction = predict(input, t);

    auto output = *msg;
    if (prediction.shifted) {
      output.pose.pose.position.x = prediction.predicted.x;
      output.pose.pose.position.y = prediction.predicted.y;
      output.pose.pose.orientation =
          quaternion_from_yaw(prediction.predicted.yaw);
    }
    delayed_odom_pub_->publish(output);
    publish_debug(t, prediction);
  }

  DelayPrediction predict(const PoseState &state, const double t) {
    const auto [current_steer, source] = estimate_current_steering(state, t);
    if (!active()) {
      return DelayPrediction{
          state, state, current_steer, current_steer, steering_delay_sec_,
          0,     mode_, false,         source};
    }

    PoseState predicted = state;
    double elapsed = 0.0;
    int steps = 0;
    double applied_steer = current_steer;
    double lagged_steer = current_steer;

    while (elapsed < steering_delay_sec_ - 1.0e-12) {
      const double step_dt =
          std::min(prediction_dt_, steering_delay_sec_ - elapsed);
      const double future_time = t + elapsed;
      applied_steer = predict_steering_for_step(future_time, step_dt,
                                                current_steer, lagged_steer);
      if (mode_ == "state_shift_with_steer_lag") {
        lagged_steer = applied_steer;
      }

      predicted.x += predicted.velocity * std::cos(predicted.yaw) * step_dt;
      predicted.y += predicted.velocity * std::sin(predicted.yaw) * step_dt;
      predicted.yaw = normalize_angle(predicted.yaw +
                                      predicted.velocity / wheelbase_ *
                                          std::tan(applied_steer) * step_dt);
      elapsed += step_dt;
      ++steps;
    }

    last_prediction_steering_rad_ = applied_steer;
    return DelayPrediction{
        state, predicted, current_steer, applied_steer, steering_delay_sec_,
        steps, mode_,     true,          source};
  }

  std::pair<double, std::string>
  estimate_current_steering(const PoseState &state, const double t) const {
    if (use_steering_status_ && latest_steering_status_.has_value()) {
      const double age = t - latest_steering_status_->time_sec;
      if (age >= 0.0 && age <= steering_status_timeout_sec_) {
        return {latest_steering_status_->steering_rad, "steering_status"};
      }
    }

    if (use_yaw_rate_fallback_ &&
        std::abs(state.velocity) >= min_velocity_for_yaw_prediction_) {
      return {std::atan2(wheelbase_ * state.yaw_rate, state.velocity),
              "yaw_rate"};
    }

    if (use_command_history_fallback_ && !command_history_.empty()) {
      return {command_at_or_before(t - steering_delay_sec_), "command_history"};
    }

    return {last_command_steering_rad_, "last_command"};
  }

  double predict_steering_for_step(const double future_time,
                                   const double step_dt,
                                   const double current_steer,
                                   const double lagged_steer) const {
    if (mode_ == "state_shift") {
      return current_steer;
    }

    const double target_steer =
        command_history_.empty()
            ? current_steer
            : command_at_or_before(future_time - steering_delay_sec_);
    if (mode_ == "delay_augmented") {
      return target_steer;
    }

    const double alpha = 1.0 - std::exp(-step_dt / steering_time_constant_sec_);
    return lagged_steer + (target_steer - lagged_steer) * alpha;
  }

  double command_at_or_before(const double target_time_sec) const {
    const CommandSample *selected = nullptr;
    for (const auto &sample : command_history_) {
      if (sample.time_sec <= target_time_sec) {
        selected = &sample;
      } else {
        break;
      }
    }
    if (selected != nullptr) {
      return selected->steering_rad;
    }
    if (!command_history_.empty()) {
      return command_history_.front().steering_rad;
    }
    return last_command_steering_rad_;
  }

  void trim_command_history(const double t) {
    const double keep_window_sec = std::max(5.0, steering_delay_sec_ * 4.0);
    while (!command_history_.empty() &&
           t - command_history_.front().time_sec > keep_window_sec) {
      command_history_.pop_front();
    }
  }

  void publish_debug(const double t, const DelayPrediction &prediction) {
    if (debug_publish_period_sec_ > 0.0 &&
        t - last_debug_publish_sec_ < debug_publish_period_sec_) {
      return;
    }
    last_debug_publish_sec_ = t;

    geometry_msgs::msg::Pose2D pose;
    pose.x = prediction.predicted.x;
    pose.y = prediction.predicted.y;
    pose.theta = prediction.predicted.yaw;
    delayed_pose_pub_->publish(pose);

    std_msgs::msg::String debug;
    std::ostringstream json;
    json << "{"
         << "\"mode\":\"" << json_escape(prediction.mode) << "\","
         << "\"shifted\":" << (prediction.shifted ? "true" : "false") << ","
         << "\"delay_sec\":" << prediction.delay_sec << ","
         << "\"prediction_steps\":" << prediction.prediction_steps << ","
         << "\"steering_source\":\"" << json_escape(prediction.steering_source)
         << "\","
         << "\"estimated_current_steering_rad\":"
         << prediction.estimated_current_steering_rad << ","
         << "\"applied_steering_rad\":" << prediction.applied_steering_rad
         << ","
         << "\"last_prediction_steering_rad\":" << last_prediction_steering_rad_
         << ","
         << "\"input_pose\":{"
         << "\"x\":" << prediction.input.x << ","
         << "\"y\":" << prediction.input.y << ","
         << "\"yaw\":" << prediction.input.yaw << ","
         << "\"velocity\":" << prediction.input.velocity << ","
         << "\"yaw_rate\":" << prediction.input.yaw_rate << "},"
         << "\"delayed_pose\":{"
         << "\"x\":" << prediction.predicted.x << ","
         << "\"y\":" << prediction.predicted.y << ","
         << "\"yaw\":" << prediction.predicted.yaw << "}"
         << "}";
    debug.data = json.str();
    debug_pub_->publish(debug);
  }

  bool enabled_{};
  std::string mode_;
  double steering_delay_sec_{};
  double prediction_dt_{};
  double steering_time_constant_sec_{};
  double wheelbase_{};
  bool use_reference_time_shift_{};
  bool use_steering_status_{};
  double steering_status_timeout_sec_{};
  bool use_yaw_rate_fallback_{};
  bool use_command_history_fallback_{};
  double min_velocity_for_yaw_prediction_{};
  double debug_publish_period_sec_{};

  double last_command_steering_rad_{};
  double last_prediction_steering_rad_{};
  double last_debug_publish_sec_{-1.0e9};
  std::optional<CommandSample> latest_steering_status_;
  std::deque<CommandSample> command_history_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr delayed_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr delayed_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<
      autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
      command_sub_;
  rclcpp::Subscription<
      autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_sub_;
};

} // namespace delay_aware_mpc_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<delay_aware_mpc_ros::DelayCompensatedOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
