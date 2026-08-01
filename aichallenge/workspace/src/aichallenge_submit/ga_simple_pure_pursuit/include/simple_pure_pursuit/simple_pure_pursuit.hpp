#ifndef SIMPLE_PURE_PURSUIT_HPP_
#define SIMPLE_PURE_PURSUIT_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace simple_pure_pursuit
{

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::PointStamped;
using nav_msgs::msg::Odometry;

class SimplePurePursuit : public rclcpp::Node
{
public:
  SimplePurePursuit();

private:
  struct TuningParameters
  {
    double lookahead_gain{0.5};
    double lookahead_min_distance{3.5};
    double actual_lookahead_distance_blend{0.0};
    double dual_preview_near_ratio{0.5};
    double dual_preview_blend{0.0};
    double steering_tire_angle_gain{1.5};
    double curvature_lookahead_min_distance{2.0};
    double curvature_lookahead_sensitivity{8.0};
    double curvature_lookahead_smoothing_alpha{0.35};
    double max_lateral_acceleration{6.0};
    double minimum_corner_speed{5.0};
    double corner_speed_retention{0.0};
    bool lateral_error_speed_gate_enabled{true};
    double corner_speed_retention_lateral_error_soft{0.5};
    double corner_speed_retention_lateral_error_hard{1.0};
    double curvature_speed_preview_distance{12.0};
    double speed_proportional_gain{1.0};
    double longitudinal_acceleration_limit{1.0};
    double external_target_vel{9.722222222222};
  };

  void onTimer();
  bool subscribeMessageAvailable() const;
  void resetExperimentState();
  double estimateCurvature(std::size_t nearest_index) const;
  double estimatePreviewCurvature(
    std::size_t nearest_index, double preview_distance) const;
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);
  void onSetEnabled(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  void onResetState(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  const double wheel_base_;
  const bool use_external_target_vel_;
  const bool ga_experiment_mode_;

  mutable std::mutex state_mutex_;
  TuningParameters tuning_;
  bool controller_enabled_;
  std::string ga_run_id_;
  std::string ga_candidate_id_;
  std::string ga_parameter_hash_;
  std::uint64_t reset_epoch_{0};
  double smoothed_curvature_{0.0};
  bool curvature_initialized_{false};
  double previous_steering_{0.0};

  Trajectory::SharedPtr trajectory_;
  Odometry::SharedPtr odometry_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_lookahead_point_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_enabled_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_state_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUIT_HPP_
