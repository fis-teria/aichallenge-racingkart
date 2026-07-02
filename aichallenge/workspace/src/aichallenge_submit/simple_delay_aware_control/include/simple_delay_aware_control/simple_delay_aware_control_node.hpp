#ifndef SIMPLE_DELAY_AWARE_CONTROL__SIMPLE_DELAY_AWARE_CONTROL_NODE_HPP_
#define SIMPLE_DELAY_AWARE_CONTROL__SIMPLE_DELAY_AWARE_CONTROL_NODE_HPP_

#include "simple_delay_aware_control/control_core.hpp"

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <optional>
#include <string>
#include <vector>

namespace simple_delay_aware_control
{

class SimpleDelayAwareControlNode : public rclcpp::Node
{
public:
  SimpleDelayAwareControlNode();

private:
  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using SteeringReport = autoware_auto_vehicle_msgs::msg::SteeringReport;
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using Odometry = nav_msgs::msg::Odometry;
  using PointStamped = geometry_msgs::msg::PointStamped;

  void onTimer();
  void onTrajectory(const Trajectory::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onSteeringReport(const SteeringReport::ConstSharedPtr msg);
  void onControlCommand(const AckermannControlCommand::ConstSharedPtr msg);

  bool ready();
  VehicleState makeVehicleState(const Odometry & odometry) const;
  double estimateCurrentSteering(const VehicleState & state, double now_sec) const;
  double targetSteeringForPrediction(double now_sec) const;
  void publishDelayAwareOdometry(
    const Odometry & source_odometry, const DelayPrediction & delay_prediction,
    const std::optional<SpeedPlan> & speed_plan);
  std::size_t findLookaheadIndex(
    const VehicleState & control_state, std::size_t nearest_index, double lookahead_distance) const;
  void publishLookaheadPoint(const TrajectorySample & sample);
  void publishDebug(
    double now_sec, const DelayPrediction & delay_prediction, const SpeedPlan & speed_plan,
    double target_steering_rad, double raw_steering_rad, double command_steering_rad);

  double nowSec();

  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_control_cmd_;

  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_delay_aware_odometry_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_lookahead_point_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_;

  rclcpp::TimerBase::SharedPtr timer_;

  Odometry::ConstSharedPtr odometry_;
  std::vector<TrajectorySample> trajectory_;
  std::optional<std::pair<double, double>> latest_steering_status_;
  std::optional<std::pair<double, double>> latest_control_cmd_steering_;

  DelayControlParams delay_params_;
  CurvatureSpeedParams speed_params_;

  double wheel_base_{1.087};
  double lookahead_gain_{0.45};
  double lookahead_min_distance_{3.0};
  double speed_proportional_gain_{1.0};
  double max_acceleration_mps2_{2.0};
  double max_deceleration_mps2_{4.0};
  double steering_tire_angle_gain_{1.5};
  double max_steering_tire_angle_rad_{0.70};
  double steering_status_timeout_sec_{0.20};
  double control_cmd_timeout_sec_{0.50};
  double min_velocity_for_yaw_prediction_{0.20};
  double control_period_ms_{10.0};
  double debug_publish_period_sec_{0.25};
  bool use_steering_status_{true};
  bool use_control_cmd_input_{true};
  bool use_yaw_rate_fallback_{true};
  bool trajectory_is_closed_{true};
  bool publish_delay_aware_odometry_{true};
  bool write_target_speed_to_output_twist_{false};

  double last_command_steering_rad_{0.0};
  double last_debug_publish_sec_{-1.0e9};
};

}  // namespace simple_delay_aware_control

#endif  // SIMPLE_DELAY_AWARE_CONTROL__SIMPLE_DELAY_AWARE_CONTROL_NODE_HPP_
