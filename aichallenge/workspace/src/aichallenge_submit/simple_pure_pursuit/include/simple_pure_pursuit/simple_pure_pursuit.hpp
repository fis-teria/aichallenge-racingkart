#ifndef SIMPLE_PURE_PURSUIT_HPP_
#define SIMPLE_PURE_PURSUIT_HPP_

#include "simple_pure_pursuit/lookahead.hpp"
#include "simple_pure_pursuit/safety.hpp"

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <cstddef>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

namespace simple_pure_pursuit {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Float32MultiArray;
using std_msgs::msg::String;

class SimplePurePursuit : public rclcpp::Node {
public:
  explicit SimplePurePursuit();

  // subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_mpc_predicted_horizon_;
  rclcpp::Subscription<Float32MultiArray>::SharedPtr sub_overtake_override_;

  // publishers
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_lookahead_point_;
  rclcpp::Publisher<String>::SharedPtr pub_debug_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // updated by subscribers
  Trajectory::SharedPtr trajectory_;
  Trajectory::SharedPtr mpc_predicted_horizon_;
  Odometry::SharedPtr odometry_;
  std::optional<double> last_odometry_receive_sec_;
  std::optional<double> last_trajectory_receive_sec_;
  std::optional<double> last_mpc_predicted_horizon_receive_sec_;

  // pure pursuit parameters
  const double wheel_base_;
  const double lookahead_gain_;
  const double lookahead_min_distance_;
  const double speed_proportional_gain_;
  const bool use_external_target_vel_;
  const double external_target_vel_;
  const double steering_tire_angle_gain_;
  const double debug_publish_period_sec_;
  const double max_odom_age_sec_;
  const double max_trajectory_age_sec_;
  const double max_override_age_sec_;
  const bool stop_on_stale_input_;
  const double diagnostic_throttle_sec_;
  const bool use_mpc_predicted_horizon_;
  const double max_mpc_horizon_age_sec_;
  const int min_mpc_horizon_points_;
  const double max_mpc_horizon_start_distance_m_;
  const double min_mpc_horizon_arc_length_m_;
  const bool use_overtake_reference_override_;
  const double overtake_override_timeout_sec_;
  const bool curvature_adaptive_lookahead_enabled_;
  const double curvature_lookahead_min_distance_;
  const double curvature_lookahead_sensitivity_;
  const double curvature_lookahead_window_ratio_;
  const double curvature_lookahead_max_window_distance_;
  const double curvature_lookahead_min_arc_length_;
  const double curvature_lookahead_smoothing_alpha_;
  double last_debug_publish_sec_{-1.0e9};
  bool has_smoothed_lookahead_distance_{false};
  double smoothed_lookahead_distance_{0.0};
  bool overtake_override_active_{false};
  int overtake_mode_id_{0};
  double last_overtake_override_sec_{-1.0e9};
  std::vector<double> overtake_lateral_offsets_;
  std::vector<double> overtake_speed_caps_;

private:
  void onTimer();
  double steadyNowSec() const;
  FreshnessResult evaluateInputFreshness(double now_sec) const;
  HorizonFreshnessResult evaluateMpcPredictedHorizon(double now_sec) const;
  void publishStopForStaleInput(const rclcpp::Time &stamp,
                                const FreshnessResult &freshness);
  void publishStaleDebug(const rclcpp::Time &stamp,
                         const FreshnessResult &freshness);
  void onOvertakeOverride(const Float32MultiArray::SharedPtr msg);
  void clearOvertakeOverride();
  bool applyOvertakeOverride(Trajectory &trajectory,
                             std::size_t nearest_traj_point_idx,
                             double now_sec);
  bool overtakeOverrideFresh(double now_sec) const;
  std::optional<double> overtakeSpeedCap(std::size_t horizon_index,
                                         double now_sec) const;
  double overtakeLateralOffset(std::size_t horizon_index) const;
  void
  publishDebug(const rclcpp::Time &stamp, const Trajectory &control_trajectory,
               std::size_t nearest_traj_point_idx,
               double target_longitudinal_vel, double current_longitudinal_vel,
               double command_accel, double base_lookahead_distance,
               double desired_lookahead_distance, double lookahead_distance,
               double path_curvature, double curvature_window_distance,
               double lookahead_point_x, double lookahead_point_y,
               double rear_x, double rear_y, double alpha,
               double raw_steering_tire_angle, double steering_tire_angle,
               bool overtake_override_applied, double overtake_lateral_offset_m,
               double overtake_speed_cap_mps, double freshness_now_sec,
               bool mpc_horizon_applied,
               bool mpc_horizon_velocity_cap_applied,
               double mpc_horizon_velocity_cap_mps,
               const HorizonFreshnessResult &mpc_horizon_freshness,
               const std::string &trajectory_source);
};

} // namespace simple_pure_pursuit

#endif // SIMPLE_PURE_PURSUIT_HPP_
