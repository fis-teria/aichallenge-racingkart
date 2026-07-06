#ifndef SIMPLE_PURE_PURSUIT_HPP_
#define SIMPLE_PURE_PURSUIT_HPP_

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
  Odometry::SharedPtr odometry_;

  // pure pursuit parameters
  const double wheel_base_;
  const double lookahead_gain_;
  const double lookahead_min_distance_;
  const double speed_proportional_gain_;
  const bool use_external_target_vel_;
  const double external_target_vel_;
  const double steering_tire_angle_gain_;
  const double debug_publish_period_sec_;
  const bool use_overtake_reference_override_;
  const double overtake_override_timeout_sec_;
  double last_debug_publish_sec_{-1.0e9};
  bool overtake_override_active_{false};
  int overtake_mode_id_{0};
  double last_overtake_override_sec_{-1.0e9};
  std::vector<double> overtake_lateral_offsets_;
  std::vector<double> overtake_speed_caps_;

private:
  void onTimer();
  bool subscribeMessageAvailable();
  void onOvertakeOverride(const Float32MultiArray::SharedPtr msg);
  void clearOvertakeOverride();
  bool applyOvertakeOverride(Trajectory &trajectory,
                             std::size_t nearest_traj_point_idx,
                             double now_sec);
  bool overtakeOverrideFresh(double now_sec) const;
  std::optional<double> overtakeSpeedCap(std::size_t horizon_index,
                                         double now_sec) const;
  double overtakeLateralOffset(std::size_t horizon_index) const;
  void publishDebug(const rclcpp::Time &stamp,
                    const Trajectory &control_trajectory,
                    std::size_t nearest_traj_point_idx,
                    double target_longitudinal_vel,
                    double current_longitudinal_vel, double command_accel,
                    double lookahead_distance, double lookahead_point_x,
                    double lookahead_point_y, double rear_x, double rear_y,
                    double alpha, double raw_steering_tire_angle,
                    double steering_tire_angle, bool overtake_override_applied,
                    double overtake_lateral_offset_m,
                    double overtake_speed_cap_mps);
};

} // namespace simple_pure_pursuit

#endif // SIMPLE_PURE_PURSUIT_HPP_
