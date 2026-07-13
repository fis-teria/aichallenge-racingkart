#ifndef SIMPLE_PURE_PURSUIT_HPP_
#define SIMPLE_PURE_PURSUIT_HPP_

#include "simple_pure_pursuit/lookahead.hpp"
#include "simple_pure_pursuit/safety.hpp"

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <utility>
#include <vector>

namespace simple_pure_pursuit {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
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
  rclcpp::Subscription<String>::SharedPtr sub_mpc_predicted_horizon_contract_;
  rclcpp::Subscription<Float32MultiArray>::SharedPtr sub_overtake_override_;
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_status_;
  rclcpp::Subscription<String>::SharedPtr sub_mpc_health_;

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
  std::optional<double> last_mpc_predicted_horizon_contract_receive_sec_;
  std::optional<double> last_steering_status_receive_sec_;
  std::optional<double> last_mpc_health_receive_sec_;
  double latest_steering_status_rad_{0.0};
  std::string mpc_health_status_{"missing"};
  int mpc_health_infeasible_count_{0};
  std::string mpc_predicted_horizon_source_{"unknown"};
  bool mpc_horizon_contract_received_{false};
  std::int32_t mpc_horizon_contract_stamp_sec_{0};
  std::uint32_t mpc_horizon_contract_stamp_nanosec_{0};
  std::string mpc_horizon_contract_source_{"unknown"};
  int mpc_horizon_contract_mode_id_{0};
  std::uint32_t mpc_horizon_contract_generation_{0};

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
  const bool require_solved_mpc_health_for_horizon_;
  const double max_mpc_health_age_sec_;
  const bool require_matching_overtake_horizon_contract_;
  const bool use_overtake_reference_override_;
  const double overtake_override_timeout_sec_;
  const bool curvature_adaptive_lookahead_enabled_;
  const double curvature_lookahead_min_distance_;
  const double curvature_lookahead_sensitivity_;
  const double curvature_lookahead_window_ratio_;
  const double curvature_lookahead_max_window_distance_;
  const double curvature_lookahead_min_arc_length_;
  const double curvature_lookahead_smoothing_alpha_;
  const double pp_control_delay_sec_;
  const double pp_prediction_dt_sec_;
  const double steering_time_constant_sec_;
  const double steering_status_timeout_sec_;
  const double min_velocity_for_delay_compensation_mps_;
  const double horizon_curvature_feedforward_gain_;
  const double horizon_curvature_feedforward_max_rad_;
  double last_debug_publish_sec_{-1.0e9};
  double last_commanded_steering_tire_angle_{0.0};
  bool has_smoothed_lookahead_distance_{false};
  double smoothed_lookahead_distance_{0.0};
  bool overtake_override_active_{false};
  int overtake_mode_id_{0};
  std::uint32_t overtake_override_generation_{0};
  double last_overtake_override_sec_{-1.0e9};
  std::vector<double> overtake_lateral_offsets_;
  std::vector<double> overtake_speed_caps_;

private:
  struct ControlPosePrediction {
    geometry_msgs::msg::Point position;
    double yaw{0.0};
    double velocity_mps{0.0};
    double current_steering_rad{0.0};
    double applied_steering_rad{0.0};
    double steering_age_sec{-1.0};
    int prediction_steps{0};
    bool shifted{false};
    std::string steering_source{"last_command"};
  };

  // onTimer()が扱う参照trajectoryを1つに正規化した結果。
  // 通常trajectory、MPC predicted horizon、overtake override適用後trajectoryの
  // どれを使ったかを、制御計算とdebug出力へ同じ契約で渡す。
  struct ControlTrajectoryContext {
    bool valid{false};
    std::string invalid_reason{"empty_trajectory"};
    std::shared_ptr<Trajectory> owned_trajectory;
    const Trajectory *trajectory{nullptr};
    std::size_t nearest_index{0};
    bool mpc_horizon_applied{false};
    std::int32_t evaluated_horizon_stamp_sec{0};
    std::uint32_t evaluated_horizon_stamp_nanosec{0};
    std::int32_t applied_horizon_stamp_sec{0};
    std::uint32_t applied_horizon_stamp_nanosec{0};
    std::string applied_horizon_source{"none"};
    int applied_horizon_mode_id{0};
    std::uint32_t applied_horizon_generation{0};
    bool overtake_override_applied{false};
    std::string source{"trajectory"};
    HorizonFreshnessResult mpc_horizon_freshness{};
  };

  // 縦方向制御の中間結果。
  // 速度capの出所を保持して、最終cmdとdebug JSONで同じ値を使う。
  struct LongitudinalCommand {
    double target_speed_mps{0.0};
    double current_speed_mps{0.0};
    double acceleration_mps2{0.0};
    bool mpc_horizon_velocity_cap_applied{false};
    double mpc_horizon_velocity_cap_mps{-1.0};
    double overtake_speed_cap_mps{0.0};
  };

  // 横方向pure pursuit計算の中間結果。
  // lookahead点、曲率、feed-forward量をまとめてdebug出力の引数肥大化を抑える。
  struct LateralCommand {
    double base_lookahead_distance_m{0.0};
    double desired_lookahead_distance_m{0.0};
    double lookahead_distance_m{0.0};
    double path_curvature_1pm{0.0};
    double signed_path_curvature_1pm{0.0};
    double curvature_window_distance_m{0.0};
    double lookahead_point_x{0.0};
    double lookahead_point_y{0.0};
    double rear_x{0.0};
    double rear_y{0.0};
    double alpha_rad{0.0};
    double pure_pursuit_steering_tire_angle_rad{0.0};
    double curvature_feedforward_steering_rad{0.0};
    double raw_steering_tire_angle_rad{0.0};
    double steering_tire_angle_rad{0.0};
  };

  void onTimer();
  double steadyNowSec() const;
  ControlPosePrediction predictControlPose(double now_sec) const;
  std::pair<double, std::string>
  estimateCurrentSteering(double now_sec, double *steering_age_sec) const;
  FreshnessResult evaluateInputFreshness(double now_sec) const;
  HorizonFreshnessResult evaluateMpcPredictedHorizon(double now_sec) const;
  bool handleInvalidFreshness(const rclcpp::Time &stamp,
                              const FreshnessResult &freshness);
  void clearStaleOvertakeOverride(double now_sec);
  ControlTrajectoryContext
  selectControlTrajectory(const ControlPosePrediction &control_pose,
                          double now_sec,
                          const HorizonFreshnessResult &mpc_horizon_freshness);
  LongitudinalCommand
  computeLongitudinalCommand(const ControlTrajectoryContext &context,
                             double now_sec) const;
  LateralCommand
  computeLateralCommand(const ControlTrajectoryContext &context,
                        const ControlPosePrediction &control_pose,
                        const LongitudinalCommand &longitudinal);
  void publishLookaheadPoint(double x, double y, double z);
  void publishStopForStaleInput(const rclcpp::Time &stamp,
                                const FreshnessResult &freshness);
  void publishStaleDebug(const rclcpp::Time &stamp,
                         const FreshnessResult &freshness);
  void onOvertakeOverride(const Float32MultiArray::SharedPtr msg);
  void onMpcPredictedHorizonContract(const String::SharedPtr msg);
  void onMpcHealth(const String::SharedPtr msg);
  double mpcHealthAgeSec(double now_sec) const;
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
               double path_curvature, double signed_path_curvature,
               double curvature_window_distance, double lookahead_point_x,
               double lookahead_point_y, double rear_x, double rear_y,
               double alpha, double pure_pursuit_steering_tire_angle,
               double curvature_feedforward_steering_rad,
               double raw_steering_tire_angle, double steering_tire_angle,
               bool overtake_override_applied, double overtake_lateral_offset_m,
               double overtake_speed_cap_mps, double freshness_now_sec,
               bool mpc_horizon_applied, bool mpc_horizon_velocity_cap_applied,
               double mpc_horizon_velocity_cap_mps,
               std::int32_t evaluated_horizon_stamp_sec,
               std::uint32_t evaluated_horizon_stamp_nanosec,
               std::int32_t applied_horizon_stamp_sec,
               std::uint32_t applied_horizon_stamp_nanosec,
               const std::string &applied_horizon_source,
               int applied_horizon_mode_id,
               std::uint32_t applied_horizon_generation,
               const HorizonFreshnessResult &mpc_horizon_freshness,
               const std::string &trajectory_source,
               const ControlPosePrediction &control_pose);
};

} // namespace simple_pure_pursuit

#endif // SIMPLE_PURE_PURSUIT_HPP_
