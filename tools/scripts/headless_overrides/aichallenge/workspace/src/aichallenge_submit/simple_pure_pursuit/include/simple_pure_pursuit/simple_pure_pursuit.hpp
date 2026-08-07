#ifndef SIMPLE_PURE_PURSUIT_HPP_
#define SIMPLE_PURE_PURSUIT_HPP_

#include "overtake_transport_contract/c002ay0_worker_ipc.hpp"
#include "overtake_transport_contract/c002ay1_runtime_observer.hpp"
#include "simple_pure_pursuit/aw2_shadow_ipc.hpp"
#include "simple_pure_pursuit/ay0_base_capture.hpp"
#include "simple_pure_pursuit/lookahead.hpp"
#include "simple_pure_pursuit/overtake_override_contract.hpp"
#include "simple_pure_pursuit/safety.hpp"

#include <array>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <memory>
#include <multi_purpose_mpc_ros_msgs/msg/controller_applied_envelope.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/controller_command_envelope.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/controller_execution_envelope.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/controller_tracking_status.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/execution_sweep_sample.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/overtake_plan.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/recovery_control_command.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/recovery_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
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
using multi_purpose_mpc_ros_msgs::msg::ControllerAppliedEnvelope;
using multi_purpose_mpc_ros_msgs::msg::ControllerCommandEnvelope;
using multi_purpose_mpc_ros_msgs::msg::ControllerExecutionEnvelope;
using multi_purpose_mpc_ros_msgs::msg::ControllerExecutionWitness;
using multi_purpose_mpc_ros_msgs::msg::ControllerTrackingStatus;
using multi_purpose_mpc_ros_msgs::msg::ExecutionSweepSample;
using multi_purpose_mpc_ros_msgs::msg::OvertakePlan;
using multi_purpose_mpc_ros_msgs::msg::RecoveryControlCommand;
using multi_purpose_mpc_ros_msgs::msg::RecoveryStatus;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Float32MultiArray;
using std_msgs::msg::String;

inline ControllerCommandEnvelope
makeControllerCommandEnvelopeV1(const AckermannControlCommand &command,
                                const ControllerTrackingStatus &tracking_status,
                                std::uint64_t producer_instance_id,
                                std::uint64_t command_sequence,
                                const OvertakePlan *typed_plan = nullptr) {
  ControllerCommandEnvelope envelope;
  envelope.header = tracking_status.header;
  const bool typed_plan_matches =
      typed_plan != nullptr &&
      typed_plan->aw2_identity_schema_version == 1U &&
      typed_plan->plan_generation == tracking_status.plan_generation &&
      typed_plan->planner_instance_id != 0U &&
      typed_plan->race_arm_epoch != 0U &&
      typed_plan->attempt_id != 0U &&
      !typed_plan->target_vehicle_id.empty() &&
      (typed_plan->pass_direction == -1 || typed_plan->pass_direction == 1) &&
      typed_plan->connector_transaction_id != 0U &&
      typed_plan->candidate_revision != 0U;
  envelope.schema_version = typed_plan_matches ? 2U : 1U;
  envelope.producer_instance_id = producer_instance_id;
  envelope.command_sequence = command_sequence;
  envelope.plan_generation = tracking_status.plan_generation;
  envelope.command = command;
  envelope.mpc_horizon_usable = tracking_status.mpc_horizon_usable;
  envelope.pp_command_fresh = tracking_status.pp_command_fresh;
  envelope.trajectory_tracking_usable =
      tracking_status.trajectory_tracking_usable;
  envelope.lateral_stop_authority_kind =
      tracking_status.lateral_stop_authority_kind;
  envelope.lateral_stop_transaction_pass_direction =
      tracking_status.lateral_stop_transaction_pass_direction;
  envelope.lateral_stop_authority_token =
      tracking_status.lateral_stop_authority_token;
  envelope.command_age_sec = tracking_status.command_age_sec;
  envelope.reason = tracking_status.reason;
  if (typed_plan_matches) {
    envelope.plan_sample_key.race_arm_epoch = typed_plan->race_arm_epoch;
    envelope.plan_sample_key.planner_instance_id =
        typed_plan->planner_instance_id;
    envelope.plan_sample_key.attempt_id = typed_plan->attempt_id;
    envelope.plan_sample_key.target_vehicle_id =
        typed_plan->target_vehicle_id;
    envelope.plan_sample_key.pass_direction = typed_plan->pass_direction;
    envelope.plan_sample_key.connector_transaction_id =
        typed_plan->connector_transaction_id;
    envelope.plan_sample_key.plan_stamp = typed_plan->header.stamp;
    envelope.plan_sample_key.plan_generation = typed_plan->plan_generation;
    envelope.candidate_revision = typed_plan->candidate_revision;
    envelope.candidate_content_sha256 =
        typed_plan->candidate_content_sha256;
  }
  return envelope;
}

inline constexpr std::size_t kMaxExecutionTrajectoryPoints = 100U;
inline constexpr std::size_t kMaxExecutionRolloutSamples = 100U;
inline constexpr std::size_t kMaxExecutionSourcePayloadBytes = 4096U;

struct CanonicalSourcePayload {
  std::vector<std::uint8_t> bytes;
  std::uint64_t original_size_bytes{0U};
  bool complete{false};
};

CanonicalSourcePayload
canonicalizeSourcePayload(const Float32MultiArray &source);

bool typedPlanOrderAcceptable(
    const builtin_interfaces::msg::Time &candidate_stamp,
    std::uint32_t candidate_generation,
    const builtin_interfaces::msg::Time &now_stamp, bool has_previous,
    const builtin_interfaces::msg::Time &previous_stamp,
    std::uint32_t previous_generation);

// Adapter-neutral inputs captured from the same control cycle.
struct ControllerExecutionWitnessInput {
  std_msgs::msg::Header header;
  std::uint8_t controller_role{ControllerExecutionWitness::ROLE_UNKNOWN};
  std::uint8_t trajectory_source{ControllerExecutionWitness::SOURCE_UNKNOWN};
  std::uint32_t source_generation{0U};
  CanonicalSourcePayload source_payload;
  builtin_interfaces::msg::Time source_stamp;
  builtin_interfaces::msg::Time reference_stamp;
  Trajectory base_trajectory;
  Trajectory applied_trajectory;
  std::size_t nearest_trajectory_index{0U};
  Pose control_pose;
  double trajectory_progress_m{0.0};
  double available_spatial_horizon_m{0.0};
  double required_spatial_horizon_m{0.0};
  double raw_steering_tire_angle_rad{0.0};
  double bounded_steering_tire_angle_rad{0.0};
  double raw_steering_tire_rotation_rate_radps{
      std::numeric_limits<double>::quiet_NaN()};
  double bounded_steering_tire_rotation_rate_radps{
      std::numeric_limits<double>::quiet_NaN()};
  double steering_tire_angle_limit_rad{
      std::numeric_limits<double>::quiet_NaN()};
  double steering_tire_rotation_rate_limit_radps{
      std::numeric_limits<double>::quiet_NaN()};
  std::vector<ExecutionSweepSample> rollout_samples;
};

ControllerExecutionEnvelope makeControllerExecutionEnvelopeV1(
    const ControllerCommandEnvelope &command_envelope,
    const ControllerExecutionWitnessInput &input,
    const OvertakePlan *typed_plan, bool typed_plan_fresh);

using Aw2Sha256Digest = std::array<std::uint8_t, 32U>;

struct Aw2CanonicalSourceWire {
  std::vector<std::uint8_t> bytes;
  std::uint32_t original_size_bytes{0U};
  Aw2Sha256Digest sha256{};
  bool complete{false};
};

Aw2Sha256Digest aw2Sha256(const std::vector<std::uint8_t> &bytes);
Aw2CanonicalSourceWire serializeAw2SourceWire(const Float32MultiArray &source);
Aw2CanonicalSourceWire
canonicalizeAw2SourceWire(const Float32MultiArray &source);
Aw2Sha256Digest aw2ControllerAdapterImplementationDigestV1();
Aw2Sha256Digest
aw2ControllerAdapterConfigDigestV1(const std::vector<double> &scalar_values,
                                   const std::vector<bool> &boolean_values);

struct ControllerAppliedEnvelopeInput {
  bool capture_resource_limit_exceeded{false};
  std::string capture_resource_limit_reason;
  bool resource_sample_key_present{false};
  multi_purpose_mpc_ros_msgs::msg::ControllerSampleKey resource_sample_key;
  std::uint32_t resource_candidate_revision{0U};
  Aw2Sha256Digest resource_candidate_content_sha256{};
  std_msgs::msg::Header header;
  std::uint8_t controller_role{ControllerAppliedEnvelope::ROLE_UNKNOWN};
  std::uint8_t geometry_relation{
      ControllerAppliedEnvelope::GEOMETRY_RELATION_UNKNOWN};
  std::uint32_t source_generation{0U};
  Aw2CanonicalSourceWire source_wire;
  Trajectory base_trajectory;
  Trajectory applied_trajectory;
  std::uint32_t base_original_point_count{0U};
  std::uint32_t applied_original_point_count{0U};
  Pose control_pose;
  builtin_interfaces::msg::Time control_pose_stamp;
  std::size_t nearest_trajectory_index{0U};
  std::size_t speed_cap_trajectory_index{0U};
  std::size_t curvature_last_read_trajectory_index{0U};
  std::size_t lookahead_selected_trajectory_index{0U};
  std::size_t required_horizon_end_trajectory_index{0U};
  bool lookahead_endpoint_fallback{false};
  double trajectory_progress_m{0.0};
  Aw2Sha256Digest controller_adapter_implementation_sha256{};
  Aw2Sha256Digest controller_adapter_config_sha256{};
  AckermannControlCommand raw_controller_command;
  double raw_steering_tire_angle_rad{0.0};
  double output_steering_tire_angle_rad{0.0};
  double raw_steering_tire_rotation_rate_radps{
      std::numeric_limits<double>::quiet_NaN()};
  double output_steering_tire_rotation_rate_radps{
      std::numeric_limits<double>::quiet_NaN()};
  bool hard_actuator_limits_present{false};
  double hard_steering_tire_angle_limit_rad{0.0};
  double hard_steering_tire_rotation_rate_limit_radps{0.0};
  double available_spatial_horizon_m{0.0};
  double required_spatial_horizon_m{0.0};
  std::uint8_t rollout_state{ControllerAppliedEnvelope::ROLLOUT_UNKNOWN};
  std::vector<ExecutionSweepSample> rollout_samples;
};

class ControllerAppliedBindingTracker {
public:
  enum class Observation : std::uint8_t {
    ACCEPTED,
    CONSISTENT_DUPLICATE,
    DUPLICATE_CONFLICT,
    SAME_GENERATION_PAYLOAD_MUTATION,
    SEQUENCE_REGRESSION,
  };

  Observation
  observe(const multi_purpose_mpc_ros_msgs::msg::ControllerSampleKey &key,
          std::uint32_t candidate_revision,
          const Aw2Sha256Digest &candidate_content_sha256,
          const Aw2Sha256Digest &payload_sha256);

private:
  struct Entry {
    multi_purpose_mpc_ros_msgs::msg::ControllerSampleKey key;
    std::uint32_t candidate_revision{0U};
    Aw2Sha256Digest candidate_content_sha256{};
    Aw2Sha256Digest payload_sha256{};
  };
  std::vector<Entry> entries_;
};

ControllerAppliedEnvelope makeControllerAppliedEnvelopeV1(
    const ControllerCommandEnvelope &command_envelope,
    const ControllerAppliedEnvelopeInput &input, const OvertakePlan *typed_plan,
    bool typed_plan_fresh, ControllerAppliedBindingTracker *binding_tracker);

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
  rclcpp::Subscription<RecoveryStatus>::SharedPtr sub_recovery_status_;
  rclcpp::Subscription<OvertakePlan>::SharedPtr sub_overtake_plan_;

  // publishers
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;
  rclcpp::Publisher<RecoveryControlCommand>::SharedPtr pub_recovery_cmd_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_lookahead_point_;
  rclcpp::Publisher<String>::SharedPtr pub_debug_;
  rclcpp::Publisher<ControllerTrackingStatus>::SharedPtr pub_tracking_status_;
  rclcpp::Publisher<ControllerCommandEnvelope>::SharedPtr pub_command_envelope_;
  rclcpp::Publisher<ControllerExecutionEnvelope>::SharedPtr
      pub_execution_envelope_;

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
  std::optional<double> last_recovery_status_receive_sec_;
  RecoveryStatus::SharedPtr recovery_status_;
  OvertakePlan::SharedPtr overtake_plan_;
  std::optional<double> last_overtake_plan_receive_sec_;
  builtin_interfaces::msg::Time last_overtake_plan_stamp_;
  std::uint32_t last_overtake_plan_generation_{0U};
  CanonicalSourcePayload overtake_source_payload_;
  Aw2CanonicalSourceWire aw2_overtake_source_wire_;
  std::uint32_t overtake_source_generation_{0U};
  std::uint32_t reference_source_generation_{0U};
  std::uint32_t mpc_source_generation_{0U};
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
  bool mpc_horizon_contract_solver_horizon_authorized_{false};
  bool mpc_horizon_contract_mandatory_lateral_avoidance_{false};

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
  const bool recovery_mode_;
  const double recovery_status_timeout_sec_;
  const double overtake_override_timeout_sec_;
  const double overtake_short_spatial_horizon_v_max_mps_;
  const double overtake_spatial_horizon_min_arc_m_;
  const double overtake_spatial_horizon_min_time_sec_;
  const double overtake_spatial_horizon_response_delay_sec_;
  const double overtake_spatial_horizon_brake_decel_mps2_;
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
  const std::uint64_t producer_instance_id_;
  std::uint64_t command_sequence_{0U};
  bool has_smoothed_lookahead_distance_{false};
  double smoothed_lookahead_distance_{0.0};
  bool overtake_override_active_{false};
  bool overtake_lateral_override_active_{false};
  bool overtake_speed_only_active_{false};
  bool overtake_solver_horizon_authorized_{false};
  bool overtake_mandatory_lateral_avoidance_{false};
  int overtake_mode_id_{0};
  std::uint32_t overtake_override_generation_{0};
  double last_overtake_override_sec_{-1.0e9};
  double last_valid_override_contract_sec_{-1.0e9};
  std::uint32_t last_valid_override_contract_generation_{0};
  bool last_valid_override_contract_inactive_{false};
  bool last_valid_override_contract_received_{false};
  std::vector<double> overtake_lateral_offsets_;
  std::vector<double> overtake_speed_caps_;
  std::vector<double> overtake_longitudinal_offsets_m_;
  OvertakeSpeedOnlyFailClosedLatch overtake_speed_only_latch_;
  bool aw2_shadow_transport_enabled_{true};
  std::unique_ptr<aw2_shadow::AsyncProducerSession> aw2_shadow_session_;
  bool c002ay1_prod_measure_enabled_{false};
  overtake_transport_contract::c002ay1::RuntimeObservationWriter
      c002ay1_runtime_observer_{};
  bool c002ay0_shadow_capture_enabled_{false};
  std::uint64_t c002ay0_session_generation_{0U};
  std::uint64_t c002ay0_session_nonce_{0U};
  overtake_transport_contract::c002ay0::Digest
      c002ay0_controller_implementation_digest_{};
  overtake_transport_contract::c002ay0::Digest
      c002ay0_controller_config_digest_{};
  std::unique_ptr<overtake_transport_contract::c002ay0::FixedBaseWorkerSession>
      c002ay0_worker_session_;

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
    const Trajectory *base_trajectory{nullptr};
    const Trajectory *trajectory{nullptr};
    std::size_t base_nearest_index{0};
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
    std::string overtake_override_apply_reason{"not_requested"};
    double overtake_spatial_horizon_arc_m{
        std::numeric_limits<double>::quiet_NaN()};
    double overtake_spatial_horizon_required_arc_m{
        std::numeric_limits<double>::quiet_NaN()};
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
    std::size_t speed_cap_trajectory_index{0U};
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
    std::size_t curvature_last_read_trajectory_index{0U};
    std::size_t lookahead_selected_trajectory_index{0U};
    bool lookahead_endpoint_fallback{false};
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
  ControllerTrackingStatus publishControllerTrackingStatus(
      const rclcpp::Time &stamp, const ControlTrajectoryContext *context,
      const AckermannControlCommand *command,
      const LongitudinalCommand *longitudinal, double now_sec);
  std::optional<ControllerCommandEnvelope> publishControllerCommandEnvelope(
      const AckermannControlCommand &command,
      const ControllerTrackingStatus &tracking_status);
  void publishControllerExecutionEnvelope(
      const ControllerCommandEnvelope &command_envelope,
      const ControlTrajectoryContext *context,
      const ControlPosePrediction *control_pose,
      double raw_steering_tire_angle_rad, double now_sec);
  void captureControllerAppliedShadow(
      const ControllerCommandEnvelope &command_envelope,
      const ControlTrajectoryContext *context,
      const ControlPosePrediction *control_pose,
      const AckermannControlCommand &raw_command,
      double required_spatial_horizon_m, std::size_t speed_cap_trajectory_index,
      std::size_t curvature_last_read_trajectory_index,
      std::size_t lookahead_selected_trajectory_index,
      std::size_t required_horizon_end_trajectory_index,
      bool lookahead_endpoint_fallback, double now_sec) noexcept;
  void captureAy0BaseShadow(const ControllerCommandEnvelope &command_envelope,
                            const ControlTrajectoryContext &context) noexcept;
  void onOvertakeOverride(const Float32MultiArray::SharedPtr msg);
  void onMpcPredictedHorizonContract(const String::SharedPtr msg);
  void onMpcHealth(const String::SharedPtr msg);
  void onRecoveryStatus(const RecoveryStatus::SharedPtr msg);
  double mpcHealthAgeSec(double now_sec) const;
  bool recoveryStatusAllowsControl(double now_sec,
                                   const Trajectory &control_trajectory,
                                   std::string *reason) const;
  void publishRecoveryControlCommand(const AckermannControlCommand &cmd,
                                     const Trajectory &control_trajectory);
  void clearOvertakeOverride();
  void applyReceivedOvertakeOverride(const OvertakeOverrideContract &contract);
  bool hasLatchedSpeedOnlyCap() const;
  bool applyOvertakeOverride(Trajectory &trajectory,
                             std::size_t nearest_traj_point_idx, double now_sec,
                             double *applied_spatial_arc_m,
                             double *required_spatial_arc_m,
                             std::string *apply_reason);
  bool overtakeOverrideFresh(double now_sec) const;
  std::optional<double> overtakeSpeedCap(std::size_t horizon_index,
                                         double now_sec) const;
  double overtakeLateralOffset(std::size_t horizon_index) const;
  void publishDebug(
      const rclcpp::Time &stamp, const Trajectory &control_trajectory,
      std::size_t nearest_traj_point_idx, double target_longitudinal_vel,
      double current_longitudinal_vel, double command_accel,
      double base_lookahead_distance, double desired_lookahead_distance,
      double lookahead_distance, double path_curvature,
      double signed_path_curvature, double curvature_window_distance,
      double lookahead_point_x, double lookahead_point_y, double rear_x,
      double rear_y, double alpha, double pure_pursuit_steering_tire_angle,
      double curvature_feedforward_steering_rad, double raw_steering_tire_angle,
      double steering_tire_angle, bool overtake_override_applied,
      double overtake_lateral_offset_m, double overtake_speed_cap_mps,
      double freshness_now_sec, bool mpc_horizon_applied,
      bool mpc_horizon_velocity_cap_applied,
      double mpc_horizon_velocity_cap_mps,
      std::int32_t evaluated_horizon_stamp_sec,
      std::uint32_t evaluated_horizon_stamp_nanosec,
      std::int32_t applied_horizon_stamp_sec,
      std::uint32_t applied_horizon_stamp_nanosec,
      const std::string &applied_horizon_source, int applied_horizon_mode_id,
      std::uint32_t applied_horizon_generation,
      const HorizonFreshnessResult &mpc_horizon_freshness,
      const std::string &trajectory_source,
      const std::string &overtake_override_apply_reason,
      double overtake_spatial_horizon_arc_m,
      double overtake_spatial_horizon_required_arc_m,
      const ControlPosePrediction &control_pose);
};

} // namespace simple_pure_pursuit

#endif // SIMPLE_PURE_PURSUIT_HPP_
