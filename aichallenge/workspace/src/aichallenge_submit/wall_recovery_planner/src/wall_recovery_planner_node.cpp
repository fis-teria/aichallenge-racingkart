#include "wall_recovery_planner/wall_recovery_core.hpp"

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/recovery_permit.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/recovery_status.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/recovery_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace wall_recovery_planner {
namespace {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using multi_purpose_mpc_ros_msgs::msg::RecoveryPermit;
using multi_purpose_mpc_ros_msgs::msg::RecoveryStatus;
using multi_purpose_mpc_ros_msgs::msg::RecoveryTrajectory;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Int32;
using std_msgs::msg::String;

double steadyNowSec() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double distance2d(double ax, double ay, double bx, double by) {
  return std::hypot(ax - bx, ay - by);
}

std::string stateName(std::uint8_t state) {
  switch (state) {
    case RecoveryStatus::INACTIVE:
      return "INACTIVE";
    case RecoveryStatus::STUCK_CONFIRMING:
      return "STUCK_CONFIRMING";
    case RecoveryStatus::STOP_HOLD:
      return "STOP_HOLD";
    case RecoveryStatus::ACTIVE:
      return "ACTIVE";
    case RecoveryStatus::HANDOFF_VERIFY:
      return "HANDOFF_VERIFY";
    case RecoveryStatus::COMPLETE:
      return "COMPLETE";
    case RecoveryStatus::ABORT:
      return "ABORT";
    case RecoveryStatus::LOCKOUT:
      return "LOCKOUT";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

class WallRecoveryPlannerNode : public rclcpp::Node {
public:
  WallRecoveryPlannerNode()
      : Node("wall_recovery_planner_node"),
        collision_tracker_(declare_parameter<int>("collision_edge_min_delta", 30)) {
    enabled_ = declare_parameter<bool>("wall_recovery_enabled", false);
    allow_unverified_recovery_ =
        declare_parameter<bool>("allow_unverified_recovery", false);
    require_recovery_permit_ =
        declare_parameter<bool>("require_recovery_permit", true);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    collision_event_ttl_sec_ =
        declare_parameter<double>("collision_event_ttl_sec", 2.0);
    stuck_confirm_sec_ = declare_parameter<double>("stuck_confirm_sec", 1.0);
    stuck_speed_threshold_mps_ =
        declare_parameter<double>("stuck_speed_threshold_mps", 0.10);
    stuck_progress_threshold_m_ =
        declare_parameter<double>("stuck_progress_threshold_m", 0.10);
    stuck_min_command_speed_mps_ =
        declare_parameter<double>("stuck_min_command_speed_mps", 0.50);
    stop_hold_sec_ = declare_parameter<double>("stop_hold_sec", 0.25);
    permit_wait_timeout_sec_ =
        declare_parameter<double>("permit_wait_timeout_sec", 0.75);
    recovery_v_max_mps_ = declare_parameter<double>("recovery_v_max_mps", 0.7);
    recovery_max_duration_sec_ =
        declare_parameter<double>("recovery_max_duration_sec", 3.0);
    recovery_no_progress_timeout_sec_ =
        declare_parameter<double>("recovery_no_progress_timeout_sec", 1.0);
    recovery_max_attempts_ = declare_parameter<int>("recovery_max_attempts", 1);
    max_odom_age_sec_ = declare_parameter<double>("max_odom_age_sec", 0.20);
    max_trajectory_age_sec_ =
        declare_parameter<double>("max_trajectory_age_sec", 0.50);
    max_control_cmd_age_sec_ =
        declare_parameter<double>("max_control_cmd_age_sec", 0.20);
    max_permit_age_sec_ = declare_parameter<double>("max_permit_age_sec", 0.30);
    min_rejoin_arc_distance_m_ =
        declare_parameter<double>("min_rejoin_arc_distance_m", 1.0);
    max_rejoin_arc_distance_m_ =
        declare_parameter<double>("max_rejoin_arc_distance_m", 4.0);
    max_rejoin_heading_error_rad_ =
        declare_parameter<double>("max_rejoin_heading_error_rad", 1.2);
    max_trajectory_segment_length_m_ =
        declare_parameter<double>("max_trajectory_segment_length_m", 5.0);
    recovery_trajectory_points_ =
        declare_parameter<int>("recovery_trajectory_points", 12);
    handoff_distance_threshold_m_ =
        declare_parameter<double>("handoff_distance_threshold_m", 0.5);
    complete_publish_window_sec_ =
        declare_parameter<double>("complete_publish_window_sec", 0.5);
    debug_publish_period_sec_ =
        declare_parameter<double>("debug_publish_period_sec", 0.25);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    sub_collision_ = create_subscription<Int32>(
        "input/pitstop_condition", qos,
        [this](const Int32::SharedPtr msg) { onCollisionCounter(msg); });
    sub_odom_ = create_subscription<Odometry>(
        "input/kinematics", qos, [this](const Odometry::SharedPtr msg) {
          odometry_ = msg;
          last_odom_sec_ = steadyNowSec();
        });
    sub_trajectory_ = create_subscription<Trajectory>(
        "input/reference_trajectory", qos,
        [this](const Trajectory::SharedPtr msg) {
          reference_trajectory_ = msg;
          last_trajectory_sec_ = steadyNowSec();
        });
    sub_control_cmd_ = create_subscription<AckermannControlCommand>(
        "input/control_cmd", qos,
        [this](const AckermannControlCommand::SharedPtr msg) {
          control_cmd_ = msg;
          last_control_cmd_sec_ = steadyNowSec();
        });
    sub_permit_ = create_subscription<RecoveryPermit>(
        "input/recovery_permit", qos,
        [this](const RecoveryPermit::SharedPtr msg) {
          recovery_permit_ = msg;
          last_permit_sec_ = steadyNowSec();
        });

    candidate_pub_ =
        create_publisher<RecoveryTrajectory>("output/candidate_trajectory", 1);
    trajectory_pub_ = create_publisher<Trajectory>("output/trajectory", 1);
    status_pub_ = create_publisher<RecoveryStatus>("output/status", 1);
    debug_pub_ = create_publisher<String>("output/debug", 1);

    const double period_sec = 1.0 / std::max(1.0, control_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(period_sec)),
        std::bind(&WallRecoveryPlannerNode::onTimer, this));
  }

private:
  void onCollisionCounter(const Int32::SharedPtr msg) {
    const auto event = collision_tracker_.observe(msg->data);
    if (!event.event) {
      return;
    }
    last_collision_event_seq_ = event.sequence;
    last_collision_event_sec_ = steadyNowSec();
  }

  void onTimer() {
    const double now_sec = steadyNowSec();
    if (!enabled_) {
      resetEpisode();
      publishStatus(RecoveryStatus::INACTIVE, "disabled", false, false, false,
                    false);
      publishDebug(now_sec, "disabled");
      return;
    }

    switch (state_) {
      case RecoveryStatus::INACTIVE:
      case RecoveryStatus::COMPLETE:
        updateInactive(now_sec);
        break;
      case RecoveryStatus::STUCK_CONFIRMING:
        updateStuckConfirming(now_sec);
        break;
      case RecoveryStatus::STOP_HOLD:
        updateStopHold(now_sec);
        break;
      case RecoveryStatus::ACTIVE:
        updateActive(now_sec);
        break;
      case RecoveryStatus::HANDOFF_VERIFY:
        updateHandoffVerify(now_sec);
        break;
      case RecoveryStatus::LOCKOUT:
      case RecoveryStatus::ABORT:
      default:
        publishStatus(state_, lockout_reason_, true, candidate_ready_,
                      false, false);
        publishDebug(now_sec, lockout_reason_);
        break;
    }
  }

  void updateInactive(double now_sec) {
    if (state_ == RecoveryStatus::COMPLETE &&
        now_sec - complete_start_sec_ < complete_publish_window_sec_) {
      publishStatus(RecoveryStatus::COMPLETE, "handoff_ready", false,
                    candidate_ready_, true, true);
      publishDebug(now_sec, "handoff_ready");
      return;
    }

    if (!stuckCandidate(now_sec)) {
      resetEpisode();
      publishStatus(RecoveryStatus::INACTIVE, last_inactive_reason_, false,
                    false, false, false);
      publishDebug(now_sec, last_inactive_reason_);
      return;
    }

    state_ = RecoveryStatus::STUCK_CONFIRMING;
    stuck_start_sec_ = now_sec;
    stuck_start_x_ = odometry_->pose.pose.position.x;
    stuck_start_y_ = odometry_->pose.pose.position.y;
    publishStatus(state_, "stuck_candidate", false, false, false, false);
    publishDebug(now_sec, "stuck_candidate");
  }

  void updateStuckConfirming(double now_sec) {
    if (!stuckCandidate(now_sec)) {
      state_ = RecoveryStatus::INACTIVE;
      publishStatus(RecoveryStatus::INACTIVE, last_inactive_reason_, false,
                    false, false, false);
      publishDebug(now_sec, last_inactive_reason_);
      return;
    }

    if (now_sec - stuck_start_sec_ < stuck_confirm_sec_) {
      publishStatus(state_, "confirming_stuck", false, false, false, false);
      publishDebug(now_sec, "confirming_stuck");
      return;
    }

    if (attempt_count_ >= std::max(1, recovery_max_attempts_)) {
      enterLockout("max_attempts_exceeded");
      return;
    }

    ++attempt_count_;
    ++attempt_id_;
    ++trajectory_generation_;
    if (!buildCandidateTrajectory()) {
      enterLockout(lockout_reason_);
      return;
    }
    state_ = RecoveryStatus::STOP_HOLD;
    stop_hold_start_sec_ = now_sec;
    permit_wait_start_sec_ = now_sec;
    publishCandidate();
    publishStatus(state_, "stop_hold_before_recovery", true, true, false,
                  false);
    publishDebug(now_sec, "stop_hold_before_recovery");
  }

  void updateStopHold(double now_sec) {
    publishCandidate();
    if (now_sec - stop_hold_start_sec_ < stop_hold_sec_) {
      publishStatus(state_, "stop_hold_before_recovery", true, true, false,
                    false);
      publishDebug(now_sec, "stop_hold_before_recovery");
      return;
    }

    const auto permit_reason = recoveryPermitReady(now_sec, false);
    if (!permit_reason.empty()) {
      if (now_sec - permit_wait_start_sec_ > permit_wait_timeout_sec_) {
        enterLockout(permit_reason);
      } else {
        publishStatus(state_, permit_reason, true, true, false, false);
        publishDebug(now_sec, permit_reason);
      }
      return;
    }

    state_ = RecoveryStatus::ACTIVE;
    active_start_sec_ = now_sec;
    active_progress_sec_ = now_sec;
    active_progress_x_ = odometry_->pose.pose.position.x;
    active_progress_y_ = odometry_->pose.pose.position.y;
    active_start_collision_seq_ = last_collision_event_seq_;
    publishRecoveryTrajectory();
    publishStatus(state_, "recovery_active", false, true, true, false);
    publishDebug(now_sec, "recovery_active");
  }

  void updateActive(double now_sec) {
    if (!fresh(last_odom_sec_, now_sec, max_odom_age_sec_) || !odometry_) {
      enterLockout("stale_odom_active");
      return;
    }
    if (last_collision_event_seq_ > active_start_collision_seq_) {
      enterLockout("collision_during_recovery");
      return;
    }
    if (now_sec - active_start_sec_ > recovery_max_duration_sec_) {
      enterLockout("recovery_timeout");
      return;
    }
    if (!recoveryPermitReady(now_sec, false).empty()) {
      enterLockout("recovery_permit_lost");
      return;
    }

    const double progress =
        distance2d(odometry_->pose.pose.position.x, odometry_->pose.pose.position.y,
                   active_progress_x_, active_progress_y_);
    if (progress > stuck_progress_threshold_m_) {
      active_progress_sec_ = now_sec;
      active_progress_x_ = odometry_->pose.pose.position.x;
      active_progress_y_ = odometry_->pose.pose.position.y;
    } else if (now_sec - active_progress_sec_ > recovery_no_progress_timeout_sec_) {
      enterLockout("no_progress_during_recovery");
      return;
    }

    publishRecoveryTrajectory();
    if (recovery_trajectory_.points.empty()) {
      enterLockout("empty_recovery_trajectory");
      return;
    }
    const auto &goal = recovery_trajectory_.points.back().pose.position;
    const double goal_distance =
        distance2d(odometry_->pose.pose.position.x, odometry_->pose.pose.position.y,
                   goal.x, goal.y);
    if (goal_distance <= handoff_distance_threshold_m_) {
      state_ = RecoveryStatus::HANDOFF_VERIFY;
      handoff_start_sec_ = now_sec;
      publishStatus(state_, "handoff_verify", false, true, true, false);
      publishDebug(now_sec, "handoff_verify");
      return;
    }

    publishStatus(state_, "recovery_active", false, true, true, false);
    publishDebug(now_sec, "recovery_active");
  }

  void updateHandoffVerify(double now_sec) {
    if (!fresh(last_odom_sec_, now_sec, max_odom_age_sec_) || !odometry_) {
      enterLockout("stale_odom_handoff");
      return;
    }
    const auto permit_reason = recoveryPermitReady(now_sec, true);
    if (!permit_reason.empty()) {
      if (now_sec - handoff_start_sec_ > recovery_no_progress_timeout_sec_) {
        enterLockout(permit_reason);
      } else {
        publishRecoveryTrajectory();
        publishStatus(state_, permit_reason, false, true, true, false);
        publishDebug(now_sec, permit_reason);
      }
      return;
    }
    state_ = RecoveryStatus::COMPLETE;
    complete_start_sec_ = now_sec;
    publishStatus(state_, "handoff_ready", false, true, true, true);
    publishDebug(now_sec, "handoff_ready");
  }

  bool stuckCandidate(double now_sec) {
    if (!fresh(last_collision_event_sec_, now_sec, collision_event_ttl_sec_)) {
      last_inactive_reason_ = "no_recent_collision_event";
      return false;
    }
    if (!fresh(last_odom_sec_, now_sec, max_odom_age_sec_) || !odometry_) {
      last_inactive_reason_ = "missing_or_stale_odom";
      return false;
    }
    if (!fresh(last_control_cmd_sec_, now_sec, max_control_cmd_age_sec_) ||
        !control_cmd_) {
      last_inactive_reason_ = "missing_or_stale_control_cmd";
      return false;
    }
    if (control_cmd_->longitudinal.speed < stuck_min_command_speed_mps_) {
      last_inactive_reason_ = "no_forward_command";
      return false;
    }
    if (std::abs(odometry_->twist.twist.linear.x) > stuck_speed_threshold_mps_) {
      last_inactive_reason_ = "ego_is_moving";
      return false;
    }
    if (state_ == RecoveryStatus::STUCK_CONFIRMING) {
      const double progress =
          distance2d(odometry_->pose.pose.position.x, odometry_->pose.pose.position.y,
                     stuck_start_x_, stuck_start_y_);
      if (progress > stuck_progress_threshold_m_) {
        last_inactive_reason_ = "progress_recovered";
        return false;
      }
    }
    last_inactive_reason_ = "stuck_candidate";
    return true;
  }

  bool buildCandidateTrajectory() {
    candidate_ready_ = false;
    if (!fresh(last_trajectory_sec_, steadyNowSec(), max_trajectory_age_sec_) ||
        !reference_trajectory_ || reference_trajectory_->points.empty()) {
      lockout_reason_ = "missing_or_stale_reference_trajectory";
      return false;
    }
    if (!odometry_) {
      lockout_reason_ = "missing_odom";
      return false;
    }

    std::vector<Waypoint2d> waypoints;
    waypoints.reserve(reference_trajectory_->points.size());
    for (const auto &point : reference_trajectory_->points) {
      Waypoint2d waypoint;
      waypoint.x = point.pose.position.x;
      waypoint.y = point.pose.position.y;
      waypoint.yaw = tf2::getYaw(point.pose.orientation);
      waypoint.velocity_mps = point.longitudinal_velocity_mps;
      waypoints.push_back(waypoint);
    }

    ForwardWaypointConfig config;
    config.min_arc_distance_m = min_rejoin_arc_distance_m_;
    config.max_arc_distance_m = max_rejoin_arc_distance_m_;
    config.max_heading_error_rad = max_rejoin_heading_error_rad_;
    config.max_segment_length_m = max_trajectory_segment_length_m_;

    const double ego_yaw = tf2::getYaw(odometry_->pose.pose.orientation);
    const auto selected = selectForwardWaypoint(
        waypoints, odometry_->pose.pose.position.x, odometry_->pose.pose.position.y,
        ego_yaw, config);
    if (!selected.valid) {
      lockout_reason_ = "target_" + selected.reason;
      return false;
    }

    const auto built = buildRecoveryTrajectory(
        odometry_->pose.pose.position.x, odometry_->pose.pose.position.y,
        ego_yaw, waypoints.at(selected.index),
        std::min(std::max(0.0, recovery_v_max_mps_),
                 std::max(0.0, waypoints.at(selected.index).velocity_mps)),
        static_cast<std::size_t>(std::max(2, recovery_trajectory_points_)));
    if (built.size() < 2) {
      lockout_reason_ = "candidate_generation_failed";
      return false;
    }

    recovery_trajectory_ = Trajectory();
    recovery_trajectory_.header = reference_trajectory_->header;
    recovery_trajectory_.header.stamp = get_clock()->now();
    recovery_trajectory_.points.reserve(built.size());
    for (const auto &point : built) {
      TrajectoryPoint trajectory_point;
      trajectory_point.pose.position.x = point.x;
      trajectory_point.pose.position.y = point.y;
      trajectory_point.pose.position.z = odometry_->pose.pose.position.z;
      trajectory_point.pose.orientation = yawToQuaternion(point.yaw);
      trajectory_point.longitudinal_velocity_mps =
          static_cast<float>(std::min(std::max(0.0, point.velocity_mps),
                                      std::max(0.0, recovery_v_max_mps_)));
      trajectory_point.acceleration_mps2 = 0.0F;
      recovery_trajectory_.points.push_back(trajectory_point);
    }
    candidate_ready_ = true;
    return true;
  }

  std::string recoveryPermitReady(double now_sec, bool handoff) const {
    if (!candidate_ready_) {
      return "missing_candidate_trajectory";
    }
    if (!require_recovery_permit_) {
      if (allow_unverified_recovery_) {
        return "";
      }
      return "unverified_recovery_disabled";
    }
    if (!fresh(last_permit_sec_, now_sec, max_permit_age_sec_) ||
        !recovery_permit_) {
      return "missing_or_stale_recovery_permit";
    }
    if (recovery_permit_->attempt_id != attempt_id_ ||
        recovery_permit_->trajectory_generation != trajectory_generation_) {
      return "recovery_permit_generation_mismatch";
    }
    if (recovery_permit_->safe_stop_active) {
      return "safe_stop_active";
    }
    if (!recovery_permit_->cbf_safe) {
      return "cbf_unsafe";
    }
    if (!recovery_permit_->opponents_fresh ||
        !recovery_permit_->reference_fresh) {
      return "permit_inputs_stale";
    }
    if (handoff) {
      return recovery_permit_->handoff_allowed ? "" : "handoff_not_allowed";
    }
    return recovery_permit_->recovery_allowed ? "" : "recovery_not_allowed";
  }

  void publishCandidate() {
    if (!candidate_ready_) {
      return;
    }
    RecoveryTrajectory msg;
    msg.header = recovery_trajectory_.header;
    msg.attempt_id = attempt_id_;
    msg.trajectory_generation = trajectory_generation_;
    msg.trajectory = recovery_trajectory_;
    candidate_pub_->publish(msg);
  }

  void publishRecoveryTrajectory() {
    if (candidate_ready_) {
      trajectory_pub_->publish(recovery_trajectory_);
    }
  }

  void publishStatus(std::uint8_t state, const std::string &reason,
                     bool stop_required, bool trajectory_valid,
                     bool trajectory_safe, bool handoff_ready) {
    RecoveryStatus msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id =
        reference_trajectory_ ? reference_trajectory_->header.frame_id : "map";
    msg.state = state;
    msg.attempt_id = attempt_id_;
    msg.trajectory_generation = trajectory_generation_;
    msg.trajectory_header = candidate_ready_ ? recovery_trajectory_.header
                                             : std_msgs::msg::Header();
    msg.input_complete = odometry_ != nullptr && control_cmd_ != nullptr &&
                         reference_trajectory_ != nullptr;
    msg.trajectory_valid = trajectory_valid;
    msg.trajectory_safe = trajectory_safe;
    msg.stop_required = stop_required;
    msg.handoff_ready = handoff_ready;
    msg.min_wall_clearance_m =
        recovery_permit_ ? recovery_permit_->min_wall_clearance_m
                         : -std::numeric_limits<float>::infinity();
    msg.min_vehicle_margin_m =
        recovery_permit_ ? recovery_permit_->min_vehicle_margin_m
                         : -std::numeric_limits<float>::infinity();
    msg.reason = reason;
    status_pub_->publish(msg);
  }

  void publishDebug(double now_sec, const std::string &reason) {
    if (debug_publish_period_sec_ <= 0.0 ||
        now_sec - last_debug_publish_sec_ < debug_publish_period_sec_) {
      return;
    }
    last_debug_publish_sec_ = now_sec;
    std::ostringstream json;
    json << "{"
         << "\"controller\":\"wall_recovery_planner\","
         << "\"enabled\":" << (enabled_ ? "true" : "false") << ","
         << "\"state\":\"" << stateName(state_) << "\","
         << "\"attempt_id\":" << attempt_id_ << ","
         << "\"trajectory_generation\":" << trajectory_generation_ << ","
         << "\"candidate_ready\":" << (candidate_ready_ ? "true" : "false")
         << ","
         << "\"last_collision_seq\":" << last_collision_event_seq_ << ","
         << "\"reason\":\"" << reason << "\"}";
    String msg;
    msg.data = json.str();
    debug_pub_->publish(msg);
  }

  void enterLockout(const std::string &reason) {
    state_ = RecoveryStatus::LOCKOUT;
    lockout_reason_ = reason;
    publishStatus(state_, reason, true, candidate_ready_, false, false);
    publishDebug(steadyNowSec(), reason);
    RCLCPP_WARN(get_logger(), "wall recovery lockout: %s", reason.c_str());
  }

  void resetEpisode() {
    state_ = RecoveryStatus::INACTIVE;
    candidate_ready_ = false;
    recovery_trajectory_ = Trajectory();
    lockout_reason_ = "inactive";
  }

  static bool fresh(const std::optional<double> &stamp_sec, double now_sec,
                    double timeout_sec) {
    return stamp_sec.has_value() && now_sec - stamp_sec.value() <= timeout_sec;
  }

  bool enabled_{false};
  bool allow_unverified_recovery_{false};
  bool require_recovery_permit_{true};
  double control_rate_hz_{20.0};
  double collision_event_ttl_sec_{2.0};
  double stuck_confirm_sec_{1.0};
  double stuck_speed_threshold_mps_{0.10};
  double stuck_progress_threshold_m_{0.10};
  double stuck_min_command_speed_mps_{0.50};
  double stop_hold_sec_{0.25};
  double permit_wait_timeout_sec_{0.75};
  double recovery_v_max_mps_{0.7};
  double recovery_max_duration_sec_{3.0};
  double recovery_no_progress_timeout_sec_{1.0};
  int recovery_max_attempts_{1};
  double max_odom_age_sec_{0.20};
  double max_trajectory_age_sec_{0.50};
  double max_control_cmd_age_sec_{0.20};
  double max_permit_age_sec_{0.30};
  double min_rejoin_arc_distance_m_{1.0};
  double max_rejoin_arc_distance_m_{4.0};
  double max_rejoin_heading_error_rad_{1.2};
  double max_trajectory_segment_length_m_{5.0};
  int recovery_trajectory_points_{12};
  double handoff_distance_threshold_m_{0.5};
  double complete_publish_window_sec_{0.5};
  double debug_publish_period_sec_{0.25};

  CollisionEventTracker collision_tracker_;
  std::uint8_t state_{RecoveryStatus::INACTIVE};
  std::uint32_t attempt_id_{0};
  int attempt_count_{0};
  std::uint32_t trajectory_generation_{0};
  std::uint32_t last_collision_event_seq_{0};
  std::uint32_t active_start_collision_seq_{0};
  std::optional<double> last_collision_event_sec_;
  double stuck_start_sec_{0.0};
  double stuck_start_x_{0.0};
  double stuck_start_y_{0.0};
  double stop_hold_start_sec_{0.0};
  double permit_wait_start_sec_{0.0};
  double active_start_sec_{0.0};
  double active_progress_sec_{0.0};
  double active_progress_x_{0.0};
  double active_progress_y_{0.0};
  double handoff_start_sec_{0.0};
  double complete_start_sec_{0.0};
  double last_debug_publish_sec_{-1.0e9};
  bool candidate_ready_{false};
  std::string last_inactive_reason_{"startup"};
  std::string lockout_reason_{"inactive"};

  Odometry::SharedPtr odometry_;
  Trajectory::SharedPtr reference_trajectory_;
  AckermannControlCommand::SharedPtr control_cmd_;
  RecoveryPermit::SharedPtr recovery_permit_;
  Trajectory recovery_trajectory_;
  std::optional<double> last_odom_sec_;
  std::optional<double> last_trajectory_sec_;
  std::optional<double> last_control_cmd_sec_;
  std::optional<double> last_permit_sec_;

  rclcpp::Subscription<Int32>::SharedPtr sub_collision_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_control_cmd_;
  rclcpp::Subscription<RecoveryPermit>::SharedPtr sub_permit_;
  rclcpp::Publisher<RecoveryTrajectory>::SharedPtr candidate_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<RecoveryStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wall_recovery_planner

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<wall_recovery_planner::WallRecoveryPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
