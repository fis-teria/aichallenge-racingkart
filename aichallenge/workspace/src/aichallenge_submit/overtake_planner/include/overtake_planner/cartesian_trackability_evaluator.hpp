#pragma once

#include "overtake_planner/frenet_frame.hpp"
#include "overtake_planner/types.hpp"

#include <cstddef>
#include <limits>
#include <string>

namespace overtake_planner {

struct CartesianTrackabilityConfig {
  double wheelbase_m{1.087};
  double steering_gain{1.639};
  double max_steering_angle_rad{kVehicleHardSteeringTireAngleRad};
  double max_steering_rate_radps{kVehicleHardSteeringRateRadps};
  double max_yaw_tangent_error_rad{0.35};
  // Cartesian arc / candidate time と predicted_speed の差に許す明示幅。
  // 絶対幅 + 相対幅 * max(execution, prediction) だけを数値積分誤差として
  // 許し、それを超えるgeometry/time/speedの組合せはfail-closedにする。
  double speed_consistency_abs_tolerance_mps{0.25};
  double speed_consistency_relative_tolerance{0.10};
  // predicted_speedは応答遅れ中にv_refを上回り得るため、単純な
  // predicted<=v_refではなく、初期実行予測と各点v_refの包絡からこの幅だけ
  // 外れることを許す。
  double speed_reference_envelope_tolerance_mps{0.05};
  double pure_pursuit_required_arc_m{3.5};
  double target_d_m{std::numeric_limits<double>::quiet_NaN()};
  double target_d_deadline_arc_m{std::numeric_limits<double>::quiet_NaN()};
  // target到達後のprojectionノイズと終端収束に許す明示幅。最初の到達以降に
  // 元の側へこの幅を超えてcross-backした候補は拒否する。
  double target_d_tolerance_m{0.05};
};

struct CartesianTrackabilityInput {
  // Candidate先頭と同じsnapshotのbase_link中心姿勢。rear axleはactive PPと
  // 同じく wheelbase / 2 だけ後方へ戻して求める。
  EgoState ego{};
  // 曲率adaptive/smoothingを適用し終えたactive PPの実lookahead snapshot。
  // gain/minからPlanner側で推測した値はexact command proofに使わない。
  bool active_lookahead_valid{false};
  double active_lookahead_distance_m{std::numeric_limits<double>::quiet_NaN()};
  // active PPがoverride適用後trajectory上で確定したexact nearest index。
  // Planner側のego距離再探索は同じtrajectory snapshotを証明しないため代用不可。
  bool nearest_source_index_valid{false};
  std::size_t nearest_source_index{0U};
  // active PPの曲率window/signed curvatureから算出・clamp済みの加算項。
  bool curvature_feedforward_valid{false};
  double curvature_feedforward_steering_rad{
      std::numeric_limits<double>::quiet_NaN()};
  // active PPがangle/rate boundのreferenceに実際に使うfresh steering値。
  // current statusもlast commandも得られない場合はvalidを立てない。
  bool steering_reference_valid{false};
  double steering_reference_angle_rad{std::numeric_limits<double>::quiet_NaN()};
  double steering_command_dt_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct CartesianTrackabilityResult {
  bool valid{false};
  bool desired_path_trackable{false};
  bool pure_pursuit_command_evaluated{false};
  bool pure_pursuit_command_trackable{false};
  bool trackable{false};
  std::string reason{"not_evaluated"};
  std::string desired_path_reason{"not_evaluated"};
  std::string pure_pursuit_command_reason{"not_evaluated"};
  std::size_t resampled_point_count{0U};
  std::size_t pure_pursuit_target_source_index{0U};
  double total_arc_m{std::numeric_limits<double>::quiet_NaN()};
  double max_abs_curvature_m_inv{std::numeric_limits<double>::quiet_NaN()};
  double max_abs_steering_angle_rad{std::numeric_limits<double>::quiet_NaN()};
  double max_abs_steering_rate_radps{std::numeric_limits<double>::quiet_NaN()};
  double max_abs_speed_consistency_error_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double target_d_reach_arc_m{std::numeric_limits<double>::quiet_NaN()};
  double terminal_projected_d_m{std::numeric_limits<double>::quiet_NaN()};
  double pure_pursuit_lookahead_distance_m{
      std::numeric_limits<double>::quiet_NaN()};
  double pure_pursuit_raw_steering_angle_rad{
      std::numeric_limits<double>::quiet_NaN()};
  double pure_pursuit_geometric_steering_angle_rad{
      std::numeric_limits<double>::quiet_NaN()};
  double curvature_feedforward_steering_angle_rad{
      std::numeric_limits<double>::quiet_NaN()};
  double pure_pursuit_requested_steering_angle_rad{
      std::numeric_limits<double>::quiet_NaN()};
  double pure_pursuit_bounded_steering_angle_rad{
      std::numeric_limits<double>::quiet_NaN()};
  double pure_pursuit_requested_steering_rate_radps{
      std::numeric_limits<double>::quiet_NaN()};
  double pure_pursuit_bounded_steering_rate_radps{
      std::numeric_limits<double>::quiet_NaN()};
  bool pure_pursuit_angle_limited{false};
  bool pure_pursuit_rate_limited{false};
};

// CandidateTrajectoryの実Cartesian列だけを最大5 cm間隔へ再サンプルし、
// controllerの操舵角・候補時刻に対する操舵速度・PP必要arcを検査し、
// Frenet再投影dの最初の交差を実Cartesian arc上へ補間してdeadline判定する
// pure層。入力異常や上限超過は補間点の切捨て、終端fallback、閾値clampを
// 行わず必ずfail-closedにする。
class CartesianTrackabilityEvaluator {
public:
  static constexpr double kMaxResampleSpacingM = 0.05;
  static constexpr std::size_t kMaxPointCount = 2000U;

  explicit CartesianTrackabilityEvaluator(const FrenetFrame &frame)
      : frame_(frame) {}

  CartesianTrackabilityResult
  evaluate(const CandidateTrajectory &candidate,
           const CartesianTrackabilityInput &input,
           const CartesianTrackabilityConfig &config) const;

private:
  const FrenetFrame &frame_;
};

} // namespace overtake_planner
