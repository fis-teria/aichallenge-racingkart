#include "overtake_planner/frenet_frame.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace overtake_planner
{

namespace
{

std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    out.push_back(cell);
  }
  return out;
}

double readCell(
  const std::vector<std::string> & row,
  const std::unordered_map<std::string, std::size_t> & header,
  const std::string & name,
  double fallback = 0.0)
{
  const auto it = header.find(name);
  if (it == header.end() || it->second >= row.size() || row[it->second].empty()) {
    return fallback;
  }
  return std::stod(row[it->second]);
}

}  // namespace

const char * toString(BehaviorMode mode)
{
  switch (mode) {
    case BehaviorMode::FREE_RUN:
      return "FREE_RUN";
    case BehaviorMode::FOLLOW_BLOCKED:
      return "FOLLOW_BLOCKED";
    case BehaviorMode::PREPARE_OVERTAKE_LEFT:
      return "PREPARE_OVERTAKE_LEFT";
    case BehaviorMode::PREPARE_OVERTAKE_RIGHT:
      return "PREPARE_OVERTAKE_RIGHT";
    case BehaviorMode::OVERTAKE_LEFT:
      return "OVERTAKE_LEFT";
    case BehaviorMode::OVERTAKE_RIGHT:
      return "OVERTAKE_RIGHT";
    case BehaviorMode::MERGE_BACK:
      return "MERGE_BACK";
    case BehaviorMode::ABORT_RECOVERY:
      return "ABORT_RECOVERY";
    case BehaviorMode::SIDE_BY_SIDE_KEEP:
      return "SIDE_BY_SIDE_KEEP";
    case BehaviorMode::YIELD_BEHIND:
      return "YIELD_BEHIND";
    case BehaviorMode::SAFE_STOP:
      return "SAFE_STOP";
  }
  return "UNKNOWN";
}

const char * toString(CandidateType type)
{
  switch (type) {
    case CandidateType::FASTEST:
      return "FASTEST";
    case CandidateType::FOLLOW:
      return "FOLLOW";
    case CandidateType::PASS_LEFT:
      return "PASS_LEFT";
    case CandidateType::PASS_RIGHT:
      return "PASS_RIGHT";
    case CandidateType::RECOVERY:
      return "RECOVERY";
    case CandidateType::SIDE_BY_SIDE_KEEP:
      return "SIDE_BY_SIDE_KEEP";
    case CandidateType::YIELD_BEHIND:
      return "YIELD_BEHIND";
    case CandidateType::SAFE_STOP:
      return "SAFE_STOP";
  }
  return "UNKNOWN";
}

bool isPassMode(BehaviorMode mode)
{
  return mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
         mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
         mode == BehaviorMode::OVERTAKE_LEFT ||
         mode == BehaviorMode::OVERTAKE_RIGHT;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

bool FrenetFrame::loadCsv(const std::string & path, std::string * error)
{
  // MPCと同じ参照CSVを読み、追い越し判断で使う中心線を構築する。
  std::ifstream ifs(path);
  if (!ifs) {
    if (error != nullptr) {
      *error = "failed to open reference csv: " + path;
    }
    return false;
  }

  std::string line;
  if (!std::getline(ifs, line)) {
    if (error != nullptr) {
      *error = "empty reference csv: " + path;
    }
    return false;
  }

  const auto names = splitCsvLine(line);
  std::unordered_map<std::string, std::size_t> header;
  for (std::size_t i = 0; i < names.size(); ++i) {
    header[names[i]] = i;
  }

  std::vector<ReferencePoint> ref;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    const auto row = splitCsvLine(line);
    ReferencePoint p;
    p.s = readCell(row, header, "s_m", std::numeric_limits<double>::quiet_NaN());
    p.x = readCell(row, header, "x_m");
    p.y = readCell(row, header, "y_m");
    p.yaw = readCell(row, header, "psi_rad");
    p.kappa = readCell(row, header, "kappa_radpm");
    p.v_ref = readCell(row, header, "vx_mps", 0.0);
    ref.push_back(p);
  }

  if (ref.size() < 2) {
    if (error != nullptr) {
      *error = "reference csv has fewer than two points: " + path;
    }
    return false;
  }
  setReference(std::move(ref));
  return true;
}

void FrenetFrame::setReference(std::vector<ReferencePoint> reference)
{
  // CSVにs_mが無い/NaNの点は、隣接点距離から累積sを補完する。
  reference_ = std::move(reference);
  double s = 0.0;
  for (std::size_t i = 0; i < reference_.size(); ++i) {
    if (i > 0) {
      const auto & prev = reference_[i - 1];
      const auto & cur = reference_[i];
      s += std::hypot(cur.x - prev.x, cur.y - prev.y);
    }
    if (!std::isfinite(reference_[i].s)) {
      reference_[i].s = s;
    }
  }
  if (reference_.size() > 1) {
    // コースは閉ループとして扱い、最後の点から先頭点へ戻る距離も含める。
    const auto & first = reference_.front();
    const auto & last = reference_.back();
    track_length_ = reference_.back().s + std::hypot(first.x - last.x, first.y - last.y);
  } else {
    track_length_ = 0.0;
  }
}

double FrenetFrame::wrapS(double s) const
{
  // 参照線の長さでsを折り返し、周回コース上の位置として扱う。
  if (track_length_ <= 0.0) {
    return s;
  }
  s = std::fmod(s, track_length_);
  if (s < 0.0) {
    s += track_length_;
  }
  return s;
}

double FrenetFrame::deltaS(double from_s, double to_s) const
{
  // to_sが次周にある場合も、前方距離として正の値を返す。
  double delta = wrapS(to_s) - wrapS(from_s);
  if (delta < 0.0) {
    delta += track_length_;
  }
  return delta;
}

FrenetPose FrenetFrame::cartesianToFrenet(double x, double y, double yaw) const
{
  // 最近傍の参照点を探し、その接線方向に対する横ずれdを計算する簡易変換。
  FrenetPose pose;
  if (reference_.empty()) {
    return pose;
  }
  double best_dist_sq = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < reference_.size(); ++i) {
    const double dx = x - reference_[i].x;
    const double dy = y - reference_[i].y;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      pose.index = i;
    }
  }

  const auto & ref = reference_[pose.index];
  const double dx = x - ref.x;
  const double dy = y - ref.y;
  pose.s = ref.s;
  pose.d = std::cos(ref.yaw) * dy - std::sin(ref.yaw) * dx;
  pose.yaw_error = normalizeAngle(yaw - ref.yaw);
  return pose;
}

ReferencePoint FrenetFrame::interpolate(double s) const
{
  // 指定sを囲む2点を線形補間し、候補軌道を滑らかに生成できるようにする。
  if (reference_.empty()) {
    return {};
  }
  const double wrapped = wrapS(s);
  auto upper = std::upper_bound(
    reference_.begin(), reference_.end(), wrapped,
    [](double value, const ReferencePoint & p) { return value < p.s; });
  if (upper == reference_.begin()) {
    return reference_.front();
  }
  if (upper == reference_.end()) {
    return reference_.back();
  }
  const auto & b = *upper;
  const auto & a = *(upper - 1);
  const double denom = std::max(1.0e-6, b.s - a.s);
  const double ratio = (wrapped - a.s) / denom;
  ReferencePoint out;
  out.s = wrapped;
  out.x = a.x + (b.x - a.x) * ratio;
  out.y = a.y + (b.y - a.y) * ratio;
  out.yaw = normalizeAngle(a.yaw + normalizeAngle(b.yaw - a.yaw) * ratio);
  out.kappa = a.kappa + (b.kappa - a.kappa) * ratio;
  out.v_ref = a.v_ref + (b.v_ref - a.v_ref) * ratio;
  return out;
}

ReferencePoint FrenetFrame::frenetToCartesian(double s, double d) const
{
  // 中心線上の点から法線方向にdだけずらして、MPCへ渡すCartesian点へ戻す。
  ReferencePoint out = interpolate(s);
  out.x = out.x - d * std::sin(out.yaw);
  out.y = out.y + d * std::cos(out.yaw);
  return out;
}

}  // namespace overtake_planner
