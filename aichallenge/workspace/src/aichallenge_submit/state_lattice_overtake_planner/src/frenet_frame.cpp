#include "state_lattice_overtake_planner/frenet_frame.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace state_lattice_overtake_planner {
namespace {
std::vector<std::string> split(const std::string &line) {
  std::stringstream stream(line);
  std::vector<std::string> cells;
  std::string cell;
  while (std::getline(stream, cell, ',')) {
    cells.push_back(cell);
  }
  return cells;
}

double cell(const std::vector<std::string> &row,
            const std::unordered_map<std::string, std::size_t> &header,
            const std::string &name, double fallback) {
  const auto it = header.find(name);
  if (it == header.end() || it->second >= row.size() ||
      row[it->second].empty()) {
    return fallback;
  }
  try {
    return std::stod(row[it->second]);
  } catch (...) {
    return fallback;
  }
}

double unwrapNear(double wrapped_s, double expected_s, double length) {
  if (length <= 0.0) {
    return wrapped_s;
  }
  return wrapped_s + std::round((expected_s - wrapped_s) / length) * length;
}
} // namespace

bool FrenetFrame::loadCsv(const std::string &path, std::string *error) {
  std::ifstream input(path);
  std::string line;
  if (!input || !std::getline(input, line)) {
    if (error != nullptr) {
      *error = "failed to read reference csv: " + path;
    }
    return false;
  }
  std::unordered_map<std::string, std::size_t> header;
  const auto names = split(line);
  for (std::size_t i = 0; i < names.size(); ++i) {
    header[names[i]] = i;
  }
  std::vector<ReferencePoint> points;
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    const auto row = split(line);
    ReferencePoint point;
    point.s =
        cell(row, header, "s_m", std::numeric_limits<double>::quiet_NaN());
    point.x =
        cell(row, header, "x_m", std::numeric_limits<double>::quiet_NaN());
    point.y =
        cell(row, header, "y_m", std::numeric_limits<double>::quiet_NaN());
    point.yaw =
        cell(row, header, "psi_rad", std::numeric_limits<double>::quiet_NaN());
    point.kappa = cell(row, header, "kappa_radpm", 0.0);
    point.speed_mps = cell(row, header, "vx_mps", 0.0);
    points.push_back(point);
  }
  return setReference(std::move(points), error);
}

bool FrenetFrame::setReference(std::vector<ReferencePoint> points,
                               std::string *error) {
  if (points.size() < 3U) {
    if (error != nullptr) {
      *error = "reference requires at least three points";
    }
    return false;
  }
  double accumulated = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y) ||
        !std::isfinite(points[i].yaw) || !std::isfinite(points[i].kappa) ||
        !std::isfinite(points[i].speed_mps) || points[i].speed_mps < 0.0) {
      if (error != nullptr) {
        *error = "reference contains non-finite or invalid motion data";
      }
      return false;
    }
    if (i > 0U) {
      accumulated += std::hypot(points[i].x - points[i - 1U].x,
                                points[i].y - points[i - 1U].y);
    }
    if (!std::isfinite(points[i].s)) {
      points[i].s = accumulated;
    }
    if (i > 0U && points[i].s <= points[i - 1U].s) {
      if (error != nullptr) {
        *error = "reference s must be strictly increasing";
      }
      return false;
    }
  }
  const double closure = std::hypot(points.front().x - points.back().x,
                                    points.front().y - points.back().y);
  length_m_ = points.back().s + closure;
  if (!std::isfinite(length_m_) || length_m_ <= points.back().s) {
    if (error != nullptr) {
      *error = "reference closure is invalid";
    }
    return false;
  }
  points_ = std::move(points);
  return true;
}

double FrenetFrame::wrapS(double s) const {
  if (length_m_ <= 0.0) {
    return s;
  }
  s = std::fmod(s, length_m_);
  return s < 0.0 ? s + length_m_ : s;
}

double FrenetFrame::forwardDeltaS(double from_s, double to_s) const {
  double delta = wrapS(to_s) - wrapS(from_s);
  return delta < 0.0 ? delta + length_m_ : delta;
}

ReferencePoint FrenetFrame::interpolate(double s) const {
  if (empty()) {
    return {};
  }
  const double wrapped = wrapS(s);
  auto upper = std::upper_bound(points_.begin(), points_.end(), wrapped,
                                [](double value, const ReferencePoint &point) {
                                  return value < point.s;
                                });
  const ReferencePoint *a = nullptr;
  const ReferencePoint *b = nullptr;
  double a_s = 0.0;
  double b_s = 0.0;
  if (upper == points_.begin() || upper == points_.end()) {
    a = &points_.back();
    b = &points_.front();
    a_s = points_.back().s;
    b_s = length_m_;
  } else {
    a = &*(upper - 1);
    b = &*upper;
    a_s = a->s;
    b_s = b->s;
  }
  double query = wrapped;
  if (query < a_s) {
    query += length_m_;
  }
  const double ratio =
      std::clamp((query - a_s) / std::max(1.0e-9, b_s - a_s), 0.0, 1.0);
  ReferencePoint result;
  result.s = wrapped;
  result.x = a->x + ratio * (b->x - a->x);
  result.y = a->y + ratio * (b->y - a->y);
  result.yaw = normalizeAngle(a->yaw + ratio * normalizeAngle(b->yaw - a->yaw));
  result.kappa = a->kappa + ratio * (b->kappa - a->kappa);
  result.speed_mps = a->speed_mps + ratio * (b->speed_mps - a->speed_mps);
  return result;
}

ReferencePoint FrenetFrame::frenetToCartesian(double s, double d) const {
  auto point = interpolate(s);
  point.x -= std::sin(point.yaw) * d;
  point.y += std::cos(point.yaw) * d;
  point.s = s;
  return point;
}

FrenetPoint FrenetFrame::project(double x, double y, double yaw) const {
  return projectInWindow(x, y, yaw, 0.0, 0.0, true);
}

FrenetPoint FrenetFrame::projectContinuous(double x, double y, double yaw,
                                           double expected_s,
                                           double half_width) const {
  return projectInWindow(x, y, yaw, expected_s, half_width, false);
}

FrenetPoint FrenetFrame::projectInWindow(double x, double y, double yaw,
                                         double expected_s, double half_width,
                                         bool unrestricted) const {
  FrenetPoint best;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  double best_expected_delta = std::numeric_limits<double>::infinity();
  if (empty() || !std::isfinite(x) || !std::isfinite(y)) {
    return best;
  }
  const auto evaluate_segment = [&](std::size_t i) {
    const auto &a = points_[i];
    const auto &b = points_[(i + 1U) % points_.size()];
    const double segment_s =
        i + 1U < points_.size() ? b.s - a.s : length_m_ - a.s;
    if (!unrestricted) {
      const double unwrapped_a = unwrapNear(a.s, expected_s, length_m_);
      const double unwrapped_b = unwrapped_a + segment_s;
      if (unwrapped_b < expected_s - half_width - 1.0e-9 ||
          unwrapped_a > expected_s + half_width + 1.0e-9) {
        return;
      }
    }
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double denom = vx * vx + vy * vy;
    if (denom <= 1.0e-12) {
      return;
    }
    const double t =
        std::clamp(((x - a.x) * vx + (y - a.y) * vy) / denom, 0.0, 1.0);
    const double px = a.x + t * vx;
    const double py = a.y + t * vy;
    const double wrapped_s = wrapS(a.s + t * segment_s);
    const double unwrapped_s =
        unrestricted ? wrapped_s : unwrapNear(wrapped_s, expected_s, length_m_);
    const double expected_delta = std::abs(unwrapped_s - expected_s);
    if (!unrestricted && expected_delta > half_width + 1.0e-9) {
      return;
    }
    const double distance_sq = (x - px) * (x - px) + (y - py) * (y - py);
    if (distance_sq + 1.0e-10 < best_distance_sq ||
        (std::abs(distance_sq - best_distance_sq) <= 1.0e-10 &&
         expected_delta < best_expected_delta)) {
      const double segment_yaw = std::atan2(vy, vx);
      best.s = unwrapped_s;
      best.d =
          -std::sin(segment_yaw) * (x - px) + std::cos(segment_yaw) * (y - py);
      best.yaw_error = normalizeAngle(yaw - segment_yaw);
      best.segment_index = i;
      best.valid = true;
      best_distance_sq = distance_sq;
      best_expected_delta = expected_delta;
    }
  };

  const auto evaluate_all_segments = [&]() {
    for (std::size_t i = 0U; i < points_.size(); ++i) {
      evaluate_segment(i);
    }
  };

  // Unrestricted projection and unusual windows keep the legacy exhaustive
  // order. Normal continuous projection uses the sorted reference s values to
  // evaluate only the contiguous segment interval intersecting the requested
  // window. Wrapped intervals are still visited in ascending physical segment
  // index so distance ties resolve exactly as before.
  if (unrestricted || !std::isfinite(expected_s) ||
      !std::isfinite(half_width) || half_width < 0.0 ||
      2.0 * half_width >= length_m_ - 2.0e-9) {
    evaluate_all_segments();
    return best;
  }

  constexpr double kWindowEpsilon = 1.0e-9;
  const double lower_s = expected_s - half_width - kWindowEpsilon;
  const double upper_s = expected_s + half_width + kWindowEpsilon;
  const double wrapped_lower = wrapS(lower_s);
  const auto upper =
      std::upper_bound(points_.begin(), points_.end(), wrapped_lower,
                       [](double value, const ReferencePoint &point) {
                         return value < point.s;
                       });
  std::size_t start_index =
      upper == points_.begin()
          ? points_.size() - 1U
          : static_cast<std::size_t>(std::distance(points_.begin(), upper) - 1);
  double segment_start_s =
      unwrapNear(points_[start_index].s, expected_s, length_m_);
  std::size_t first_index = start_index;
  std::size_t selected_count = 0U;
  for (std::size_t visited = 0U; visited < points_.size(); ++visited) {
    const std::size_t index = (start_index + visited) % points_.size();
    const auto &a = points_[index];
    const auto &b = points_[(index + 1U) % points_.size()];
    const double segment_length_s =
        index + 1U < points_.size() ? b.s - a.s : length_m_ - a.s;
    const double segment_end_s = segment_start_s + segment_length_s;
    if (segment_end_s >= lower_s && segment_start_s <= upper_s) {
      if (selected_count == 0U) {
        first_index = index;
      }
      ++selected_count;
    } else if (selected_count > 0U && segment_start_s > upper_s) {
      break;
    }
    segment_start_s = segment_end_s;
  }

  if (selected_count == 0U || selected_count > points_.size()) {
    evaluate_all_segments();
    return best;
  }
  if (first_index + selected_count <= points_.size()) {
    for (std::size_t i = first_index; i < first_index + selected_count; ++i) {
      evaluate_segment(i);
    }
  } else {
    const std::size_t wrapped_end =
        (first_index + selected_count) % points_.size();
    for (std::size_t i = 0U; i < wrapped_end; ++i) {
      evaluate_segment(i);
    }
    for (std::size_t i = first_index; i < points_.size(); ++i) {
      evaluate_segment(i);
    }
  }
  return best;
}

std::size_t FrenetFrame::nearestIndex(double s) const {
  const double wrapped = wrapS(s);
  std::size_t best = 0U;
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < points_.size(); ++i) {
    const double delta = std::min(forwardDeltaS(wrapped, points_[i].s),
                                  forwardDeltaS(points_[i].s, wrapped));
    if (delta < distance) {
      distance = delta;
      best = i;
    }
  }
  return best;
}

double FrenetFrame::unwrappedIndexS(long long index, std::size_t anchor_index,
                                    double anchor_s) const {
  const long long count = static_cast<long long>(points_.size());
  const long long laps =
      index >= 0 ? index / count : -((-index + count - 1) / count);
  long long wrapped_index = index - laps * count;
  if (wrapped_index < 0) {
    wrapped_index += count;
  }
  const double anchor_wrapped = points_[anchor_index].s;
  double value = points_[static_cast<std::size_t>(wrapped_index)].s +
                 static_cast<double>(laps) * length_m_;
  value += anchor_s - unwrapNear(anchor_wrapped, anchor_s, length_m_);
  return value;
}

} // namespace state_lattice_overtake_planner
