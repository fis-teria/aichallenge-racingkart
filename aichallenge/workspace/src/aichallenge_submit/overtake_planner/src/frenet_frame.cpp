#include "overtake_planner/frenet_frame.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace overtake_planner {

namespace {

// 入力: CSVの1行。
// 出力: カンマ区切りで分割した文字列配列。
// 処理概要: 参照CSVを軽量に読むため、引用符処理を持たない単純分割を行う。
std::vector<std::string> splitCsvLine(const std::string &line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    out.push_back(cell);
  }
  return out;
}

// 入力: CSV行、ヘッダ名から列番号へのmap、読みたい列名、fallback値。
// 出力: 指定列をdouble化した値。列が無い/空ならfallback。
// 処理概要: 参照CSVの任意列を安全に読むため、列存在チェックを共通化する。
double readCell(const std::vector<std::string> &row,
                const std::unordered_map<std::string, std::size_t> &header,
                const std::string &name, double fallback = 0.0) {
  const auto it = header.find(name);
  if (it == header.end() || it->second >= row.size() ||
      row[it->second].empty()) {
    return fallback;
  }
  return std::stod(row[it->second]);
}

} // namespace

// 入力: BehaviorMode enum。
// 出力: debug JSONやログへ出す固定文字列。
// 処理概要: modeの可読化を1箇所に集約し、ログとテストの表記揺れを避ける。
const char *toString(BehaviorMode mode) {
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
  case BehaviorMode::SPEED_GUARD:
    return "SPEED_GUARD";
  }
  return "UNKNOWN";
}

// 入力: CandidateType enum。
// 出力: debug JSONやログへ出す固定文字列。
// 処理概要: 候補種別の可読化を1箇所に集約する。
const char *toString(CandidateType type) {
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

// 入力: 現在のBehaviorMode。
// 出力: 追い越し準備/実行中ならtrue。
// 処理概要: PREPAREとOVERTAKEをまとめて「追い越し文脈」として扱う。
bool isPassMode(BehaviorMode mode) {
  return mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
         mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
         mode == BehaviorMode::OVERTAKE_LEFT ||
         mode == BehaviorMode::OVERTAKE_RIGHT;
}

// 入力: 任意の角度[rad]。
// 出力: (-pi, pi]へ正規化した角度[rad]。
// 処理概要: yaw差分や補間結果を連続的に扱いやすい範囲へ折り返す。
double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

// 入力: 参照CSVパスと、失敗時に理由を書き込む任意のerrorポインタ。
// 出力: 読み込み成功ならtrue。失敗時はfalse。
// 処理概要: MPC参照CSVからFrenetFrameの中心線を構築する。
bool FrenetFrame::loadCsv(const std::string &path, std::string *error) {
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

  // 処理ブロック: ヘッダ行を列名mapへ変換する。
  // 設計意図: CSV列順が変わっても、必要な名前の列だけを読めるようにする。
  const auto names = splitCsvLine(line);
  std::unordered_map<std::string, std::size_t> header;
  for (std::size_t i = 0; i < names.size(); ++i) {
    header[names[i]] = i;
  }

  // 処理ブロック: 各行をReferencePointへ変換する。
  // 設計意図: 参照線のs/x/y/yaw/kappa/vを同じ構造体にまとめ、候補生成で再利用する。
  std::vector<ReferencePoint> ref;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    const auto row = splitCsvLine(line);
    ReferencePoint p;
    p.s =
        readCell(row, header, "s_m", std::numeric_limits<double>::quiet_NaN());
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

// 入力: 路面境界をFrenet s/dで記録したCSVと、失敗時の理由を書き込む任意のerror。
// 出力: 参照CSVと同じサンプル列ならtrue。対応しないprofileはfalse。
// 処理概要: 参照線と異なるコースの回廊を安全制約へ誤適用しないよう、s列を照合して読む。
bool FrenetFrame::loadCorridorCsv(const std::string &path, std::string *error) {
  if (reference_.size() < 2) {
    if (error != nullptr) {
      *error = "cannot load corridor before reference csv";
    }
    return false;
  }

  std::ifstream ifs(path);
  if (!ifs) {
    if (error != nullptr) {
      *error = "failed to open corridor csv: " + path;
    }
    return false;
  }

  std::string line;
  if (!std::getline(ifs, line)) {
    if (error != nullptr) {
      *error = "empty corridor csv: " + path;
    }
    return false;
  }
  const auto names = splitCsvLine(line);
  std::unordered_map<std::string, std::size_t> header;
  for (std::size_t i = 0; i < names.size(); ++i) {
    header[names[i]] = i;
  }
  if (header.find("s_m") == header.end() ||
      header.find("d_min_m") == header.end() ||
      header.find("d_max_m") == header.end()) {
    if (error != nullptr) {
      *error = "corridor csv must contain s_m,d_min_m,d_max_m: " + path;
    }
    return false;
  }

  std::vector<FrenetCorridorPoint> corridor;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    const auto row = splitCsvLine(line);
    FrenetCorridorPoint point;
    point.s = readCell(row, header, "s_m",
                       std::numeric_limits<double>::quiet_NaN());
    point.d_min = readCell(row, header, "d_min_m",
                           std::numeric_limits<double>::quiet_NaN());
    point.d_max = readCell(row, header, "d_max_m",
                           std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(point.s) || !std::isfinite(point.d_min) ||
        !std::isfinite(point.d_max) || point.d_min >= point.d_max) {
      if (error != nullptr) {
        *error = "invalid corridor row in: " + path;
      }
      return false;
    }
    corridor.push_back(point);
  }

  if (corridor.size() != reference_.size()) {
    if (error != nullptr) {
      *error = "corridor/reference point count mismatch: " + path;
    }
    return false;
  }
  constexpr double kSMatchToleranceM = 1.0e-3;
  for (std::size_t i = 0; i < corridor.size(); ++i) {
    if (i > 0 && corridor[i].s <= corridor[i - 1].s) {
      if (error != nullptr) {
        *error = "corridor s_m must be strictly increasing: " + path;
      }
      return false;
    }
    if (std::abs(corridor[i].s - reference_[i].s) > kSMatchToleranceM) {
      if (error != nullptr) {
        *error = "corridor s_m does not match reference csv: " + path;
      }
      return false;
    }
    if (i > 0 &&
        std::max(corridor[i - 1].d_min, corridor[i].d_min) >=
            std::min(corridor[i - 1].d_max, corridor[i].d_max)) {
      if (error != nullptr) {
        *error = "adjacent corridor bounds do not overlap: " + path;
      }
      return false;
    }
  }
  if (std::max(corridor.back().d_min, corridor.front().d_min) >=
      std::min(corridor.back().d_max, corridor.front().d_max)) {
    if (error != nullptr) {
      *error = "closing corridor bounds do not overlap: " + path;
    }
    return false;
  }
  setCorridor(std::move(corridor));
  return true;
}

// 入力: 参照点列。sがNaNの点を含んでいてもよい。
// 出力: 内部reference_とtrack_length_を更新する。
// 処理概要: 欠損sを距離累積で補い、閉ループコースとして全長を計算する。
void FrenetFrame::setReference(std::vector<ReferencePoint> reference) {
  // CSVにs_mが無い/NaNの点は、隣接点距離から累積sを補完する。
  reference_ = std::move(reference);
  double s = 0.0;
  for (std::size_t i = 0; i < reference_.size(); ++i) {
    if (i > 0) {
      const auto &prev = reference_[i - 1];
      const auto &cur = reference_[i];
      s += std::hypot(cur.x - prev.x, cur.y - prev.y);
    }
    if (!std::isfinite(reference_[i].s)) {
      reference_[i].s = s;
    }
  }
  if (reference_.size() > 1) {
    // コースは閉ループとして扱い、最後の点から先頭点へ戻る距離も含める。
    const auto &first = reference_.front();
    const auto &last = reference_.back();
    track_length_ =
        reference_.back().s + std::hypot(first.x - last.x, first.y - last.y);
  } else {
    track_length_ = 0.0;
  }
  // 参照を更新したら、以前のコースに由来する回廊を残さない。
  corridor_.clear();
}

// 入力: 参照線と同じs列の路面境界プロファイル。
// 出力: 内部回廊を更新する。
// 処理概要: テストと起動時ロードで同じs依存境界を使えるよう、参照線とは別に保持する。
void FrenetFrame::setCorridor(std::vector<FrenetCorridorPoint> corridor) {
  corridor_ = std::move(corridor);
}

// 入力: 任意のs座標[m]。
// 出力: コース長で折り返したs座標[m]。
// 処理概要: 周回コースで負値や1周超えを同じ基準へ正規化する。
double FrenetFrame::wrapS(double s) const {
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

// 入力: 始点from_sと目標to_s。
// 出力: 周回を考慮した前方距離[m]。
// 処理概要: to_sが次周に回った場合も、負距離ではなく前方距離として扱う。
double FrenetFrame::deltaS(double from_s, double to_s) const {
  // to_sが次周にある場合も、前方距離として正の値を返す。
  double delta = wrapS(to_s) - wrapS(from_s);
  if (delta < 0.0) {
    delta += track_length_;
  }
  return delta;
}

// 入力: map座標系のx,y,yaw。
// 出力: 最近傍参照点に基づくFrenetPose。
// 処理概要: 最近傍参照点を探し、参照接線に対する横ずれdとyaw誤差を計算する。
FrenetPose FrenetFrame::cartesianToFrenet(double x, double y,
                                          double yaw) const {
  // 最近傍「点」ではなく全線分へ射影する。点への量子化は約1 mごとのs飛びを
  // 作り、相手とのdelta_s、curve gate、局所回避の開始位置を不連続にしていた。
  FrenetPose pose;
  if (reference_.size() < 2 || track_length_ <= 0.0) {
    return pose;
  }
  double best_dist_sq = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < reference_.size(); ++i) {
    const std::size_t next = (i + 1) % reference_.size();
    const auto &a = reference_[i];
    const auto &b = reference_[next];
    const double segment_x = b.x - a.x;
    const double segment_y = b.y - a.y;
    const double segment_length_sq =
        segment_x * segment_x + segment_y * segment_y;
    if (segment_length_sq <= 1.0e-12) {
      continue;
    }
    const double projection_ratio = std::clamp(
        ((x - a.x) * segment_x + (y - a.y) * segment_y) /
            segment_length_sq,
        0.0, 1.0);
    const double projected_x = a.x + projection_ratio * segment_x;
    const double projected_y = a.y + projection_ratio * segment_y;
    const double dx = x - projected_x;
    const double dy = y - projected_y;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      const double segment_s =
          next == 0 ? track_length_ - a.s : b.s - a.s;
      pose.s = wrapS(a.s + projection_ratio * std::max(0.0, segment_s));
      pose.index = i;
    }
  }

  const auto ref = interpolate(pose.s);
  const double dx = x - ref.x;
  const double dy = y - ref.y;
  pose.d = std::cos(ref.yaw) * dy - std::sin(ref.yaw) * dx;
  pose.yaw_error = normalizeAngle(yaw - ref.yaw);
  return pose;
}

// 入力: 参照線上のs座標[m]。
// 出力: sを囲む参照点を線形補間したReferencePoint。
// 処理概要: 候補horizonの任意sで中心線姿勢と曲率を取得できるようにする。
ReferencePoint FrenetFrame::interpolate(double s) const {
  // 指定sを囲む2点を線形補間し、候補軌道を滑らかに生成できるようにする。
  if (reference_.empty()) {
    return {};
  }
  const double wrapped = wrapS(s);
  auto upper = std::upper_bound(
      reference_.begin(), reference_.end(), wrapped,
      [](double value, const ReferencePoint &p) { return value < p.s; });
  if (upper == reference_.begin()) {
    return reference_.front();
  }
  if (upper == reference_.end()) {
    // wrapSはlast.sより後の閉路seamも返す。最後の点で固定すると、seam上の
    // candidate x/y/yawが停止し、Frenetへの再投影と追い越し候補が崩れる。
    const auto &a = reference_.back();
    const auto &b = reference_.front();
    const double denom = std::max(1.0e-6, track_length_ - a.s);
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
  const auto &b = *upper;
  const auto &a = *(upper - 1);
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

// 入力: Frenet座標s,d。
// 出力: 中心線から法線方向にdだけずらしたCartesianのReferencePoint。
// 処理概要: MPCへ渡す横オフセット付き参照点をmap座標へ戻す。
ReferencePoint FrenetFrame::frenetToCartesian(double s, double d) const {
  // 中心線上の点から法線方向にdだけずらして、MPCへ渡すCartesian点へ戻す。
  ReferencePoint out = interpolate(s);
  out.x = out.x - d * std::sin(out.yaw);
  out.y = out.y + d * std::cos(out.yaw);
  return out;
}

// 入力: 評価したいsと、profile未ロード時の固定境界。
// 出力: そのsで使える物理境界。profileがあれば隣接sampleの狭い側を返す。
// 処理概要: sample間を線形に広げると実境界を越える可能性があるため、各線分で
// 左境界は小さい方、右境界は大きい方を採用してfail-closedにする。
FrenetCorridorBounds FrenetFrame::corridorBounds(
    double s, double fallback_d_min, double fallback_d_max) const {
  FrenetCorridorBounds out{fallback_d_min, fallback_d_max};
  if (corridor_.size() < 2 || track_length_ <= 0.0) {
    return out;
  }
  const double wrapped = wrapS(s);
  auto upper = std::upper_bound(
      corridor_.begin(), corridor_.end(), wrapped,
      [](double value, const FrenetCorridorPoint &point) {
        return value < point.s;
      });
  std::size_t lower_index = 0;
  std::size_t upper_index = 0;
  if (upper == corridor_.begin()) {
    lower_index = 0;
    upper_index = 1;
  } else if (upper == corridor_.end()) {
    lower_index = corridor_.size() - 1;
    upper_index = 0;
  } else {
    lower_index = static_cast<std::size_t>(upper - corridor_.begin() - 1);
    upper_index = static_cast<std::size_t>(upper - corridor_.begin());
  }
  const auto &a = corridor_[lower_index];
  const auto &b = corridor_[upper_index];
  out.d_min = std::max(a.d_min, b.d_min);
  out.d_max = std::min(a.d_max, b.d_max);
  if (!std::isfinite(out.d_min) || !std::isfinite(out.d_max) ||
      out.d_min >= out.d_max) {
    return FrenetCorridorBounds{fallback_d_min, fallback_d_max};
  }
  return out;
}

} // namespace overtake_planner
