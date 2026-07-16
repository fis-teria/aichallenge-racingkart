#include "overtake_planner/blocked_risk_analyzer.hpp"

#include <algorithm>
#include <cmath>

namespace overtake_planner {

namespace {

// 入力: 現在mode。
// 出力: 左追い越し準備/実行中ならtrue。
// 処理概要: 左側のpass gap hysteresisを適用するかを判定する。
bool isLeftPassMode(BehaviorMode mode) {
  return mode == BehaviorMode::PREPARE_OVERTAKE_LEFT ||
         mode == BehaviorMode::OVERTAKE_LEFT;
}

// 入力: 現在mode。
// 出力: 右追い越し準備/実行中ならtrue。
// 処理概要: 右側のpass gap hysteresisを適用するかを判定する。
bool isRightPassMode(BehaviorMode mode) {
  return mode == BehaviorMode::PREPARE_OVERTAKE_RIGHT ||
         mode == BehaviorMode::OVERTAKE_RIGHT;
}

// 入力: 自車s、相手s、コース長。
// 出力: 周回を考慮した符号付きs差分。正は相手が前方、負は後方。
// 処理概要: 1周の半分を超える差分を逆向きに折り返し、横並び判定で前後関係を保つ。
double signedDeltaS(double ego_s, double other_s, double track_length) {
  double signed_delta_s = other_s - ego_s;
  if (track_length > 0.0) {
    signed_delta_s = std::fmod(signed_delta_s, track_length);
    if (signed_delta_s > track_length * 0.5) {
      signed_delta_s -= track_length;
    } else if (signed_delta_s < -track_length * 0.5) {
      signed_delta_s += track_length;
    }
  }
  return signed_delta_s;
}

} // namespace

// 入力: Frenet変換器とplanner設定。
// 出力: 閉塞/並走リスク解析器のインスタンス。
// 処理概要: 相手車のs/d分類とpass gap判定を、同じ参照線/設定で実行できるようにする。
BlockedRiskAnalyzer::BlockedRiskAnalyzer(const FrenetFrame &frame,
                                         const PlannerConfig &config)
    : frame_(frame), config_(config) {}

// 入力: 自車状態、相手車一覧、現在時刻。
// 出力: 前方閉塞、横並び、並走候補、同方向判定を詰めたBlockedInfo。
// 処理概要: V2Xで見えた相手車をFrenet上で分類し、plannerが候補生成に使うリスク情報へ変換する。
BlockedInfo BlockedRiskAnalyzer::detectBlocked(
    const EgoState &ego, const std::vector<OpponentState> &opponents,
    double now_sec) const {
  BlockedInfo info;
  bool nearest_front_is_side_by_side = false;
  // 処理ブロック: 各相手車を鮮度、進行方向、相対s/dで分類する。
  // 設計意図: 逆走や古い点を早めに除外し、前方閉塞と横並びを別々の状態として保持する。
  for (std::size_t i = 0; i < opponents.size(); ++i) {
    const auto &opp = opponents[i];
    if (!opp.valid ||
        now_sec - opp.stamp_sec > config_.opponent_stale_time_sec) {
      continue;
    }
    const double delta_s = frame_.deltaS(ego.frenet.s, opp.frenet.s);
    const double signed_delta_s =
        signedDeltaS(ego.frenet.s, opp.frenet.s, frame_.length());
    const double s_dot = opponentSDot(opp);
    const bool direction_known = opp.v >= config_.same_direction_min_speed_mps;
    const bool same_direction =
        !direction_known || s_dot >= config_.same_direction_min_s_dot_mps;
    if (config_.same_direction_filter_enabled && direction_known &&
        !same_direction) {
      ++info.ignored_opposite_direction_count;
      continue;
    }
    const double delta_d = opp.frenet.d - ego.frenet.d;
    const bool front = delta_s > 0.0 && delta_s < config_.lookahead_s_m;
    const bool same_corridor =
        std::abs(delta_d) < config_.same_corridor_width_m;
    const bool side_by_side =
        std::abs(signed_delta_s) < config_.side_by_side_s_m &&
        std::abs(delta_d) < config_.side_margin_m;
    const bool parallel_side_candidate =
        config_.parallel_side_detection_enabled && !side_by_side &&
        config_.parallel_side_s_m > 0.0 &&
        config_.parallel_side_margin_m > 0.0 &&
        std::abs(signed_delta_s) < config_.parallel_side_s_m &&
        std::abs(delta_d) < config_.parallel_side_margin_m;
    if (side_by_side) {
      // 処理ブロック: 現在ほぼ横にいる車両を記録する。
      // 設計意図: 前方閉塞ではなくても壁側へ押し出されるリスクがあるため、独立したフラグにする。
      info.side_by_side = true;
      if (info.side_index < 0 ||
          std::abs(signed_delta_s) < std::abs(info.side_delta_s)) {
        info.side_index = static_cast<int>(i);
        info.side_id = opp.id;
        info.side_delta_s = signed_delta_s;
        info.side_delta_d = delta_d;
        info.side_rel_v = ego.v - opp.v;
        info.side_s_dot_mps = s_dot;
        info.side_direction_known = direction_known;
        info.side_same_direction = same_direction;
      }
    }
    if (parallel_side_candidate) {
      // 処理ブロック: 少し前後にずれている並走車両を記録する。
      // 設計意図: コーナー進入前のサイドバイサイド化を早めに検出し、譲り判断へつなげる。
      info.parallel_side_candidate = true;
      if (info.parallel_side_index < 0 ||
          std::abs(signed_delta_s) < std::abs(info.parallel_side_delta_s)) {
        info.parallel_side_index = static_cast<int>(i);
        info.parallel_side_id = opp.id;
        info.parallel_side_delta_s = signed_delta_s;
        info.parallel_side_delta_d = delta_d;
        info.parallel_side_rel_v = ego.v - opp.v;
        info.parallel_side_s_dot_mps = s_dot;
        info.parallel_side_direction_known = direction_known;
        info.parallel_side_same_direction = same_direction;
      }
    }
    if (!front || !same_corridor) {
      continue;
    }
    // 横並び車両はside riskとして保持しつつ、別の通常前走車がいるなら
    // PASS/FOLLOWの対象をそちらへ固定する。横並び車を近傍前走車として
    // 優先すると、独立した前走障害物を見失うためである。
    const bool select_as_front =
        info.nearest_index < 0 ||
        (!side_by_side && nearest_front_is_side_by_side) ||
        (side_by_side == nearest_front_is_side_by_side &&
         delta_s < info.front_delta_s);
    if (select_as_front) {
      info.nearest_index = static_cast<int>(i);
      info.nearest_id = opp.id;
      info.front_delta_s = delta_s;
      info.front_delta_d = delta_d;
      info.front_rel_v = ego.v - opp.v;
      info.front_vehicle_speed_mps = opp.v;
      info.front_s_dot_mps = s_dot;
      info.front_direction_known = direction_known;
      info.front_same_direction = same_direction;
      nearest_front_is_side_by_side = side_by_side;
    }
  }

  if (info.nearest_index >= 0) {
    const bool closing = info.front_rel_v > config_.dv_block_threshold_mps;
    const bool slow_gap = info.front_delta_s < config_.follow_trigger_s_m;
    info.blocked = closing || slow_gap;
  }
  return info;
}

// 入力: 閉塞情報、相手車一覧、相手予測、現在mode。
// 出力: 左右追い越し可否とpass gap理由を更新したBlockedInfo。
// 処理概要: 対象車両の現在/予測dと壁マージンから、左右どちらに抜ける空間が残るかを評価する。
BlockedInfo BlockedRiskAnalyzer::evaluatePassGap(
    const BlockedInfo &blocked_info,
    const std::vector<OpponentState> &opponents,
    const std::vector<PredictedOpponent> &predictions,
    BehaviorMode mode) const {
  BlockedInfo out = blocked_info;
  const double ellipse_gap =
      config_.safety_ellipse_b_m * std::sqrt(1.0 + config_.min_ellipse_h);
  out.pass_gap_required_m = std::max(config_.min_pass_gap_m, ellipse_gap);

  // 処理ブロック: PASS/YIELDの対象車両を決める。
  // 設計意図: 前方車両がいればそれを優先し、いない場合は横並びリスクを基準にする。
  const int target_index = yieldTargetIndex(out);
  if (target_index < 0 ||
      static_cast<std::size_t>(target_index) >= opponents.size()) {
    out.left_pass_gap_m = 0.0;
    out.right_pass_gap_m = 0.0;
    out.can_pass_left = false;
    out.can_pass_right = false;
    out.pass_gap_reason = "no_target";
    return out;
  }

  const auto &target = opponents[static_cast<std::size_t>(target_index)];
  const auto target_bounds = frame_.corridorBounds(
      target.frenet.s, config_.d_min_m, config_.d_max_m);
  double min_left_gap =
      target_bounds.d_max - config_.min_wall_margin_m - target.frenet.d;
  double min_right_gap =
      target.frenet.d - (target_bounds.d_min + config_.min_wall_margin_m);

  // 処理ブロック: 現在位置だけでなく予測dも含めて最小隙間を見る。
  // 設計意図: 相手が将来壁側へ寄る場合、現在は空いて見える追い越しラインも不許可にする。
  for (const auto &pred : predictions) {
    if (pred.id != target.id) {
      continue;
    }
    const std::size_t count = std::min(pred.s.size(), pred.d.size());
    for (std::size_t i = 0; i < count; ++i) {
      const auto bounds = frame_.corridorBounds(pred.s[i], config_.d_min_m,
                                                 config_.d_max_m);
      min_left_gap = std::min(
          min_left_gap, bounds.d_max - config_.min_wall_margin_m - pred.d[i]);
      min_right_gap = std::min(
          min_right_gap, pred.d[i] -
                             (bounds.d_min + config_.min_wall_margin_m));
    }
    break;
  }

  out.left_pass_gap_m = min_left_gap;
  out.right_pass_gap_m = min_right_gap;
  const double left_threshold =
      out.pass_gap_required_m -
      (isLeftPassMode(mode) ? config_.pass_gap_hysteresis_m : 0.0);
  const double right_threshold =
      out.pass_gap_required_m -
      (isRightPassMode(mode) ? config_.pass_gap_hysteresis_m : 0.0);
  out.can_pass_left = min_left_gap >= left_threshold;
  out.can_pass_right = min_right_gap >= right_threshold;

  if (out.can_pass_left && out.can_pass_right) {
    out.pass_gap_reason = "ok";
  } else if (out.can_pass_left) {
    out.pass_gap_reason = "right_gap_narrow";
  } else if (out.can_pass_right) {
    out.pass_gap_reason = "left_gap_narrow";
  } else {
    out.pass_gap_reason = "both_gap_narrow";
  }
  return out;
}

// 入力: Frenet横位置d。
// 出力: 左右壁マージンのうち小さい方の余裕[m]。
// 処理概要: 負値なら安全コリドー外として、譲り/復帰/速度ガードの判断に使う。
double BlockedRiskAnalyzer::wallClearance(double s, double d) const {
  const auto bounds =
      frame_.corridorBounds(s, config_.d_min_m, config_.d_max_m);
  const double lower_d = bounds.d_min + config_.min_wall_margin_m;
  const double upper_d = bounds.d_max - config_.min_wall_margin_m;
  return std::min(d - lower_d, upper_d - d);
}

// 入力: 相手車状態。
// 出力: 参照線接線方向の速度成分[m/s]。
// 処理概要: vx/vyを相手のFrenet s位置のyawへ射影し、同方向フィルタや予測に使う。
double
BlockedRiskAnalyzer::opponentSDot(const OpponentState &opponent) const {
  const auto ref = frame_.interpolate(opponent.frenet.s);
  return opponent.vx * std::cos(ref.yaw) + opponent.vy * std::sin(ref.yaw);
}

// 入力: BlockedInfo。
// 出力: 横方向リスクとして使う相手index。無ければ-1。
// 処理概要: 現在横並びを優先し、なければ並走候補へフォールバックする。
int BlockedRiskAnalyzer::sideRiskIndex(const BlockedInfo &info) const {
  return info.side_index >= 0 ? info.side_index : info.parallel_side_index;
}

// 入力: BlockedInfo。
// 出力: 譲り/追従対象の相手index。無ければ-1。
// 処理概要: 前方閉塞車両を優先し、無ければ横方向リスク車両を対象にする。
int BlockedRiskAnalyzer::yieldTargetIndex(const BlockedInfo &info) const {
  return info.nearest_index >= 0 ? info.nearest_index : sideRiskIndex(info);
}

} // namespace overtake_planner
