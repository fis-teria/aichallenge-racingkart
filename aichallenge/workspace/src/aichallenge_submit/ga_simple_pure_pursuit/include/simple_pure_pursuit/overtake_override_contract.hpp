#ifndef SIMPLE_PURE_PURSUIT_OVERTAKE_OVERRIDE_CONTRACT_HPP_
#define SIMPLE_PURE_PURSUIT_OVERTAKE_OVERRIDE_CONTRACT_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace simple_pure_pursuit {

constexpr std::int64_t kMaxOvertakeOverridePoints = 1000;
constexpr std::int64_t kMaxOvertakeOverrideGeneration = 16777215;
constexpr double kMinimumSpatialHorizonArcM = 0.50;
constexpr double kMinimumSpatialHorizonForwardTimeSec = 0.75;
constexpr double kMinimumSpatialHorizonResponseDelaySec = 0.25;
constexpr double kMaximumVerifiedSpatialHorizonBrakeDecelMps2 = 1.00;
constexpr double kMaximumSpatialHorizonFailSafeSpeedMps = 0.20;
constexpr double kSpatialProfileEndpointToleranceM = 1.0e-9;

enum class OvertakeOverrideContractKind {
  INACTIVE,
  LATERAL_AND_SPEED_V1,
  SPEED_ONLY_V2,
  LATERAL_AND_SPEED_V3,
  SPATIAL_LATERAL_AND_SPEED_V4,
};

struct OvertakeOverrideContract {
  OvertakeOverrideContractKind kind{OvertakeOverrideContractKind::INACTIVE};
  int mode_id{0};
  std::uint32_t generation{0};
  bool solver_horizon_authorized{false};
  bool mandatory_lateral_avoidance{false};
  std::vector<double> lateral_offsets;
  std::vector<double> speed_caps;
  std::vector<double> longitudinal_offsets_m;
};

// 入力: 単調非減少の距離軸、同じ長さの値列、参照軌道上の距離[m]。
// 出力: 距離軸上で線形補間した値。契約不正ならnullopt。
// 処理概要: plannerの時間horizonを参照軌道の点番号へ直結せず、物理距離で
// Pure Pursuit用の空間profileへ再サンプルする。
inline std::optional<double>
sampleOvertakeProfileByDistance(const std::vector<double> &distances_m,
                                const std::vector<double> &values,
                                double query_distance_m) {
  if (distances_m.empty() || distances_m.size() != values.size() ||
      !std::isfinite(query_distance_m)) {
    return std::nullopt;
  }
  for (std::size_t i = 0; i < distances_m.size(); ++i) {
    if (!std::isfinite(distances_m[i]) || !std::isfinite(values[i]) ||
        (i > 0U && distances_m[i] + 1.0e-9 < distances_m[i - 1U])) {
      return std::nullopt;
    }
  }
  if (query_distance_m <= distances_m.front()) {
    return values.front();
  }
  for (std::size_t i = 1; i < distances_m.size(); ++i) {
    if (query_distance_m > distances_m[i]) {
      continue;
    }
    const double span_m = distances_m[i] - distances_m[i - 1U];
    if (span_m <= 1.0e-9) {
      return values[i];
    }
    const double ratio =
        std::clamp((query_distance_m - distances_m[i - 1U]) / span_m, 0.0, 1.0);
    return values[i - 1U] + (values[i] - values[i - 1U]) * ratio;
  }
  // SafetyEvaluatorが検証した空間horizonより先へ、終端の横offsetを
  // 外挿しない。呼び出し側はnulloptの点を未変更の参照軌道として扱う。
  return std::nullopt;
}

// 入力: 検証済みprofile、再積算したquery距離、現在点と挿入済み終端のindex。
// 出力: provenanceと距離が終端契約に一致した時だけvalues.back()。
// 処理概要: 通常sampleの範囲判定は緩めず、PPが明示的に挿入・特定した終端点
// だけを固定絶対公差内でprofile終端へ正規化する。
inline std::optional<double> normalizeProvenSpatialEndpointDistance(
    double query_distance_m, std::size_t query_index,
    std::size_t endpoint_index, double profile_endpoint_distance_m) {
  if (query_index != endpoint_index || !std::isfinite(query_distance_m) ||
      !std::isfinite(profile_endpoint_distance_m) ||
      std::abs(query_distance_m - profile_endpoint_distance_m) >
          kSpatialProfileEndpointToleranceM) {
    return std::nullopt;
  }
  return profile_endpoint_distance_m;
}

inline std::optional<double> sampleOvertakeProfileAtProvenEndpoint(
    const std::vector<double> &distances_m, const std::vector<double> &values,
    double query_distance_m, std::size_t query_index,
    std::size_t endpoint_index) {
  if (distances_m.empty() || distances_m.size() != values.size() ||
      !normalizeProvenSpatialEndpointDistance(
           query_distance_m, query_index, endpoint_index, distances_m.back())
           .has_value()) {
    return std::nullopt;
  }
  // Keep payload validation identical to the strict generic sampler. Only the
  // proven trajectory endpoint receives normalization to the profile endpoint.
  if (!sampleOvertakeProfileByDistance(distances_m, values,
                                       distances_m.back())
           .has_value() ||
      !std::isfinite(values.back())) {
    return std::nullopt;
  }
  return values.back();
}

// 入力: 通常lookahead、現在速度、低速時にも必要な最小arcと前方時間。
// 出力: 検証済み空間profileに最低限必要なarc長[m]。
// 処理概要: 現在速度で一定時間先までと、検証済み応答遅れ・減速度で
// fail-safe速度まで落とせる距離の大きい方を要求する。停止近傍でも2点目を
// 持てる最小arcを残しつつ、高速時は短い横profileを許可しない。
inline double minimumExecutableSpatialHorizonArc(
    double required_lookahead_m, double current_speed_mps, double minimum_arc_m,
    double minimum_forward_time_sec, double response_delay_sec,
    double verified_brake_decel_mps2, double fail_safe_speed_mps) {
  if (!std::isfinite(required_lookahead_m) || required_lookahead_m < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  if (!std::isfinite(current_speed_mps) || !std::isfinite(minimum_arc_m) ||
      !std::isfinite(minimum_forward_time_sec) ||
      !std::isfinite(response_delay_sec) ||
      !std::isfinite(verified_brake_decel_mps2) ||
      verified_brake_decel_mps2 <= 0.0 ||
      verified_brake_decel_mps2 >
          kMaximumVerifiedSpatialHorizonBrakeDecelMps2 ||
      !std::isfinite(fail_safe_speed_mps)) {
    return std::numeric_limits<double>::infinity();
  }

  // These parameters may only make the admission check more conservative.
  // A launch/config typo must not silently shorten the verified trajectory.
  const double safe_minimum_arc_m =
      std::max(kMinimumSpatialHorizonArcM, minimum_arc_m);
  const double safe_forward_time_sec =
      std::max(kMinimumSpatialHorizonForwardTimeSec, minimum_forward_time_sec);
  const double safe_current_speed_mps = std::max(0.0, current_speed_mps);
  const double safe_response_delay_sec =
      std::max(kMinimumSpatialHorizonResponseDelaySec, response_delay_sec);
  const double safe_fail_speed_mps = std::clamp(
      fail_safe_speed_mps, 0.0, kMaximumSpatialHorizonFailSafeSpeedMps);
  double required_brake_distance_m = 0.0;
  if (safe_current_speed_mps > safe_fail_speed_mps + 1.0e-9) {
    required_brake_distance_m =
        safe_current_speed_mps * safe_response_delay_sec +
        (safe_current_speed_mps * safe_current_speed_mps -
         safe_fail_speed_mps * safe_fail_speed_mps) /
            (2.0 * verified_brake_decel_mps2);
  }
  return std::max({safe_minimum_arc_m,
                   safe_current_speed_mps * safe_forward_time_sec,
                   required_brake_distance_m});
}

// 入力: 空間profile内に残る参照点数・実arc長・最低限必要なarc長[m]。
// 出力: 横overrideをPure Pursuitへ渡せる長さならtrue。
// 処理概要: 1点軌道や最低実行arc未満の評価horizonでは操舵せず、呼び出し側を
// speed-only fail-safeへ落とすための共通境界判定。
inline bool spatialOverrideHorizonSufficient(std::size_t covered_point_count,
                                             double covered_arc_m,
                                             double minimum_required_arc_m) {
  return covered_point_count >= 2U && std::isfinite(covered_arc_m) &&
         std::isfinite(minimum_required_arc_m) && covered_arc_m > 1.0e-6 &&
         covered_arc_m + 1.0e-9 >= std::max(0.0, minimum_required_arc_m);
}

// 入力: 検証済みoverride契約。
// 出力: 直前の有効v2 speed-only契約、または空。
// 処理概要: 不正payload/timeout時に縦速度capだけをfail-closedで維持する。
// v1横軌道と明示inactiveは従来どおり保持せず、横方向を再利用しない。
class OvertakeSpeedOnlyFailClosedLatch {
public:
  void observeValid(const OvertakeOverrideContract &contract) {
    const bool valid_speed_only =
        contract.kind == OvertakeOverrideContractKind::SPEED_ONLY_V2 &&
        contract.mode_id > 0 && contract.generation > 0 &&
        contract.lateral_offsets.empty() && contract.speed_caps.size() == 1 &&
        std::isfinite(contract.speed_caps.front()) &&
        contract.speed_caps.front() > 0.0;
    if (valid_speed_only) {
      speed_only_override_ = contract;
    } else {
      clear();
    }
  }

  const std::optional<OvertakeOverrideContract> &retained() const {
    return speed_only_override_;
  }

  void clear() { speed_only_override_.reset(); }

private:
  std::optional<OvertakeOverrideContract> speed_only_override_;
};

// 入力: speed-only契約の鮮度/種別/generationと、この周期で実際に使った
// 速度cap・最終PP command速度[m/s]。
// 出力: 同一generationのspeed-only契約が最終commandへ反映された時だけtrue。
// 処理概要: speed-onlyをinactiveや横overrideと混同せず、有限な正のcapが
// commandを実際に上限拘束していることまで確認する。古い契約や別generationの
// last-known-goodをtracking proofとして再利用しない。
inline bool speedOnlyTrackingContractApplied(
    bool contract_fresh, bool contract_inactive, bool speed_only_active,
    bool lateral_override_active, std::uint32_t active_generation,
    std::uint32_t contract_generation, double applied_speed_cap_mps,
    double command_speed_mps) {
  return contract_fresh && !contract_inactive && speed_only_active &&
         !lateral_override_active && active_generation > 0U &&
         active_generation == contract_generation &&
         std::isfinite(applied_speed_cap_mps) && applied_speed_cap_mps > 0.0 &&
         std::isfinite(command_speed_mps) && command_speed_mps >= 0.0 &&
         command_speed_mps <= applied_speed_cap_mps + 1.0e-6;
}

inline std::optional<std::int64_t>
finiteIntegerInRange(float value, std::int64_t minimum, std::int64_t maximum) {
  const double as_double = static_cast<double>(value);
  if (!std::isfinite(as_double) || std::floor(as_double) != as_double ||
      as_double < static_cast<double>(minimum) ||
      as_double > static_cast<double>(maximum)) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(as_double);
}

// 入力: /overtake/reference_override のFloat32配列。
// 出力:
// 厳密に検証済みのv1/v3横+速度、v4距離軸付き横+速度、v2速度のみ、またはnullopt。
// 処理概要:
// v2は横軌道を含めず、単一の正の速度capを全horizonで使う契約に限定する。
// 明示inactiveは呼び出し側でclearする。NaN/Infなどの不正値とtimeoutでは、呼び
// 出し側が直前の有効v2速度capだけを保持できる。v1横軌道は保持しない。
inline std::optional<OvertakeOverrideContract>
parseOvertakeOverrideContract(const std::vector<float> &data) {
  if (data.size() < 3) {
    return std::nullopt;
  }
  const auto valid = finiteIntegerInRange(data[0], 1, 1);
  const auto mode_id = finiteIntegerInRange(data[1], 0, 255);
  const auto point_count =
      finiteIntegerInRange(data[2], 0, kMaxOvertakeOverridePoints);
  if (!valid.has_value() || !mode_id.has_value() || !point_count.has_value()) {
    return std::nullopt;
  }

  if (point_count.value() == 0) {
    if (mode_id.value() == 0) {
      if (data.size() == 3) {
        return OvertakeOverrideContract{};
      }
      if (data.size() != 5) {
        return std::nullopt;
      }
      const auto version = finiteIntegerInRange(data[3], 1, 1);
      const auto generation =
          finiteIntegerInRange(data[4], 1, kMaxOvertakeOverrideGeneration);
      if (!version.has_value() || !generation.has_value()) {
        return std::nullopt;
      }
      OvertakeOverrideContract contract;
      contract.generation = static_cast<std::uint32_t>(generation.value());
      return contract;
    }

    if (data.size() != 6) {
      return std::nullopt;
    }
    const auto version = finiteIntegerInRange(data[3], 2, 2);
    const auto generation =
        finiteIntegerInRange(data[4], 1, kMaxOvertakeOverrideGeneration);
    const double speed_cap_mps = static_cast<double>(data[5]);
    if (!version.has_value() || !generation.has_value() ||
        !std::isfinite(speed_cap_mps) || speed_cap_mps <= 0.0) {
      return std::nullopt;
    }
    OvertakeOverrideContract contract;
    contract.kind = OvertakeOverrideContractKind::SPEED_ONLY_V2;
    contract.mode_id = static_cast<int>(mode_id.value());
    contract.generation = static_cast<std::uint32_t>(generation.value());
    contract.speed_caps = {speed_cap_mps};
    return contract;
  }

  if (mode_id.value() == 0) {
    return std::nullopt;
  }
  const std::size_t count = static_cast<std::size_t>(point_count.value());
  const std::size_t expected = 3 + 2 * count;
  const std::size_t spatial_expected = expected + count + 3;
  if (data.size() != expected && data.size() != expected + 2 &&
      data.size() != expected + 3 && data.size() != spatial_expected) {
    return std::nullopt;
  }
  OvertakeOverrideContract contract;
  contract.kind = OvertakeOverrideContractKind::LATERAL_AND_SPEED_V1;
  contract.mode_id = static_cast<int>(mode_id.value());
  contract.lateral_offsets.reserve(count);
  contract.speed_caps.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double offset_m = static_cast<double>(data[3 + i]);
    const double speed_cap_mps = static_cast<double>(data[3 + count + i]);
    if (!std::isfinite(offset_m) || !std::isfinite(speed_cap_mps) ||
        speed_cap_mps <= 0.0) {
      return std::nullopt;
    }
    contract.lateral_offsets.push_back(offset_m);
    contract.speed_caps.push_back(speed_cap_mps);
  }
  if (data.size() == expected + 2) {
    const auto version = finiteIntegerInRange(data[expected], 1, 1);
    const auto generation = finiteIntegerInRange(
        data[expected + 1], 1, kMaxOvertakeOverrideGeneration);
    if (!version.has_value() || !generation.has_value()) {
      return std::nullopt;
    }
    contract.generation = static_cast<std::uint32_t>(generation.value());
  } else if (data.size() == expected + 3) {
    const auto version = finiteIntegerInRange(data[expected], 3, 3);
    const auto generation = finiteIntegerInRange(
        data[expected + 1], 1, kMaxOvertakeOverrideGeneration);
    const auto horizon_intent = finiteIntegerInRange(data[expected + 2], 0, 2);
    if (!version.has_value() || !generation.has_value() ||
        !horizon_intent.has_value()) {
      return std::nullopt;
    }
    contract.kind = OvertakeOverrideContractKind::LATERAL_AND_SPEED_V3;
    contract.generation = static_cast<std::uint32_t>(generation.value());
    contract.solver_horizon_authorized = horizon_intent.value() != 0;
    contract.mandatory_lateral_avoidance = horizon_intent.value() == 2;
  } else if (data.size() == spatial_expected) {
    contract.longitudinal_offsets_m.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      const double distance_m = static_cast<double>(data[expected + i]);
      if (!std::isfinite(distance_m) || distance_m < 0.0 ||
          (i == 0U && std::abs(distance_m) > 1.0e-5) ||
          (i > 0U &&
           distance_m + 1.0e-6 < contract.longitudinal_offsets_m.back())) {
        return std::nullopt;
      }
      contract.longitudinal_offsets_m.push_back(distance_m);
    }
    if (contract.longitudinal_offsets_m.back() <= 1.0e-6) {
      return std::nullopt;
    }
    const auto version = finiteIntegerInRange(data[expected + count], 4, 4);
    const auto generation = finiteIntegerInRange(
        data[expected + count + 1U], 1, kMaxOvertakeOverrideGeneration);
    const auto horizon_intent =
        finiteIntegerInRange(data[expected + count + 2U], 0, 2);
    if (!version.has_value() || !generation.has_value() ||
        !horizon_intent.has_value()) {
      return std::nullopt;
    }
    contract.kind = OvertakeOverrideContractKind::SPATIAL_LATERAL_AND_SPEED_V4;
    contract.generation = static_cast<std::uint32_t>(generation.value());
    contract.solver_horizon_authorized = horizon_intent.value() != 0;
    contract.mandatory_lateral_avoidance = horizon_intent.value() == 2;
  }
  return contract;
}

} // namespace simple_pure_pursuit

#endif // SIMPLE_PURE_PURSUIT_OVERTAKE_OVERRIDE_CONTRACT_HPP_
