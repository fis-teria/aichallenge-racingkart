#ifndef SIMPLE_PURE_PURSUIT_OVERTAKE_OVERRIDE_CONTRACT_HPP_
#define SIMPLE_PURE_PURSUIT_OVERTAKE_OVERRIDE_CONTRACT_HPP_

#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace simple_pure_pursuit {

constexpr std::int64_t kMaxOvertakeOverridePoints = 1000;
constexpr std::int64_t kMaxOvertakeOverrideGeneration = 16777215;

enum class OvertakeOverrideContractKind {
  INACTIVE,
  LATERAL_AND_SPEED_V1,
  SPEED_ONLY_V2,
  LATERAL_AND_SPEED_V3,
};

struct OvertakeOverrideContract {
  OvertakeOverrideContractKind kind{OvertakeOverrideContractKind::INACTIVE};
  int mode_id{0};
  std::uint32_t generation{0};
  bool solver_horizon_authorized{false};
  bool mandatory_lateral_avoidance{false};
  std::vector<double> lateral_offsets;
  std::vector<double> speed_caps;
};

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

inline std::optional<std::int64_t> finiteIntegerInRange(
    float value, std::int64_t minimum, std::int64_t maximum) {
  const double as_double = static_cast<double>(value);
  if (!std::isfinite(as_double) || std::floor(as_double) != as_double ||
      as_double < static_cast<double>(minimum) ||
      as_double > static_cast<double>(maximum)) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(as_double);
}

// 入力: /overtake/reference_override のFloat32配列。
// 出力: 厳密に検証済みのv1横+速度、v2速度のみ、またはnullopt。
// 処理概要: v2は横軌道を含めず、単一の正の速度capを全horizonで使う契約に限定する。
// 明示inactiveは呼び出し側でclearする。NaN/Infなどの不正値とtimeoutでは、呼び
// 出し側が直前の有効v2速度capだけを保持できる。v1横軌道は保持しない。
inline std::optional<OvertakeOverrideContract> parseOvertakeOverrideContract(
    const std::vector<float> &data) {
  if (data.size() < 3) {
    return std::nullopt;
  }
  const auto valid = finiteIntegerInRange(data[0], 1, 1);
  const auto mode_id = finiteIntegerInRange(data[1], 0, 255);
  const auto point_count = finiteIntegerInRange(
      data[2], 0, kMaxOvertakeOverridePoints);
  if (!valid.has_value() || !mode_id.has_value() ||
      !point_count.has_value()) {
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
      const auto generation = finiteIntegerInRange(
          data[4], 1, kMaxOvertakeOverrideGeneration);
      if (!version.has_value() || !generation.has_value()) {
        return std::nullopt;
      }
      return OvertakeOverrideContract{};
    }

    if (data.size() != 6) {
      return std::nullopt;
    }
    const auto version = finiteIntegerInRange(data[3], 2, 2);
    const auto generation = finiteIntegerInRange(
        data[4], 1, kMaxOvertakeOverrideGeneration);
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
  if (data.size() != expected && data.size() != expected + 2 &&
      data.size() != expected + 3) {
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
  }
  return contract;
}

}  // namespace simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUIT_OVERTAKE_OVERRIDE_CONTRACT_HPP_
