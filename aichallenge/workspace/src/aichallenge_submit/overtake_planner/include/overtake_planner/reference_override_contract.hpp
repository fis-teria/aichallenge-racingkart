#ifndef OVERTAKE_PLANNER_REFERENCE_OVERRIDE_CONTRACT_HPP_
#define OVERTAKE_PLANNER_REFERENCE_OVERRIDE_CONTRACT_HPP_

#include "overtake_planner/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace overtake_planner {

enum class ReferenceOverrideWireKind {
  INACTIVE,
  LATERAL_AND_SPEED_V1,
  SPEED_ONLY_V2,
  LATERAL_AND_SPEED_V3,
};

struct ReferenceOverrideWirePayload {
  ReferenceOverrideWireKind kind{ReferenceOverrideWireKind::INACTIVE};
  int mode_id{0};
  double speed_cap_mps{0.0};
  std::vector<float> data;
};

// 入力: 同じ契約で生成した2つのwire payload。
// 出力: generation以外の意味的な内容が一致する時だけtrue。
// 処理概要: generationは各変更を識別するための末尾フィールドであり、同じ
// overrideを再publishするかの判定には含めない。生成関数が作る全契約形式で
// generationは末尾にあるため、先頭から末尾直前までを比較する。
inline bool referenceOverrideWirePayloadSemanticallyEqual(
    const ReferenceOverrideWirePayload &lhs,
    const ReferenceOverrideWirePayload &rhs) {
  if (lhs.kind != rhs.kind || lhs.mode_id != rhs.mode_id ||
      lhs.data.empty() || lhs.data.size() != rhs.data.size()) {
    return false;
  }
  // v1 / explicit inactive append generation at the end. v2 is intentionally
  // [valid, mode, 0, version, generation, cap], so its generation is index 4.
  // v3 appends [version, generation, solver_horizon_intent].
  const std::size_t generation_index =
      lhs.kind == ReferenceOverrideWireKind::SPEED_ONLY_V2
          ? 4U
          : lhs.kind == ReferenceOverrideWireKind::LATERAL_AND_SPEED_V3
                ? lhs.data.size() - 2U
                : lhs.data.size() - 1U;
  if (generation_index >= lhs.data.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.data.size(); ++i) {
    if (i != generation_index && lhs.data[i] != rhs.data[i]) {
      return false;
    }
  }
  return true;
}

// 入力: PlannerOutputとFloat32で正確に表せるgeneration。
// 出力: /overtake/reference_overrideへ出すv3またはv2 payload。
// 処理概要: 安全評価済み横列がある時だけv3を使い、横列を安全に作れない時は
// speed-only v2へ落とす。どちらも成立しない場合だけ明示inactiveを出す。
inline ReferenceOverrideWirePayload makeReferenceOverrideWirePayload(
    const PlannerOutput &output, std::uint32_t generation) {
  constexpr std::uint32_t kMaxGeneration = 16777215U;
  const auto valid_generation =
      std::clamp(generation, std::uint32_t{1}, kMaxGeneration);
  const int mode_id = static_cast<int>(output.mode);
  const auto inactive_payload = [valid_generation]() {
    ReferenceOverrideWirePayload payload;
    payload.data = {1.0F, 0.0F, 0.0F, 1.0F,
                    static_cast<float>(valid_generation)};
    return payload;
  };

  const bool lateral_payload_valid =
      output.active_override && mode_id > 0 &&
      output.lateral_offsets.size() == output.speed_caps.size() &&
      !output.lateral_offsets.empty() &&
      std::all_of(output.lateral_offsets.begin(), output.lateral_offsets.end(),
                  [](double value) { return std::isfinite(value); }) &&
      std::all_of(output.speed_caps.begin(), output.speed_caps.end(),
                  [](double value) {
                    return std::isfinite(value) && value > 0.0;
                  });
  if (lateral_payload_valid) {
    ReferenceOverrideWirePayload payload;
    payload.kind = ReferenceOverrideWireKind::LATERAL_AND_SPEED_V3;
    payload.mode_id = mode_id;
    payload.data.reserve(3 + output.lateral_offsets.size() * 2 + 3);
    payload.data = {1.0F, static_cast<float>(mode_id),
                    static_cast<float>(output.lateral_offsets.size())};
    for (const double offset_m : output.lateral_offsets) {
      payload.data.push_back(static_cast<float>(offset_m));
    }
    for (const double speed_cap_mps : output.speed_caps) {
      payload.data.push_back(static_cast<float>(speed_cap_mps));
    }
    // v3: [valid, mode, n, d[n], v[n], version=3, generation, intent].
    // intent=NONEの横列は通常trajectory用であり、solver horizonとしては
    // publish/採用してはならない。旧receiverはv3を厳密にrejectするため、
    // 横軌道を誤って継続するより安全側へ倒れる。
    payload.data.push_back(3.0F);
    payload.data.push_back(static_cast<float>(valid_generation));
    payload.data.push_back(static_cast<float>(
        static_cast<int>(output.solver_horizon_intent)));
    return payload;
  }

  double speed_cap_mps = output.applied_speed_cap_mps;
  if (!std::isfinite(speed_cap_mps) || speed_cap_mps <= 0.0) {
    speed_cap_mps = std::numeric_limits<double>::infinity();
    for (const double candidate_cap_mps : output.speed_caps) {
      if (std::isfinite(candidate_cap_mps) && candidate_cap_mps > 0.0) {
        speed_cap_mps = std::min(speed_cap_mps, candidate_cap_mps);
      }
    }
  }
  if (output.longitudinal_speed_cap_active && mode_id > 0 &&
      std::isfinite(speed_cap_mps) && speed_cap_mps > 0.0) {
    ReferenceOverrideWirePayload payload;
    payload.kind = ReferenceOverrideWireKind::SPEED_ONLY_V2;
    payload.mode_id = mode_id;
    payload.speed_cap_mps = speed_cap_mps;
    // v2: [valid, mode_id, n=0, contract_version=2, generation, speed_cap].
    payload.data = {1.0F, static_cast<float>(mode_id), 0.0F, 2.0F,
                    static_cast<float>(valid_generation),
                    static_cast<float>(speed_cap_mps)};
    return payload;
  }
  return inactive_payload();
}

}  // namespace overtake_planner

#endif  // OVERTAKE_PLANNER_REFERENCE_OVERRIDE_CONTRACT_HPP_
