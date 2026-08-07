#include "overtake_transport_contract/c002ay1_runtime_observer.hpp"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

namespace c002ay1 = overtake_transport_contract::c002ay1;

namespace {

std::optional<std::uint64_t> parseUnsigned(const char *text) {
  if (text == nullptr || *text == '\0') {
    return std::nullopt;
  }
  char *end = nullptr;
  const auto value = std::strtoull(text, &end, 10);
  if (end == text || *end != '\0') {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(value);
}

c002ay1::RuntimeObserverConfig makeConfig(const std::string &socket_path,
                                          const std::string &run_id,
                                          c002ay1::ProducerRole role,
                                          std::uint64_t nonce,
                                          std::uint64_t instance) {
  c002ay1::RuntimeObserverConfig config{};
  config.enabled = true;
  config.socket_path = socket_path;
  config.run_id = run_id;
  config.role = role;
  config.session_nonce = nonce;
  config.producer_instance_id = instance;
  return config;
}

} // namespace

int main(int argc, char **argv) {
  if (argc != 7) {
    return 2;
  }
  const auto planner_nonce = parseUnsigned(argv[3]);
  const auto planner_instance = parseUnsigned(argv[4]);
  const auto pp_nonce = parseUnsigned(argv[5]);
  const auto pp_instance = parseUnsigned(argv[6]);
  if (!planner_nonce.has_value() || !planner_instance.has_value() ||
      !pp_nonce.has_value() || !pp_instance.has_value()) {
    return 3;
  }

  auto planner = c002ay1::RuntimeObservationWriter::attach(
      makeConfig(argv[1], argv[2], c002ay1::ProducerRole::kPlanner,
                 planner_nonce.value(), planner_instance.value()));
  if (!planner.enabled()) {
    std::cerr << "planner attach failed\n";
    return 4;
  }
  {
    c002ay1::PlannerObservationScope scope(
        planner, 1, 2U,
        c002ay1::ConfiguredStreamKind::kLegacyReferenceOverride);
    scope.record().legacy_publish_count = 1U;
    scope.record().selected_proposal_count = 1U;
    scope.record().flags |= c002ay1::kFlagEmitted;
    scope.setReturnReason(c002ay1::ReturnReason::kCompletedEmit);
  }

  auto pp = c002ay1::RuntimeObservationWriter::attach(
      makeConfig(argv[1], argv[2], c002ay1::ProducerRole::kPrimaryPurePursuit,
                 pp_nonce.value(), pp_instance.value()));
  if (!pp.enabled()) {
    std::cerr << "PP attach failed\n";
    return 5;
  }
  {
    c002ay1::PpObservationScope scope(pp, 3, 4U);
    scope.record().flags |= c002ay1::kFlagEmitted;
    scope.setReturnReason(c002ay1::ReturnReason::kCompletedEmit);
  }
  return 0;
}
