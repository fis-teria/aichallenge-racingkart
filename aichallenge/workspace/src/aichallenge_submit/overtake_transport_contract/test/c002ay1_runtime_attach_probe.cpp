#include "overtake_transport_contract/c002ay1_runtime_observer.hpp"

#include <cstdint>
#include <cstdlib>
#include <string>

namespace c002ay1 = overtake_transport_contract::c002ay1;

int main(int argc, char **argv) {
  if (argc != 6) {
    return 2;
  }
  char *nonce_end = nullptr;
  char *instance_end = nullptr;
  const std::uint64_t nonce = std::strtoull(argv[3], &nonce_end, 10);
  const std::uint64_t instance = std::strtoull(argv[4], &instance_end, 10);
  if (nonce == 0U || instance == 0U || nonce_end == argv[3] ||
      *nonce_end != '\0' || instance_end == argv[4] || *instance_end != '\0') {
    return 3;
  }
  const bool expect_enabled = std::string(argv[5]) == "enabled";
  if (!expect_enabled && std::string(argv[5]) != "disabled") {
    return 4;
  }

  c002ay1::RuntimeObserverConfig config{};
  config.enabled = true;
  config.role = c002ay1::ProducerRole::kPlanner;
  config.socket_path = argv[1];
  config.run_id = argv[2];
  config.session_nonce = nonce;
  config.producer_instance_id = instance;
  auto writer = c002ay1::RuntimeObservationWriter::attach(config);
  if (writer.enabled() != expect_enabled) {
    return 5;
  }
  if (writer.enabled()) {
    c002ay1::PlannerCallbackObservationV1 record{};
    record.sequence = 1U;
    record.return_reason =
        static_cast<std::uint16_t>(c002ay1::ReturnReason::kCompletedNoEmit);
    record.configured_stream_kind = static_cast<std::uint8_t>(
        c002ay1::ConfiguredStreamKind::kLegacyReferenceOverride);
    if (!writer.tryWrite(record)) {
      return 6;
    }
  }
  return 0;
}
