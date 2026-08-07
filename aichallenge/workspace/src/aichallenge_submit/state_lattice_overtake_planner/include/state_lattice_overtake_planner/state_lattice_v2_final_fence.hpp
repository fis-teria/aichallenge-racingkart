#pragma once

#include "overtake_transport_contract/c002ay0_canonical.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace state_lattice_overtake_planner {

using FinalFenceDigest = std::array<std::uint8_t, 32U>;

struct FinalFenceIdentity {
  std::string producer_instance_id;
  std::string session_id;
  std::uint64_t proposal_sequence{0U};
  std::uint32_t plan_generation{0U};
  std::uint32_t source_generation{0U};
  std::int32_t source_stamp_sec{0};
  std::uint32_t source_stamp_nanosec{0U};
  std::string frame_id;
  FinalFenceDigest canonical_sha256{};
  std::uint64_t publish_monotonic_ns{0U};
};

struct FinalFenceRequest {
  std::uint8_t schema_version{0U};
  std::string execution_nonce;
  std::string expected_producer_instance_id;
  std::string expected_session_id;
  std::uint64_t sealed_epoch_id{0U};
  std::uint64_t request_id{0U};
  FinalFenceDigest canonical_sha256{};
};

struct FinalFenceSnapshot {
  std::uint8_t schema_version{1U};
  std::string execution_nonce;
  std::string producer_instance_id;
  std::string session_id;
  std::uint64_t sealed_epoch_id{0U};
  std::uint64_t fence_id{1U};
  std::uint64_t request_id{1U};
  FinalFenceDigest request_canonical_sha256{};
  std::uint64_t final_committed_ordinal{0U};
  std::uint64_t successful_emission_count{0U};
  bool final_identity_present{false};
  FinalFenceIdentity final_identity{};
  std::string proposal_qos_fingerprint;
  std::uint64_t quiesced_monotonic_ns{0U};
  std::uint64_t fence_publish_monotonic_ns{0U};
  FinalFenceDigest canonical_sha256{};
};

namespace final_fence_canonical {

inline void appendU8(std::vector<std::uint8_t> *bytes, std::uint8_t value) {
  bytes->push_back(value);
}

template <typename UInt>
inline void appendUnsigned(std::vector<std::uint8_t> *bytes, UInt value) {
  for (std::size_t index = 0; index < sizeof(UInt); ++index) {
    const auto shift = 8U * (sizeof(UInt) - 1U - index);
    bytes->push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
  }
}

inline void appendString(std::vector<std::uint8_t> *bytes,
                         const std::string &value) {
  appendUnsigned<std::uint32_t>(bytes,
                                static_cast<std::uint32_t>(value.size()));
  bytes->insert(bytes->end(), value.begin(), value.end());
}

inline void appendDigest(std::vector<std::uint8_t> *bytes,
                         const FinalFenceDigest &digest) {
  bytes->insert(bytes->end(), digest.begin(), digest.end());
}

inline void appendIdentity(std::vector<std::uint8_t> *bytes,
                           const FinalFenceIdentity &identity) {
  appendString(bytes, identity.producer_instance_id);
  appendString(bytes, identity.session_id);
  appendUnsigned(bytes, identity.proposal_sequence);
  appendUnsigned(bytes, identity.plan_generation);
  appendUnsigned(bytes, identity.source_generation);
  appendUnsigned(bytes, static_cast<std::uint32_t>(identity.source_stamp_sec));
  appendUnsigned(bytes, identity.source_stamp_nanosec);
  appendString(bytes, identity.frame_id);
  appendDigest(bytes, identity.canonical_sha256);
  appendUnsigned(bytes, identity.publish_monotonic_ns);
}

inline FinalFenceDigest requestDigest(const FinalFenceRequest &request) {
  std::vector<std::uint8_t> bytes;
  bytes.reserve(256U);
  appendString(&bytes, "state_lattice_v2_quiesce_request/v1");
  appendU8(&bytes, request.schema_version);
  appendString(&bytes, request.execution_nonce);
  appendString(&bytes, request.expected_producer_instance_id);
  appendString(&bytes, request.expected_session_id);
  appendUnsigned(&bytes, request.sealed_epoch_id);
  appendUnsigned(&bytes, request.request_id);
  return overtake_transport_contract::c002ay0::sha256(bytes);
}

inline FinalFenceDigest fenceDigest(const FinalFenceSnapshot &fence) {
  std::vector<std::uint8_t> bytes;
  bytes.reserve(512U);
  appendString(&bytes, "state_lattice_v2_final_fence/v1");
  appendU8(&bytes, fence.schema_version);
  appendString(&bytes, fence.execution_nonce);
  appendString(&bytes, fence.producer_instance_id);
  appendString(&bytes, fence.session_id);
  appendUnsigned(&bytes, fence.sealed_epoch_id);
  appendUnsigned(&bytes, fence.fence_id);
  appendUnsigned(&bytes, fence.request_id);
  appendDigest(&bytes, fence.request_canonical_sha256);
  appendUnsigned(&bytes, fence.final_committed_ordinal);
  appendUnsigned(&bytes, fence.successful_emission_count);
  appendU8(&bytes, fence.final_identity_present ? 1U : 0U);
  if (fence.final_identity_present) {
    appendIdentity(&bytes, fence.final_identity);
  }
  appendString(&bytes, fence.proposal_qos_fingerprint);
  appendUnsigned(&bytes, fence.quiesced_monotonic_ns);
  appendUnsigned(&bytes, fence.fence_publish_monotonic_ns);
  return overtake_transport_contract::c002ay0::sha256(bytes);
}

} // namespace final_fence_canonical

enum class FinalFencePublicationPhase : std::uint8_t {
  kDisabled,
  kOpen,
  kQuiescing,
  kFenced,
};

enum class FinalFenceRequestResult : std::uint8_t {
  kAccepted,
  kMalformed,
  kDuplicateOrConflicting,
  kDisabled,
  kAlreadyTerminal,
};

// Evidence-only state around the one actual publish-then-commit path. No wait
// is exposed: the request callback closes admission and returns immediately;
// the last admitted publisher makes the immutable fence snapshot eligible.
class StateLatticeV2FinalFenceState {
public:
  struct Config {
    bool enabled{false};
    std::string execution_nonce;
    std::string producer_instance_id;
    std::string session_id;
    std::uint64_t sealed_epoch_id{0U};
    std::string proposal_qos_fingerprint;
  };

  class Admission {
  public:
    Admission() = default;
    Admission(const Admission &) = delete;
    Admission &operator=(const Admission &) = delete;
    Admission(Admission &&other) noexcept : owner_(other.owner_) {
      other.owner_ = nullptr;
    }
    Admission &operator=(Admission &&) = delete;
    explicit operator bool() const { return owner_ != nullptr; }

  private:
    explicit Admission(StateLatticeV2FinalFenceState *owner) : owner_(owner) {}
    StateLatticeV2FinalFenceState *owner_{nullptr};
    friend class StateLatticeV2FinalFenceState;
  };

  explicit StateLatticeV2FinalFenceState(Config config)
      : config_(std::move(config)),
        phase_(config_.enabled ? FinalFencePublicationPhase::kOpen
                              : FinalFencePublicationPhase::kDisabled) {
    if (config_.enabled &&
        (config_.execution_nonce.empty() ||
         config_.producer_instance_id.empty() || config_.session_id.empty() ||
         config_.sealed_epoch_id == 0U ||
         config_.proposal_qos_fingerprint.empty())) {
      evidence_fault_ = "invalid_final_fence_configuration";
    }
  }

  std::optional<Admission> tryAdmit(const std::string &producer_instance_id,
                                    const std::string &session_id,
                                    std::uint64_t sealed_epoch_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != FinalFencePublicationPhase::kOpen) {
      if (phase_ == FinalFencePublicationPhase::kFenced) {
        latchFaultLocked("post_fence_proposal_admission");
      }
      return std::nullopt;
    }
    if (producer_instance_id != config_.producer_instance_id ||
        session_id != config_.session_id ||
        sealed_epoch_id != config_.sealed_epoch_id) {
      latchFaultLocked("proposal_identity_outside_sealed_epoch");
      return std::nullopt;
    }
    if (in_flight_ == std::numeric_limits<std::uint64_t>::max()) {
      latchFaultLocked("publication_in_flight_overflow");
      return std::nullopt;
    }
    ++in_flight_;
    return Admission(this);
  }

  bool finishCommitted(Admission &&admission, std::uint64_t ordinal,
                       const FinalFenceIdentity &identity) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!consumeAdmissionLocked(admission)) {
      latchFaultLocked("publication_admission_mismatch");
      return false;
    }
    const bool valid =
        ordinal != 0U &&
        ordinal != std::numeric_limits<std::uint64_t>::max() &&
        successful_emission_count_ != std::numeric_limits<std::uint64_t>::max() &&
        ordinal == final_committed_ordinal_ + 1U &&
        ordinal == successful_emission_count_ + 1U &&
        identity.proposal_sequence == ordinal &&
        identity.producer_instance_id == config_.producer_instance_id &&
        identity.session_id == config_.session_id;
    if (!valid) {
      latchFaultLocked("committed_ordinal_or_identity_mismatch");
      return false;
    }
    // Accounting remains independent of the evidence-fault latch. A malformed
    // diagnostic request invalidates the campaign but cannot perturb ordinary
    // proposal publication or its production commit state.
    final_committed_ordinal_ = ordinal;
    ++successful_emission_count_;
    final_identity_ = identity;
    return true;
  }

  void finishUncommitted(Admission &&admission) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!consumeAdmissionLocked(admission)) {
      latchFaultLocked("publication_admission_mismatch");
    }
  }

  void finishAmbiguous(Admission &&admission) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!consumeAdmissionLocked(admission)) {
      latchFaultLocked("publication_admission_mismatch");
    }
    latchFaultLocked("publication_or_commit_ambiguous");
  }

  FinalFenceRequestResult requestQuiesce(const FinalFenceRequest &request,
                                         std::uint64_t monotonic_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ == FinalFencePublicationPhase::kDisabled) {
      return FinalFenceRequestResult::kDisabled;
    }
    if (phase_ == FinalFencePublicationPhase::kFenced) {
      latchFaultLocked("request_after_fence");
      return FinalFenceRequestResult::kAlreadyTerminal;
    }
    if (request_seen_) {
      latchFaultLocked("duplicate_or_conflicting_quiesce_request");
      return FinalFenceRequestResult::kDuplicateOrConflicting;
    }
    request_seen_ = true;
    const bool valid =
        request.schema_version == 1U && request.request_id == 1U &&
        request.execution_nonce == config_.execution_nonce &&
        request.expected_producer_instance_id == config_.producer_instance_id &&
        request.expected_session_id == config_.session_id &&
        request.sealed_epoch_id == config_.sealed_epoch_id &&
        request.canonical_sha256 ==
            final_fence_canonical::requestDigest(request) &&
        monotonic_ns != 0U;
    if (!valid) {
      latchFaultLocked("malformed_or_wrong_quiesce_request");
      return FinalFenceRequestResult::kMalformed;
    }
    accepted_request_ = request;
    quiesced_monotonic_ns_ = monotonic_ns;
    phase_ = FinalFencePublicationPhase::kQuiescing;
    return FinalFenceRequestResult::kAccepted;
  }

  std::optional<FinalFenceSnapshot>
  prepareFence(std::uint64_t fence_publish_monotonic_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != FinalFencePublicationPhase::kQuiescing || in_flight_ != 0U ||
        !evidence_fault_.empty() || !accepted_request_.has_value() ||
        fence_publish_monotonic_ns == 0U) {
      return std::nullopt;
    }
    FinalFenceSnapshot fence;
    fence.execution_nonce = config_.execution_nonce;
    fence.producer_instance_id = config_.producer_instance_id;
    fence.session_id = config_.session_id;
    fence.sealed_epoch_id = config_.sealed_epoch_id;
    fence.request_id = accepted_request_->request_id;
    fence.request_canonical_sha256 = accepted_request_->canonical_sha256;
    fence.final_committed_ordinal = final_committed_ordinal_;
    fence.successful_emission_count = successful_emission_count_;
    fence.final_identity_present = final_identity_.has_value();
    if (final_identity_.has_value()) {
      fence.final_identity = final_identity_.value();
    }
    fence.proposal_qos_fingerprint = config_.proposal_qos_fingerprint;
    fence.quiesced_monotonic_ns = quiesced_monotonic_ns_;
    fence.fence_publish_monotonic_ns = fence_publish_monotonic_ns;
    fence.canonical_sha256 = final_fence_canonical::fenceDigest(fence);
    frozen_fence_ = fence;
    phase_ = FinalFencePublicationPhase::kFenced;
    return fence;
  }

  void markFencePublicationAmbiguous() {
    std::lock_guard<std::mutex> lock(mutex_);
    latchFaultLocked("fence_publication_ambiguous");
  }

  FinalFencePublicationPhase phase() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return phase_;
  }
  std::uint64_t inFlight() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return in_flight_;
  }
  std::uint64_t finalCommittedOrdinal() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return final_committed_ordinal_;
  }
  std::uint64_t successfulEmissionCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return successful_emission_count_;
  }
  std::string evidenceFault() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return evidence_fault_;
  }

private:
  bool consumeAdmissionLocked(Admission &admission) {
    if (admission.owner_ != this || in_flight_ == 0U) {
      return false;
    }
    admission.owner_ = nullptr;
    --in_flight_;
    return true;
  }
  void latchFaultLocked(const char *fault) {
    if (evidence_fault_.empty()) {
      evidence_fault_ = fault;
    }
  }

  const Config config_;
  mutable std::mutex mutex_;
  FinalFencePublicationPhase phase_;
  std::string evidence_fault_;
  bool request_seen_{false};
  std::uint64_t in_flight_{0U};
  std::uint64_t final_committed_ordinal_{0U};
  std::uint64_t successful_emission_count_{0U};
  std::uint64_t quiesced_monotonic_ns_{0U};
  std::optional<FinalFenceIdentity> final_identity_;
  std::optional<FinalFenceRequest> accepted_request_;
  std::optional<FinalFenceSnapshot> frozen_fence_;
};

} // namespace state_lattice_overtake_planner
