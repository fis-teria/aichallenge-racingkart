#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>

namespace simple_pure_pursuit::timing_diagnostic {

enum class CallbackKind : std::uint16_t {
  kKinematics = 1,
  kTrajectory = 2,
  kMpcPredictedHorizon = 3,
  kMpcPredictedHorizonContract = 4,
  kOvertakeOverride = 5,
  kSteeringStatus = 6,
  kMpcHealth = 7,
  kRecoveryStatus = 8,
  kOvertakePlan = 9,
  kRaceArmed = 10,
};

struct CallbackSpan {
  std::uint64_t sequence{};
  std::uint64_t entry_ns{};
  std::uint64_t exit_ns{};
  std::uint64_t thread_id{};
  CallbackKind kind{};
};

class CallbackSpanRecorder {
public:
  static constexpr std::size_t kCapacity = 262144;

  static CallbackSpanRecorder &instance() noexcept;

  void initialize(bool enabled = true) noexcept;
  void commit(CallbackKind kind, std::uint64_t entry_ns,
              std::uint64_t exit_ns, std::uint64_t thread_id) noexcept;
  bool writeFromEnvironment() const noexcept;

  const CallbackSpan *data() const noexcept { return records_.data(); }
  std::size_t size() const noexcept { return size_; }
  bool overflowed() const noexcept { return overflowed_; }
  bool enabled() const noexcept { return enabled_; }
  void resetForTest() noexcept;

private:
  std::array<CallbackSpan, kCapacity> records_{};
  std::size_t size_{};
  std::uint64_t next_sequence_{1};
  bool overflowed_{};
  bool enabled_{true};
};

class CallbackSpanScope {
public:
  explicit CallbackSpanScope(CallbackKind kind) noexcept;
  ~CallbackSpanScope();

  CallbackSpanScope(const CallbackSpanScope &) = delete;
  CallbackSpanScope &operator=(const CallbackSpanScope &) = delete;

private:
  CallbackKind kind_;
  std::uint64_t entry_ns_;
  std::uint64_t thread_id_;
  bool active_;
};

}  // namespace simple_pure_pursuit::timing_diagnostic
