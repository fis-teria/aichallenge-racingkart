#include "pp_callback_span_recorder.hpp"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <functional>

namespace simple_pure_pursuit::timing_diagnostic {
namespace {

std::uint64_t steadyNowNs() noexcept {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

std::uint64_t currentThreadId() noexcept {
  return static_cast<std::uint64_t>(
      std::hash<std::thread::id>{}(std::this_thread::get_id()));
}

}  // namespace

CallbackSpanRecorder &CallbackSpanRecorder::instance() noexcept {
  static CallbackSpanRecorder recorder;
  return recorder;
}

void CallbackSpanRecorder::initialize(bool enabled) noexcept {
  for (auto &record : records_) {
    record = CallbackSpan{};
  }
  size_ = 0;
  next_sequence_ = 1;
  overflowed_ = false;
  enabled_ = enabled;
}

void CallbackSpanRecorder::commit(CallbackKind kind, std::uint64_t entry_ns,
                                  std::uint64_t exit_ns,
                                  std::uint64_t thread_id) noexcept {
  if (size_ >= records_.size()) {
    overflowed_ = true;
    return;
  }
  records_[size_++] =
      CallbackSpan{next_sequence_++, entry_ns, exit_ns, thread_id, kind};
}

bool CallbackSpanRecorder::writeFromEnvironment() const noexcept {
  const char *path = std::getenv("C002AY0_PP_CALLBACK_SPAN_PATH");
  if (path == nullptr || path[0] == '\0') {
    return false;
  }
  try {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
      return false;
    }
    output << "count " << size_ << " overflow " << (overflowed_ ? 1 : 0)
           << '\n';
    for (std::size_t index = 0; index < size_; ++index) {
      const auto &record = records_[index];
      output << record.sequence << ' '
             << static_cast<std::uint16_t>(record.kind) << ' '
             << record.entry_ns << ' ' << record.exit_ns << ' '
             << record.thread_id << '\n';
    }
    return static_cast<bool>(output);
  } catch (...) {
    return false;
  }
}

void CallbackSpanRecorder::resetForTest() noexcept { initialize(true); }

CallbackSpanScope::CallbackSpanScope(CallbackKind kind) noexcept
    : kind_(kind), entry_ns_(0), thread_id_(0),
      active_(CallbackSpanRecorder::instance().enabled()) {
  if (active_) {
    entry_ns_ = steadyNowNs();
    thread_id_ = currentThreadId();
  }
}

CallbackSpanScope::~CallbackSpanScope() {
  if (active_) {
    CallbackSpanRecorder::instance().commit(kind_, entry_ns_, steadyNowNs(),
                                            thread_id_);
  }
}

}  // namespace simple_pure_pursuit::timing_diagnostic
