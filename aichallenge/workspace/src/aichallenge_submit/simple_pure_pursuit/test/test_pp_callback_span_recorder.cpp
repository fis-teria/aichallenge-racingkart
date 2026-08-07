#include "pp_callback_span_recorder.hpp"

#include <gtest/gtest.h>

#include <thread>

namespace diagnostic = simple_pure_pursuit::timing_diagnostic;

TEST(PpCallbackSpanRecorder, CommitsOrderedSameThreadSpans) {
  auto &recorder = diagnostic::CallbackSpanRecorder::instance();
  recorder.resetForTest();
  {
    diagnostic::CallbackSpanScope scope(
        diagnostic::CallbackKind::kKinematics);
  }
  {
    diagnostic::CallbackSpanScope scope(
        diagnostic::CallbackKind::kTrajectory);
  }

  ASSERT_EQ(recorder.size(), 2U);
  EXPECT_FALSE(recorder.overflowed());
  EXPECT_EQ(recorder.data()[0].sequence, 1U);
  EXPECT_EQ(recorder.data()[1].sequence, 2U);
  EXPECT_LE(recorder.data()[0].entry_ns, recorder.data()[0].exit_ns);
  EXPECT_LE(recorder.data()[1].entry_ns, recorder.data()[1].exit_ns);
  EXPECT_EQ(recorder.data()[0].thread_id, recorder.data()[1].thread_id);
  EXPECT_LE(recorder.data()[0].exit_ns, recorder.data()[1].entry_ns);
}

TEST(PpCallbackSpanRecorder, OverflowIsStickyAndDoesNotOverwrite) {
  auto &recorder = diagnostic::CallbackSpanRecorder::instance();
  recorder.resetForTest();
  for (std::size_t index = 0;
       index < diagnostic::CallbackSpanRecorder::kCapacity + 1; ++index) {
    recorder.commit(diagnostic::CallbackKind::kKinematics, index, index,
                    1U);
  }

  ASSERT_EQ(recorder.size(), diagnostic::CallbackSpanRecorder::kCapacity);
  EXPECT_TRUE(recorder.overflowed());
  EXPECT_EQ(recorder.data()[0].sequence, 1U);
  EXPECT_EQ(
      recorder.data()[diagnostic::CallbackSpanRecorder::kCapacity - 1]
          .sequence,
      diagnostic::CallbackSpanRecorder::kCapacity);
}

TEST(PpCallbackSpanRecorder, DisabledScopeDoesNotReadClockOrCommit) {
  auto &recorder = diagnostic::CallbackSpanRecorder::instance();
  recorder.initialize(false);
  {
    diagnostic::CallbackSpanScope scope(
        diagnostic::CallbackKind::kKinematics);
  }

  EXPECT_FALSE(recorder.enabled());
  EXPECT_EQ(recorder.size(), 0U);
  EXPECT_FALSE(recorder.overflowed());
  recorder.resetForTest();
}
