// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_program_fetch.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
// Speculative fetch control only. The native CPU owns all architectural changes.
struct DspLoopArchitecture {
  std::uint32_t pc = 0;
  std::uint32_t omr = 0;
  std::uint32_t la = 0;
  std::uint32_t lc = 0;
  std::uint32_t sp = 0;
  std::uint32_t sc = 0;
  std::uint32_t status = 0; // SR without CCR.
  bool operator==(const DspLoopArchitecture &) const = default;
};

struct DspLoopOpening {
  DspProgramWord word;
  DspProgramWord extension;
  DspLoopArchitecture before;
  DspLoopArchitecture after;
};

struct DspSubroutineControl {
  DspLoopArchitecture before;
  DspLoopArchitecture after;
  bool operator==(const DspSubroutineControl &) const = default;
};

struct DspLoopBoundary {
  DspProgramWord last;
  std::uint64_t occurrence = 0;
  DspLoopArchitecture expected;
  bool backedge = false;
  bool operator==(const DspLoopBoundary &) const = default;
};

// Copyable checkpoint state; current core-owner tickets must guard its consumers.
class DspLoopFetch {
  struct Frame {
    DspLoopArchitecture restore;
    std::uint32_t body = 0;
    std::uint32_t last = 0;
    bool forever = false;
    std::size_t calls_at_entry = 0;
  };

  struct Sequence {
    std::atomic<std::uint64_t> serial{0};
  };

public:
  class Prediction {
  public:
    [[nodiscard]] std::optional<std::uint32_t> loop_last() const {
      if (depth_ == 0) { return std::nullopt; }
      return frames_.at(depth_ - 1).last;
    }

  private:
    friend class DspLoopFetch;
    std::array<Frame, dsp_program_fetch_capacity> frames_{};
    DspLoopArchitecture forecast_;
    std::shared_ptr<Sequence> sequence_;
    std::uint64_t cut_ = 0;
    std::uint64_t revision_ = 0;
    std::size_t depth_ = 0;
    std::size_t calls_ = 0;
  };

  explicit DspLoopFetch(DspLoopArchitecture initial);
  void opened(const DspLoopOpening &opening);
  void check_fetch(std::uint32_t address) const;
  [[nodiscard]] std::optional<DspLoopBoundary> fetched(DspProgramWord word);
  void confirm(const DspLoopBoundary &boundary, DspLoopArchitecture native_result);
  [[nodiscard]] Prediction prediction() const;
  void discard_after(const Prediction &prediction, std::uint32_t next_pc);
  void predict_subroutine(DspProgramWord word, const DspSubroutineControl &control, bool call);

  [[nodiscard]] std::uint32_t next_address() const { return next_; }

  [[nodiscard]] std::size_t depth() const { return depth_; }

  [[nodiscard]] std::size_t pending() const { return pending_; }

private:
  [[nodiscard]] std::uint64_t next_occurrence();
  // Host continuation bounds follow the existing16-entry stack/eight-word queue.
  std::array<Frame, dsp_program_fetch_capacity> frames_{};
  std::array<DspLoopBoundary, dsp_program_fetch_capacity> boundaries_{};
  DspLoopArchitecture forecast_;
  std::uint32_t next_ = 0;
  std::uint64_t occurrence_ = 0;
  // Shared monotonic identity survives copied/restored speculative state.
  std::shared_ptr<Sequence> sequence_ = std::make_shared<Sequence>();
  std::array<std::uint64_t, dsp_program_fetch_capacity> history_{};
  std::uint64_t confirmed_ = 0;
  std::uint64_t revision_ = 0;
  std::size_t depth_ = 0;
  std::size_t pending_ = 0;
  std::size_t calls_ = 0;
};
}
