// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/nand_array.hpp"
#include "storage/nand_time_window.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
struct NandWritePins {
  std::optional<bool> ce_n;
  std::optional<bool> re_n;
  std::optional<bool> we_n;
  std::optional<bool> cle;
  std::optional<bool> ale;
  NandByte data{};
};

enum class NandWriteKind : std::uint8_t { command, address, data };

struct NandWriteLatch {
  NandWriteKind kind;
  std::uint8_t value;
  bool operator==(const NandWriteLatch &) const = default;
};

// K9F1208U0C Rev1.1, 3.3V input timing, table12 and diagrams19/20.
// Complete ordered pin observations are required, including changes between WE
// edges. A returned latch occurs at rising WE; later hold violations still fail.
// Protocol effects, read turnaround, busy timing and board routing are separate.
class NandWriteTiming {
public:
  explicit NandWriteTiming(std::uint32_t timebase_hz);
  bool operator==(const NandWriteTiming &) const = default;
  [[nodiscard]] std::optional<NandWriteLatch> observe(std::uint64_t ticks, NandWritePins pins);

  // Prove quiescence before forgetting CE transitions; resume retains a latest
  // possible selection bound, not a fabricated observed edge.
  void qualify_ce_window(std::uint64_t start) const;
  void resume_ce_window(std::uint64_t start, std::uint64_t end, bool ce_n);

private:
  struct KnownPins {
    bool ce_n;
    bool re_n;
    bool we_n;
    bool cle;
    bool ale;
    NandByte data;
    bool operator==(const KnownPins &) const = default;
  };

  [[noreturn]] void fail(const std::string &reason) const;
  [[nodiscard]] std::uint64_t deadline(std::uint64_t ticks,
                                       std::chrono::nanoseconds interval) const;
  void changed(std::uint64_t ticks, const KnownPins &pins);
  [[nodiscard]] std::optional<NandWriteLatch> edge(std::uint64_t ticks, const KnownPins &pins);
  [[nodiscard]] NandWriteLatch latch(std::uint64_t ticks, const KnownPins &pins) const;
  std::uint32_t timebase_hz_;
  std::uint64_t ticks_ = 0;
  // nullopt is a retired constraint: its minimum is met for every future time.
  std::array<std::optional<NandTimeWindow>, 4> stable_since_{};
  std::optional<std::uint64_t> fell_at_;
  std::optional<std::uint64_t> rose_at_;
  std::optional<std::uint64_t> high_since_ = 0;
  unsigned low_changes_ = 0;
  KnownPins pins_{.ce_n = true, .re_n = true, .we_n = true, .cle = false, .ale = false, .data = {}};
  bool initialized_ = false;
};
}
