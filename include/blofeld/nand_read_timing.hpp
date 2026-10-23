// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/nand_array.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

namespace blofeld {
struct NandReadPins {
  std::optional<bool> ce_n;
  std::optional<bool> re_n;
};

struct NandReadOutput {
  // nullopt means that drive/Hi-Z is not established by the timing bounds.
  std::optional<bool> driven = std::nullopt;
  NandByte data{};
  bool operator==(const NandReadOutput &) const = default;
};

// K9F1208U0C Rev1.1, 3.3V array-read output bounds only. The caller supplies a
// monotonic reference timebase, actual CE/RE observations and each protocol
// byte. No chip selection, array completion, data or physical timing is guessed.
// Initial idle-high levels still require the output-release bound. Dynamic
// status, write-side timing and ready-to-read setup need separate contracts.
// Input/output observations advance reference time; copies retain value state.
class NandReadTiming {
public:
  using Source = std::function<NandByte()>;
  explicit NandReadTiming(std::uint32_t timebase_hz);
  // The source is called only for an admitted selected falling RE edge and is
  // never retained. Coordinate protocol and timing checkpoints in the owner.
  void observe(std::uint64_t ticks, NandReadPins pins, const Source &source = {});
  [[nodiscard]] NandReadOutput output(std::uint64_t ticks);

private:
  enum class Phase : std::uint8_t { floating, reading, releasing };
  [[noreturn]] void fail(const std::string &reason) const;
  [[nodiscard]] std::uint64_t deadline(std::uint64_t ticks,
                                       std::chrono::nanoseconds interval) const;
  void check_time(std::uint64_t ticks) const;
  void read_edge(std::uint64_t ticks, bool rising, const Source &source);
  void select_edge(std::uint64_t ticks, bool rising);
  void release(std::uint64_t ticks, std::chrono::nanoseconds maximum_ns);
  [[nodiscard]] NandReadOutput at(std::uint64_t ticks) const;
  std::uint32_t timebase_hz_ = 0;
  std::uint64_t ticks_ = 0;
  std::uint64_t selected_at_ = 0;
  std::uint64_t valid_at_ = 0;
  std::uint64_t hold_until_ = 0;
  std::uint64_t floating_at_ = 0;
  std::optional<std::uint64_t> fell_at_;
  std::optional<std::uint64_t> rose_at_;
  NandByte data_{};
  std::optional<bool> hold_driven_;
  Phase phase_ = Phase::floating;
  bool initialized_ = false;
  bool ce_n_ = true;
  bool re_n_ = true;
};
}
