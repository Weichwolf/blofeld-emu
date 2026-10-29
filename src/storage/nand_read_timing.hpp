// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/nand_array.hpp"
#include "storage/nand_time_window.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

namespace blofeld {
// Logical status propagation projection; physical pad delay remains unqualified.
enum class NandStatusTiming : std::uint8_t { unqualified, reference_controller_boundary };

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
// First known RE-high with either CE level bounds prior edges/selection; no
// source is sampled until an actual later falling RE. Release still takes its
// full maximum, and the first new pulse waits out prior constraints. Dynamic
// status, write-side timing and ready-to-read setup need separate contracts.
// Input/output observations advance reference time; copies retain value state.
class NandReadTiming {
public:
  using Source = std::function<NandByte()>;
  bool operator==(const NandReadTiming &) const = default;
  explicit NandReadTiming(std::uint32_t timebase_hz);
  // The source is called only for an admitted selected falling RE edge and is
  // never retained. Coordinate protocol and timing checkpoints in the owner.
  void observe(std::uint64_t ticks, NandReadPins pins, const Source &source = {});
  [[nodiscard]] NandReadOutput output(std::uint64_t ticks);
  // Refresh the actual selected, held-low status read without a new pin edge.
  // Retains all access/pulse/release bounds; never samples an array byte.
  void refresh_status(std::uint64_t ticks, NandByte data, NandStatusTiming contract);
  // Next strictly future output-knowledge change if input pins stay unchanged.
  // A bound is not a physical edge. This query never advances time or a source.
  [[nodiscard]] std::optional<std::uint64_t> next_transition() const;

  // Requires initialized RE high and expired output/pulse constraints. The
  // caller proves inactive WE and unchanged non-CE controls/Ready in [start,end].
  // Resume records a conservative CE bound, never a manufactured pin edge.
  void qualify_ce_window(std::uint64_t start) const;
  void resume_ce_window(std::uint64_t start, std::uint64_t end, bool ce_n);

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
  NandTimeWindow selected_at_{};
  NandTimeWindow valid_at_{};
  std::uint64_t hold_until_ = 0;
  std::uint64_t floating_at_ = 0;
  // Initial values bound latest unobserved prior edges; later values are exact.
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
