// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/firmware.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::gpio_request {
inline constexpr unsigned vector = 0x10;
inline constexpr unsigned entry = 0x2662;
inline constexpr unsigned low_wait = 0x267e;
inline constexpr unsigned high_wait = 0x2682;
inline constexpr unsigned source_read = 0x269a;
inline constexpr unsigned data_port = 0xffffbd;
inline constexpr unsigned direction = 0xffffbe;
inline constexpr unsigned control = 0xffffbf;
inline constexpr unsigned word_mask = 0xffffff;
inline constexpr unsigned byte_mask = 0xff;
inline constexpr unsigned port_mask = 0xfff;
inline constexpr unsigned upper_mask = 0xf00;
inline constexpr unsigned ring = 0x2d00;
inline constexpr unsigned ring_words = 128;
inline constexpr unsigned table = 0x2d80;
inline constexpr unsigned table_base = 0x1200;
inline constexpr unsigned table_stride = 0x80;
inline constexpr unsigned canary = 0x2468ac;
inline constexpr unsigned request_seed = 0x135724;
inline constexpr unsigned deferred_seed = 0x123456;
inline constexpr unsigned flags_seed = 0x2200;
enum class Kind : std::uint8_t { part, multi, metadata, global, deferred };
enum class Stop : std::uint8_t { reply, low, high };
enum class Failure : std::uint8_t {
  none,
  low_unknown,
  second_unknown,
  index_unknown,
  word_unknown,
  contention
};

struct Case {
  Kind kind = Kind::part;
  unsigned first = 0;
  unsigned second = 0;
  unsigned address = table_base;
  unsigned value = 0;
  unsigned upper = 0;
  Stop stop = Stop::reply;
  Failure failure = Failure::none;
};

struct Access {
  DspNativeAccessKind kind = DspNativeAccessKind::read;
  DspNativeAccessStage stage = DspNativeAccessStage::before;
  DspSpace space = DspSpace::p;
  unsigned address = 0;
  unsigned pc = 0;
  std::optional<std::uint32_t> value;
  std::optional<DspNativeAccessBit> bit;
  bool operator==(const Access &) const = default;
};

struct Stats {
  unsigned steps = 0;
  unsigned low = 0;
  unsigned high = 0;
  unsigned input_reads = 0;
  unsigned source_reads = 0;
  unsigned table_reads = 0;
  unsigned releases = 0;
  std::optional<unsigned> reply;
  std::vector<std::array<unsigned, 2>> gpio;
  std::vector<Access> accesses;
  bool operator==(const Stats &) const = default;
};

struct Saved {
  DspCheckpoint machine;
  Stats stats;
  bool release_due = false;
};

struct Outcome {
  DspDebugState cpu;
  std::array<unsigned, 8> state;
  std::array<unsigned, 4> pads;
  Stats stats;
  bool operator==(const Outcome &) const = default;
};

class Rig {
public:
  Rig(const Firmware &firmware, Case input);
  [[nodiscard]] Saved save() const;
  void restore(const Saved &saved);
  [[nodiscard]] std::optional<Saved> turnaround();
  void finish();
  [[nodiscard]] Outcome outcome();

  [[nodiscard]] const Stats &stats() const { return stats_; }

private:
  void prepare(const Firmware &firmware);
  void observe(const DspNativeAccess &access);
  void observe_write(unsigned address, unsigned value);
  void observe_read(const DspNativeAccess &access, unsigned value);
  void observe_bit(const DspNativeAccess &access, unsigned value);
  void peer();
  void step();
  void check_reply();
  void check_guards();
  void check_failure(const std::string &message, DspState before);
  void cleanup();
  [[nodiscard]] unsigned saved_word() const;
  [[nodiscard]] unsigned expected_reply() const;
  DspMachine machine_;
  Case input_;
  DspState initial_;
  unsigned foreground_ = 0;
  Stats stats_;
  bool release_due_ = false;
};
}
