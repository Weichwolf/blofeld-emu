// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/firmware.hpp"

#include <array>
#include <optional>
#include <span>
#include <string>

namespace blofeld::test::keyboard_short {
constexpr unsigned flags = 6, context = 0x22, other_context = 0x14f6;
constexpr unsigned kind = 0x28, count = 0x29, selector = 0x50, report = 0x76c;
constexpr unsigned buffer = 0x778, single_byte = 0xbe2, append = 0xcaf, idle = 0xc5c;
constexpr unsigned universal = 0xaa5, dispatch_return = 0xaf1;
constexpr unsigned bulk = 0x1000, secondary = 0x10, sentinel = 0x2468ac;
constexpr unsigned raw_start = 0xf0, raw_data = 0x70, raw_end = 0xf7, byte_shift = 16;
constexpr unsigned manufacturer_word = raw_data << byte_shift, step_limit = 2048;
constexpr unsigned parameter_program = 0x808, message_program = 0x967;
constexpr unsigned x_limit = 0x3000, y_limit = 0x2000, absolute_jump = 0x0af080;
constexpr unsigned identity_word = 0x7e0601;
constexpr std::array<unsigned, 3> stale_pc{0xaa5, 0xaa6, 0xaa8};
constexpr std::array<unsigned, 6> values{0, 1, 0x7fffff, 0x800000, 0xfffffe, 0xffffff};

struct Ring {
  unsigned base, size, producer, consumer, entry;
};

constexpr std::array rings{
    Ring{.base = 0x2f80, .size = 64, .producer = 0x3b, .consumer = 0x3c, .entry = 0xc53},
    Ring{.base = 0x2d00, .size = 128, .producer = 0x32, .consumer = 0x33, .entry = 0xc5d}};

struct Events {
  unsigned reads = 0, entries = 0, returns = 0;
};

struct Saved {
  DspCheckpoint owner;
  Events events;
};

inline std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected strict original short-message rejection");
}

class Fixture {
public:
  Fixture(const Firmware &firmware, unsigned route, std::optional<unsigned> missing = std::nullopt);
  void values(unsigned u, unsigned v);
  void enqueue(std::span<const unsigned> bytes);
  void invoke(unsigned entry);
  void verify(unsigned remaining_flags = 0);
  void blocked();
  void recover_missing();
  void resume_blocked();
  void intact(const Firmware &firmware);

  [[nodiscard]] unsigned result() const { return manufacturer_word + (u_ >> 8U) + (v_ >> 16U); }

private:
  [[nodiscard]] unsigned read(unsigned address);
  void write(unsigned address, unsigned value);
  void install(const Firmware &firmware);
  void run();
  void observe(const DspNativeAccess &access);
  void check_context(bool completed);
  DspMachine machine_;
  unsigned route_, u_ = 0, v_ = 0, complete_ = 0;
  std::optional<unsigned> missing_;
  std::optional<Saved> saved_;
  Events events_;
};
} // namespace blofeld::test::keyboard_short
