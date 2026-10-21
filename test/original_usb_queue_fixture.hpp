// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/mcu_machine.hpp"
#include "usb_host_fixture.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace blofeld::test::usb_queue {
using Bytes = std::vector<std::uint8_t>;
using Registers = std::array<unsigned, 6>;
inline constexpr unsigned entry = 0xb3ed;
inline constexpr unsigned staged_pc = 0xb451;
inline constexpr unsigned wait_pc = 0xb463;
inline constexpr unsigned read_ccr = 0xb34f;
inline constexpr unsigned returned = 0x8000;
inline constexpr unsigned stack = 0x2200;
inline constexpr unsigned stack_low = stack - 24;
inline constexpr unsigned source = 0x1700;
inline constexpr unsigned ring = 0x18f2;
inline constexpr unsigned reader = 0x1972;
inline constexpr unsigned writer = 0x1973;
inline constexpr unsigned tick = 0x182e;
inline constexpr unsigned scan_flags = 0x1815;
inline constexpr unsigned ram_begin = 0x1200;
inline constexpr unsigned ram_bytes = 5120;
inline constexpr unsigned cursor_mask = 0x7c;
inline constexpr unsigned ring_bytes = 128;
inline constexpr unsigned event_bytes = 4;
inline constexpr unsigned endpoint_bytes = 64;
inline constexpr unsigned canary = 0xa5;
inline constexpr unsigned initial_tick = 0xfe;
inline constexpr unsigned usb_vector = 0xffca;
inline constexpr unsigned timer_vector = 0xfff0;
inline constexpr unsigned timer_period = 1024;
inline constexpr unsigned usb_handler = 0xbe6b;
inline constexpr unsigned timer_handler = 0x88a5;

struct Message {
  std::array<std::uint8_t, 3> data{};
  std::uint8_t header = 0;
};

struct Input {
  Message message;
  unsigned flags = 0;
  unsigned read = 0;
  unsigned write = 0;
  unsigned ccr = 0x40;
  bool busy = false;
  bool timer = false;
  bool raw_alias = false;
};

struct Stats {
  unsigned instructions = 0;
  unsigned usb_irqs = 0;
  unsigned timer_irqs = 0;
  unsigned ccr_calls = 0;
  std::vector<std::pair<unsigned, unsigned>> writes;
  bool operator==(const Stats &) const = default;
};

struct Saved {
  McuCheckpoint machine;
  Stats stats;
  bool staged = false;
  bool next_pid = false;
};

struct Outcome {
  Registers cpu;
  Bytes ram;
  Bytes buffer;
  unsigned control = 0;
  bool host_pid = false;
  std::optional<bool> device_pid = std::nullopt;
  Stats stats;
  bool operator==(const Outcome &) const = default;
};

struct ExpectedReturn {
  unsigned result = 0;
  unsigned read = 0;
  unsigned write = 0;
  unsigned masks = 0;
};

class Fixture {
public:
  Fixture(const Firmware &firmware, Input supplied);
  void step();
  void run();
  void until(unsigned pc, bool advance = false);
  void service(unsigned vector);
  void acknowledge();
  void drain(std::span<const std::uint8_t> expected);
  void check_return(ExpectedReturn expected);
  void guard() const;
  [[nodiscard]] Saved save() const;
  void restore(const Saved &saved);
  [[nodiscard]] Outcome outcome();
  [[nodiscard]] Bytes record() const;

  [[nodiscard]] McuMachine &board() { return machine; }

  [[nodiscard]] const Bytes &old_events() const { return prior; }

  [[nodiscard]] const Stats &counters() const { return stats; }

private:
  McuMachine machine;
  Input input;
  Bytes prior;
  Stats stats;

  void observe();
  void stage();
  Bytes before_ram_;
  McuFlashImage before_flash_;
  UsbHostEvidence evidence_;
  UsbConfigurationInfo configuration_;
  bool staged_ = false;
  bool next_pid_ = false;
};

void test_interrupts(const Firmware &firmware);

template <typename Operation> void replay(Fixture &fixture, const Operation &operation) {
  const auto saved = fixture.save();
  std::optional<Outcome> expected;
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    if (attempt != 0) { fixture.restore(saved); }
    operation(attempt);
    fixture.guard();
    const auto actual = fixture.outcome();
    if (!expected) { expected = actual; }
    usb_host::require(
        actual == *expected,
        "Two joint restores reproduce complete CPU/RAM/USB data and native event history");
  }
}
}
