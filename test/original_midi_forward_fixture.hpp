// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "firmware/firmware.hpp"
#include "mcu/mcu_machine.hpp"
#include "storage/flash_image.hpp"
#include "usb_host_fixture.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace blofeld::test::midi_forward {
using Bytes = std::vector<std::uint8_t>;
using Event = std::array<std::uint8_t, 4>;
inline constexpr unsigned entry = 0xb4aa;
inline constexpr unsigned queue_entry = 0xb3ed;
inline constexpr unsigned wait_pc = 0xb463;
inline constexpr unsigned publish_pc = 0xb522;
inline constexpr unsigned returned = 0x8000;
inline constexpr unsigned stack = 0x2200;
inline constexpr unsigned stack_bytes = 40;
inline constexpr unsigned ram_begin = 0x1200;
inline constexpr unsigned ram_bytes = 5120;
inline constexpr unsigned tx_ring = 0x18ae;
inline constexpr unsigned tx_reader = 0x18ee;
inline constexpr unsigned tx_writer = 0x18ef;
inline constexpr unsigned mirror_reader = 0x18f0;
inline constexpr unsigned peer_ring = 0x1974;
inline constexpr unsigned peer_writer = 0x19b4;
inline constexpr unsigned peer_reader = 0x19b5;
inline constexpr unsigned usb_ring = 0x18f2;
inline constexpr unsigned usb_reader = 0x1972;
inline constexpr unsigned usb_writer = 0x1973;
inline constexpr unsigned ring_mask = 0x3f;
inline constexpr unsigned usb_mask = 0x7c;
inline constexpr unsigned canary = 0xa5;
inline constexpr unsigned independent_reader = 37;
inline constexpr unsigned peer_cursor = 63;
inline constexpr unsigned endpoint_bytes = 64;
inline constexpr unsigned usb_vector = 0xffca;
inline constexpr unsigned sci_vector = 0xffd6;

struct Input {
  Bytes bytes;
  unsigned cursor = 0;
  unsigned usb_cursor = 0;
  unsigned ccr = 0x40;
  std::optional<Event> event = std::nullopt;
  bool busy = false;
  bool full = false;
  bool peer = false;
  bool raw_alias = false;
};

struct Stats {
  unsigned instructions = 0;
  unsigned usb_irqs = 0;
  unsigned sci_irqs = 0;
  unsigned usb_calls = 0;
  unsigned uart_calls = 0;
  unsigned publications = 0;
  std::vector<std::pair<unsigned, unsigned>> writes;
  bool operator==(const Stats &) const = default;
};

struct Saved {
  McuCheckpoint machine;
  Stats stats;
  bool next_pid = false;
};

struct Outcome {
  std::array<unsigned, 6> cpu;
  Bytes ram;
  Bytes usb_buffer;
  std::array<unsigned, 8> sci;
  unsigned control = 0;
  bool host_pid = false;
  std::optional<bool> device_pid = std::nullopt;
  Stats stats;
  bool operator==(const Outcome &) const = default;
};

class Fixture {
public:
  Fixture(const Firmware &firmware, Input input);
  void step();
  void run();
  void until(unsigned pc, bool advance = false);
  void acknowledge();
  void service_usb();
  void drain(std::span<const std::uint8_t> expected);
  void check_return(unsigned expected_cursor) const;
  void guard() const;
  [[nodiscard]] Saved save() const;
  void restore(const Saved &saved);
  [[nodiscard]] Outcome outcome() const;

  [[nodiscard]] McuMachine &board() { return machine_; }

  [[nodiscard]] const Stats &counters() const { return stats_; }

  [[nodiscard]] const Bytes &old_events() const { return prior_; }

private:
  void observe();
  void prepare();
  void before_step();
  McuMachine machine_;
  Input input_;
  Stats stats_;
  Bytes prior_;
  Bytes before_ram_;
  McuFlashImage before_flash_;
  UsbHostEvidence evidence_;
  UsbConfigurationInfo configuration_;
  bool next_pid_ = false;
};
}
