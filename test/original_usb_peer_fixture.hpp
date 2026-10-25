// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"
#include "original_midi_forward_fixture.hpp"
#include "storage/flash_image.hpp"
#include "usb_host_fixture.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace blofeld::test::usb_peer {
using midi_forward::Bytes;
using midi_forward::Event;
inline constexpr unsigned foreground = 0x8000;
inline constexpr unsigned source = 0x1700;
inline constexpr unsigned stack = 0x2200;
inline constexpr unsigned forward_entry = 0xb8b0;
inline constexpr unsigned frame_entry = 0xb870;
inline constexpr unsigned dispatch_entry = 0xb352;
inline constexpr unsigned strobe = 0xb6d0;
inline constexpr unsigned payload_load = 0xb8fe;
inline constexpr unsigned out_handler = 0xbe7a;
inline constexpr unsigned out_vector = 0xffc8;
inline constexpr unsigned out_window = 0x3c0;
inline constexpr unsigned activity = 0x1812;
inline constexpr unsigned bus_flags = 0x1815;
inline constexpr unsigned transfer_flags = 0x1831;
inline constexpr unsigned initial_activity = 0xa1;
inline constexpr unsigned initial_bus_flags = 0x21;
enum class Mode : std::uint8_t { usb, forward, frame, dispatch };

struct Case {
  Mode mode = Mode::usb;
  Bytes packet;
  Bytes expected;
  std::vector<unsigned> headers;
  std::vector<unsigned> lengths;
  unsigned count = 4;
  unsigned phase = 5;
  unsigned ccr = 0x40;
  bool rtie = false;
  bool repeat = false;
  bool missing_j = false;
  bool truncated = false;
  bool activity = false;
  bool multi = false;
  bool clear_tail = false;
};

struct Write {
  unsigned address = 0;
  unsigned value = 0;
  unsigned pc = 0;
  std::array<unsigned, 8> pads{};
  unsigned q_direction = 0;
  bool operator==(const Write &) const = default;
};

struct Stats {
  unsigned instructions = 0;
  unsigned irqs = 0;
  unsigned strobes = 0;
  std::vector<Write> writes;
  bool operator==(const Stats &) const = default;
};

struct Saved {
  McuCheckpoint board;
  Stats stats;
};

struct Outcome {
  std::array<unsigned, 6> cpu;
  Bytes ram;
  std::vector<unsigned> buffers;
  std::array<unsigned, 8> pads;
  unsigned control = 0;
  unsigned q_direction = 0;
  std::optional<bool> data1 = std::nullopt;
  Stats stats;
  bool operator==(const Outcome &) const = default;
};

class Rig {
public:
  Rig(const Firmware &firmware, Case input);
  void capture_boundary();
  void complete();
  [[nodiscard]] Saved save() const;
  void restore(const Saved &saved);
  [[nodiscard]] Outcome outcome() const;

  [[nodiscard]] const Stats &stats() const { return stats_; }

private:
  void prepare();
  void observe();
  void check_write(const Write &write);
  [[nodiscard]] unsigned expected_preload() const;
  void check_consumed_header();
  void check_pads(unsigned phase) const;
  void step();
  void until(unsigned pc);
  void receive(bool data1);
  void duplicate(bool data1);
  void check_frame() const;
  void check_result() const;
  void check_failure();
  void guard() const;
  void guard_buffers() const;
  [[nodiscard]] std::vector<std::pair<unsigned, unsigned>> expected_writes() const;
  McuMachine machine_;
  Case input_;
  CpuState initial_;
  Stats stats_;
  Bytes before_;
  McuFlashImage flash_;
  UsbHostEvidence evidence_;
  UsbConfigurationInfo configuration_;
  unsigned active_pc_ = 0;
};
}
