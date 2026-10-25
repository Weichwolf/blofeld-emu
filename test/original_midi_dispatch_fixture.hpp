// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "firmware/firmware.hpp"
#include "mcu/mcu_machine.hpp"
#include "original_midi_forward_fixture.hpp"
#include "storage/flash_image.hpp"
#include "usb_host_fixture.hpp"

#include <optional>
#include <vector>

namespace blofeld::test::midi_dispatch {
using midi_forward::Bytes;
using midi_forward::Event;
inline constexpr unsigned entry = 0xb352;
inline constexpr unsigned source = 0x1700;
inline constexpr unsigned caller = 0x1649;
inline constexpr unsigned finished = caller + 4;

struct Call {
  unsigned pointer = 0;
  unsigned argument = 0;
  Bytes bytes;
  unsigned ordinal = 0;
  bool operator==(const Call &) const = default;
};

struct Case {
  Bytes source;
  Bytes mutated;
  Bytes tx;
  std::vector<Event> events;
  std::vector<unsigned> packets;
  std::vector<Call> uart_calls;
  std::vector<Call> usb_calls;
  unsigned selector = 0;
  unsigned cursor = 0;
  unsigned pointer_advance = 0;
  bool mirror = false;
  bool after_dispatch = false;
};

struct Saved {
  McuCheckpoint board;
  midi_forward::Stats stats;
  std::vector<Call> uart;
  std::vector<Call> usb;
  unsigned own = 0;
  bool pid = false;
};

struct Outcome {
  midi_forward::Outcome board;
  std::vector<Call> uart;
  std::vector<Call> usb;
  unsigned own = 0;
  bool operator==(const Outcome &) const = default;
};

class Rig {
public:
  Rig(const Firmware &firmware, Case supplied);
  void step();
  void until(unsigned pc);
  void capture_boundary();
  void complete();
  [[nodiscard]] Saved save() const;
  void restore(const Saved &saved);
  [[nodiscard]] Outcome outcome() const;

  [[nodiscard]] const midi_forward::Stats &stats() const { return stats_; }

  [[nodiscard]] unsigned own_instructions() const { return own_; }

private:
  void prepare();
  void observe();
  void record_call();
  void check_dispatch() const;
  void check_final() const;
  void drain();
  void guard() const;
  McuMachine machine_;
  Case input_;
  midi_forward::Stats stats_;
  std::vector<Call> uart_;
  std::vector<Call> usb_;
  Bytes before_;
  McuFlashImage flash_;
  UsbHostEvidence evidence_;
  UsbConfigurationInfo configuration_;
  unsigned own_ = 0;
  bool pid_ = false;
};
}
