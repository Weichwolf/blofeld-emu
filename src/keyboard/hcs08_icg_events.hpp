// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <cstdint>

namespace blofeld {
namespace hcs08_icg_events {
constexpr unsigned status_address = 0x4a, control_address = 0x49;
constexpr unsigned loss_lock = 1, loss_clock = 2, both_sources = loss_lock | loss_clock;
constexpr unsigned status_known = 0x15, interrupt_flag = 1;
constexpr unsigned lock_status = 0x10, clock_status = 4;
constexpr unsigned reset_on_lock = 0x80, reset_on_clock = 8;
}

enum class Hcs08IcgEventContract : std::uint8_t { unqualified, candidate_gt8a_fault_latches };
enum class Hcs08IcgFault : std::uint8_t {
  unqualified,
  loss_of_lock,
  loss_of_clock,
  loss_of_clock_and_lock
};
enum class Hcs08IcgFaultAction : std::uint8_t { none, interrupt, reset };

struct Hcs08IcgEventState {
  std::uint8_t sources = 0, reset_sources = 0;
  bool interrupt_read_armed = false;
  bool operator==(const Hcs08IcgEventState &) const = default;
};

struct Hcs08IcgFlagBits {
  unsigned value = 0, known = hcs08_icg_events::status_known;
  bool operator==(const Hcs08IcgFlagBits &) const = default;
};

// Fault latches only. The connected clock detector must establish each event.
// No oscillator, LOCK/DCOS, CLKST/REFST/ERCS or complete ICGS1 byte is supplied.
// Copies are joint in-process checkpoint state; they retain no callbacks/peers.
class Hcs08IcgEvents {
public:
  explicit Hcs08IcgEvents(Hcs08IcgEventContract contract);
  void context(ExecutionContext value);
  void reset();
  void control2(std::uint8_t value);
  void signal(Hcs08IcgFault event);
  [[nodiscard]] Hcs08IcgFlagBits read_status_flags();
  void write_status(std::uint8_t value);
  [[nodiscard]] Hcs08IcgEventState state() const;
  [[nodiscard]] Hcs08IcgFaultAction action() const;

private:
  [[nodiscard]] unsigned interrupt_sources() const;
  [[nodiscard]] unsigned flag_value() const;
  Hcs08IcgEventState state_;
  ExecutionContext context_;
};
} // namespace blofeld
