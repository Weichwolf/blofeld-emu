// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_bdc.hpp"
#include "blofeld/hcs08_bdc_wire.hpp"
#include "blofeld/hcs08_flash.hpp"
#include "hcs08_bdc_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"

#include <cstdint>

namespace blofeld::test::hcs08_wire {
constexpr auto chip = Hcs08FlashContract::candidate_gt8a;
constexpr auto wire_contract = Hcs08BdcWireContract::reference_command_delay;
constexpr Hcs08BdcWireClock wire_clock{.period_ticks = 1, .first_edge_ticks = 1};
constexpr auto released = Hcs08BdcDrive::released;
constexpr auto high = Hcs08BdcDrive::high, low = Hcs08BdcDrive::low;
constexpr unsigned algorithm = 0xa0, seed = 0x1200, stack = 0x47c;

struct Target {
  hcs08_programmer::Memory memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};
  hcs08_bdc::Memory mapping{memory};

  explicit Target(const hcs08_programmer::Memory *snapshot = nullptr)
      : memory(snapshot != nullptr ? *snapshot : initial_memory()) {
    cpu.set_state({.pc = algorithm, .hx = seed, .sp = stack});
  }

  static hcs08_programmer::Memory initial_memory() {
    hcs08_programmer::Memory memory(Hcs08IdleLaunch::unqualified, 5);
    memory.flash.reset(); // No absolute target NVM clock is observed by this wire fixture.
    return memory;
  }
};

inline void clocks(Hcs08BdcWire &wire, const unsigned count) {
  for (unsigned n = 0; n < count; ++n) { wire.clock(); }
}

// Own independent protocol stimulus: sample at the documented tenth cycle;
// sixteen complete target cycles plus the next-edge synchronization uncertainty.
inline void send(Hcs08BdcWire &wire, const std::uint8_t byte) {
  constexpr unsigned byte_bits = 8;
  constexpr unsigned high_bit = 0x80;
  for (unsigned bit = 0; bit < byte_bits; ++bit) {
    wire.host_drive(low, {.pc = algorithm});
    clocks(wire, 4);
    if (((static_cast<unsigned>(byte) << bit) & high_bit) != 0) {
      wire.host_drive(high, {.pc = algorithm});
    }
    clocks(wire, 9);
    wire.host_drive(high, {.pc = algorithm});
    clocks(wire, 4);
  }
}

inline std::uint8_t receive(Hcs08BdcWire &wire) {
  constexpr unsigned byte_bits = 8;
  constexpr unsigned byte_mask = 255;
  unsigned value = 0;
  for (unsigned bit = 0; bit < byte_bits; ++bit) {
    wire.host_drive(low, {.pc = algorithm});
    clocks(wire, 3);
    wire.host_drive(released, {.pc = algorithm});
    clocks(wire, 8);
    value = ((value << 1U) | (wire.level() ? 1U : 0U)) & byte_mask;
    clocks(wire, 6);
  }
  return static_cast<std::uint8_t>(value);
}
} // namespace blofeld::test::hcs08_wire
