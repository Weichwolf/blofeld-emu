// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_interrupts.hpp"

#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld::detail::hcs08 {
namespace {
constexpr unsigned mask = 8, first_vector = 0xffc0, last_vector = 0xfffa;

[[noreturn]] void fail(const Hcs08State &state, const unsigned address, const std::string &reason) {
  throw Fault("HCS08 IRQ query address=" + hex(address) +
              " pc=" + (state.pc_known ? hex(state.pc) : "unknown pc-known=0") +
              " ccr=" + hex(state.ccr) + " ccr-known=" + hex(state.ccr_known) + ": " + reason);
}
}

std::optional<std::uint16_t> pending_interrupt(const Bus &bus, const Hcs08State &state) {
  // Ask for local pending sources without substituting an unknown global I bit.
  // CPU masking is decided here; querying a source does not acknowledge it.
  const auto request = bus.interrupt_request(static_cast<std::uint8_t>(state.ccr & ~mask));
  if (!request) { return {}; }
  if (request->kind != InterruptKind::maskable) {
    fail(state, request->vector, "unsupported HCS08 interrupt kind");
  }
  if ((request->vector & 1U) != 0 || request->vector < first_vector ||
      request->vector > last_vector) {
    fail(state, request->vector, "invalid maskable vector (reset/SWI are not hardware IRQs)");
  }
  if ((state.ccr_known & mask) == 0) { fail(state, request->vector, "unknown CPU I mask"); }
  if ((state.ccr & mask) != 0) { return {}; }
  if (!state.pc_known) { fail(state, request->vector, "unknown stacked CPU PC"); }
  return request->vector;
}
}
