// SPDX-License-Identifier: GPL-3.0-only
#include "history_startup_memory.hpp"

#include "common/bus.hpp"
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_icg.hpp"
#include "keyboard/hcs08_icg_events.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_tpm.hpp"

#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::test::icg_history {
namespace hi = hcs08_icg;

namespace {
constexpr unsigned interrupt_mask = 8, calibration = 0x80, strap = 8;
constexpr unsigned timer_running = 8;

bool status(const unsigned address) {
  return address == hi::status1 || address == hi::status2;
}

bool timer_register(const unsigned address) {
  return address >= Hcs08Tpm::status && address <= Hcs08Tpm::count_low;
}
} // namespace

Memory::Memory(const std::span<const std::uint8_t> pages)
    : data(pages, {.seed = 0, .trim = calibration, .observation = strap}) {
  data.warm_reset();
  data.icg.reset(Hcs08IcgReset::power_on);
}

void Memory::begin_instruction(const ExecutionContext value) {
  data.begin_instruction(value);
  history.context(value);
  history.unobserved_interval();
  hcs08::require(history.action(true) != Hcs08IcgFaultAction::reset,
                 "A reset requires coordinated peripheral/CPU handling before execution");
  timer.context(value);
}

void Memory::begin_reset(const ExecutionContext value) {
  data.begin_reset(value);
  history.context(value);
  history.reset();
  timer.context(value);
  timer.reset();
}

void Memory::end_reset() {
  data.end_reset();
}

void Memory::retire_instruction() {
  data.retire_instruction();
}

std::optional<InterruptRequest> Memory::interrupt_request(const std::uint8_t ccr) const {
  throw Fault("HCS08 uncertain IRQ query requires actual CPU mask knowledge ccr=" + hex(ccr));
}

std::optional<InterruptRequest> Memory::interrupt_request(const Hcs08State &state) const {
  auto observed = history;
  observed.context({.pc = state.pc, .ccr = state.ccr});
  const bool masked = (state.ccr_known & interrupt_mask) != 0 && (state.ccr & interrupt_mask) != 0;
  hcs08::require(observed.action(masked) == Hcs08IcgFaultAction::none,
                 "A known ICG action requires a coordinated CPU interrupt/reset owner");
  return data.interrupt_request(static_cast<std::uint8_t>(state.ccr & ~interrupt_mask));
}

std::uint8_t Memory::read(const std::uint16_t address) {
  if (status(address)) { return history.read(address); }
  if (address == hi::control1 || address == hi::control2) {
    throw Fault("HCS08 ICG history read address=" + hex(address) + " pc=" + hex(data.context.pc) +
                " ccr=" + hex(data.context.ccr) +
                " agent=CPU: software requests do not establish live control readback");
  }
  if (!timer_register(address)) { return data.read(address); }
  if ((timer.state().control & timer_running) != 0) {
    throw Fault("HCS08 TPM live read address=" + hex(address) + " pc=" + hex(data.context.pc) +
                " ccr=" + hex(data.context.ccr) +
                " agent=CPU: actual clock history and prescaler phase are unqualified");
  }
  return timer.read(address);
}

Hcs08KnownByte Memory::read_known_byte(const std::uint16_t address) {
  if (status(address)) {
    const auto byte = history.read_known(address);
    data.peripheral_accesses.push_back(
        {.address = address, .value = byte.value, .write = false, .known = byte.known});
    return byte;
  }
  if (timer_register(address) || address == hi::control1 || address == hi::control2) {
    return {.value = read(address)};
  }
  return data.read_known_byte(address);
}

void Memory::write(const std::uint16_t address, const std::uint8_t value) {
  if (status(address)) {
    history.write({.address = address, .value = value});
  } else if (timer_register(address)) {
    timer.write({.address = address, .value = value});
  } else if (address == hi::control2) {
    auto next = history;
    next.control2(value);
    data.write(address, value);
    history = next;
    return;
  } else {
    data.write(address, value);
    return;
  }
  data.peripheral_accesses.push_back({.address = address, .value = value, .write = true});
  ++data.generation;
}

void Memory::write_known_byte(const ByteWrite access, const std::uint8_t mask) {
  if (mask == Hcs08KnownByte::all) {
    write(access.address, access.value);
  } else if (status(access.address)) {
    history.write(access, mask);
  } else {
    data.write_known_byte(access, mask);
  }
}

std::uint8_t Memory::peek(const std::uint16_t address) const {
  return data.peek(address);
}

std::uint64_t Memory::revision() const {
  return data.revision();
}

std::uint64_t Memory::code_revision() const {
  return data.code_revision();
}

void Memory::note_code_byte(const std::uint16_t address) const {
  data.note_code_byte(address);
}

void Memory::check_execute(const InstructionFetch fetch) const {
  data.check_execute(fetch);
}
} // namespace blofeld::test::icg_history
