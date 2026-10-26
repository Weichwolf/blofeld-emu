// SPDX-License-Identifier: GPL-3.0-only
#include "clock_prefix_memory.hpp"

#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_cop.hpp"
#include "keyboard/hcs08_icg_events.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_system_options.hpp"
#include "keyboard/hcs08_tpm.hpp"

#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::test::clock_prefix {
namespace {
constexpr unsigned declared_bus_hz = 12000000, declared_supply_uv = 3300000, power_on_status = 0x80;

bool timer_register(const unsigned address) {
  return address >= Hcs08Tpm::status && address <= Hcs08Tpm::count_low;
}

bool cop_register(const unsigned address) {
  return address == Hcs08Cop::status || address == Hcs08SystemOptions::address;
}
} // namespace

Memory::Memory(const std::span<const std::uint8_t> pages, const Profile input)
    : core(pages), profile(input) {}

void Memory::configure_adc() {
  if (!profile.adc_reference) { return; }
  core.data.adc.observe_reference({.bus_hz = declared_bus_hz,
                                   .supply_uv = declared_supply_uv,
                                   .low_uv = 0,
                                   .high_uv = declared_supply_uv});
  core.data.adc.observe_input(0, 0);
}

void Memory::begin_instruction(const ExecutionContext value) {
  core.begin_instruction(value);
  cop.context(value);
}

void Memory::begin_reset(const ExecutionContext value) {
  core.begin_reset(value);
  core.timer.reset(profile.phase);
  core.data.adc.reset();
  configure_adc();
  cop.context(value);
  cop.reset(power_on_status);
}

void Memory::end_reset() {
  core.end_reset();
}

void Memory::retire_instruction() {
  core.retire_instruction();
}

std::optional<InterruptRequest> Memory::interrupt_request(const std::uint8_t ccr) const {
  return core.interrupt_request(ccr);
}

std::optional<InterruptRequest> Memory::interrupt_request(const Hcs08State &state) const {
  return core.interrupt_request(state);
}

void Memory::devices(const Hcs08State &state) {
  if (failed) { throw Fault("HCS08 coupled clock requires coordinated recovery"); }
  failed = true;
  ++attempted;
  const ExecutionContext context{.pc = state.pc, .ccr = state.ccr};
  core.history.context(context);
  core.timer.context(context);
  core.data.adc.context(context);
  cop.context(context);
  core.history.unobserved_interval();
  if (core.history.action(true) == Hcs08IcgFaultAction::reset) {
    throw Fault("HCS08 coupled clock requires coordinated ICG reset pc=" + hex(state.pc));
  }
  if (cop.advance_bus(1) != 1 || cop.state().reset_requested) {
    throw Fault("HCS08 coupled COP reset address=" + hex(Hcs08Cop::status) +
                " pc=" + hex(state.pc) + " ccr=" + hex(state.ccr));
  }
  core.timer.advance_bus(1);
  core.data.adc.advance_bus(1);
}

std::uint8_t Memory::read(const std::uint16_t address) {
  if (timer_register(address)) { return core.timer.read(address); }
  if (cop_register(address)) { return cop.read(address); }
  return core.read(address);
}

Hcs08KnownByte Memory::read_known_byte(const std::uint16_t address) {
  if (timer_register(address) || cop_register(address)) { return {.value = read(address)}; }
  return core.read_known_byte(address);
}

void Memory::service(const ByteWrite access, const std::uint8_t mask) {
  cop.write(access); // SRS ignores every data bit; unknown A remains unknown.
  if (access.address == Hcs08Cop::status) {
    ++services;
    service_known = mask;
  }
  ++core.data.generation;
}

void Memory::write(const std::uint16_t address, const std::uint8_t value) {
  if (cop_register(address)) {
    service({.address = address, .value = value}, Hcs08KnownByte::all);
    return;
  }
  core.write(address, value);
}

void Memory::write_known_byte(const ByteWrite access, const std::uint8_t mask) {
  if (access.address == Hcs08Cop::status) {
    service(access, mask);
  } else if (mask == Hcs08KnownByte::all) {
    write(access.address, access.value);
  } else {
    core.write_known_byte(access, mask);
  }
}

std::uint8_t Memory::peek(const std::uint16_t address) const {
  return core.peek(address);
}

std::uint64_t Memory::revision() const {
  return core.revision();
}

std::uint64_t Memory::code_revision() const {
  return core.code_revision();
}

void Memory::note_code_byte(const std::uint16_t address) const {
  core.note_code_byte(address);
}

void Memory::check_execute(const InstructionFetch fetch) const {
  core.check_execute(fetch);
}
} // namespace blofeld::test::clock_prefix
