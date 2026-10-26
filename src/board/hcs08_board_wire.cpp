// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board_wire.hpp"

#include "board/gpio_pads.hpp"
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_run_clock.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>

namespace blofeld {
Hcs08BoardWireCheckpoint::Hcs08BoardWireCheckpoint(const Hcs08BoardCheckpoint &board,
                                                   const Hcs08BdcWireCheckpoint &wire,
                                                   const Hcs08BdcWireClock clock)
    : board_(board), wire_(wire), clock_(clock) {}

Hcs08BoardWire::Hcs08BoardWire(const Hcs08BoardResetCheckpoint &reset,
                               const std::filesystem::path &specification,
                               const Hcs08BoardInputs &board_inputs,
                               const Hcs08BoardWireInputs &wire_inputs)
    : board_(reset, specification, board_inputs),
      wire_(board_.bdc_, wire_inputs.contract, wire_inputs.initial_host, wire_inputs.clock),
      clock_(wire_inputs.clock) {}

Hcs08BoardWire::Hcs08BoardWire(const Hcs08BoardWireCheckpoint &checkpoint,
                               const std::filesystem::path &specification)
    : board_(checkpoint.board_, specification),
      // A released constructor supplies no falling edge,reply consumption or service.
      // The trusted wire checkpoint then restores its actual partial-bit ownership.
      wire_(board_.bdc_,
            Hcs08BdcWireContract::reference_command_delay,
            Hcs08BdcDrive::released,
            checkpoint.clock_),
      clock_(checkpoint.clock_) {
  wire_.restore_checkpoint(checkpoint.wire_);
}

void Hcs08BoardWire::healthy() const {
  if (faulted()) {
    fail("access", "incomplete target/wire transaction requires coordinated recovery");
  }
}

[[noreturn]] void Hcs08BoardWire::fail(const std::string &operation,
                                       const std::string &reason) const {
  const auto cpu = board_.cpu_state();
  const auto wire = wire_.state();
  throw Fault("HCS08 board BKGD " + operation +
              " address=BKGD pc=" + (cpu.pc_known ? hex(cpu.pc) : "unknown pc-known=0") +
              " ccr=" + hex(cpu.ccr) + " ccr-known=" + hex(cpu.ccr_known) +
              " agent=BDC cycles=" + std::to_string(wire.cycles) +
              " time=" + std::to_string(wire.time_ticks) + ": " + reason);
}

void Hcs08BoardWire::begin_wire() {
  healthy();
  healthy_ = false;
  board_.healthy_ = false;
  if (board_.state_.phase != Hcs08BoardPhase::boundary) {
    fail("access", "target period is not at its external boundary");
  }
  board_.state_.phase = Hcs08BoardPhase::bdc;
}

void Hcs08BoardWire::end_wire() {
  board_.state_.phase = Hcs08BoardPhase::boundary;
  board_.healthy_ = true;
  healthy_ = true;
}

void Hcs08BoardWire::host_drive(const Hcs08BdcDrive drive,
                                const ExecutionContext context,
                                const std::optional<std::uint64_t> time_ticks) {
  begin_wire();
  try {
    wire_.host_drive(drive, context, time_ticks);
  } catch (const Fault &error) { fail("host drive", error.what()); }
  end_wire();
}

void Hcs08BoardWire::clock_bdc() {
  begin_wire();
  const auto pending = wire_.state();
  if (pending.service_due && pending.next_clock_ticks >= *pending.service_due &&
      board_.run_state().remaining_periods != 0) {
    fail("service", "native instruction budget is pending;wire service grant unqualified");
  }
  try {
    wire_.clock(); // No fabricated running-memory grant is passed to native BDC.
  } catch (const Fault &error) { fail("clock", error.what()); }
  end_wire();
}

void Hcs08BoardWire::advance_reference_period() {
  healthy();
  healthy_ = false;
  board_.advance_reference_period();
  healthy_ = true;
}

void Hcs08BoardWire::attach_pad_source(const Hcs08GpioPort port, GpioPadReader reader) {
  healthy();
  healthy_ = false;
  board_.attach_pad_source(port, std::move(reader));
  healthy_ = true;
}

bool Hcs08BoardWire::level() const {
  healthy();
  return wire_.level();
}

Hcs08BdcDrive Hcs08BoardWire::target_drive() const {
  healthy();
  return wire_.target_drive();
}

Hcs08BoardState Hcs08BoardWire::board_state() const {
  return board_.state();
}

Hcs08State Hcs08BoardWire::cpu_state() const {
  return board_.cpu_state();
}

Hcs08RunState Hcs08BoardWire::run_state() const {
  return board_.run_state();
}

Hcs08BdcState Hcs08BoardWire::bdc_state() const {
  return board_.bdc_state();
}

Hcs08BdcWireState Hcs08BoardWire::wire_state() const {
  return wire_.state();
}

const Hcs08BoardMemory &Hcs08BoardWire::memory() const {
  return board_.memory();
}

bool Hcs08BoardWire::faulted() const {
  return !healthy_ || board_.faulted() || wire_.faulted();
}

Hcs08BoardWireCheckpoint Hcs08BoardWire::checkpoint() const {
  healthy();
  return {board_.checkpoint(), wire_.checkpoint(), clock_};
}

Hcs08BoardResetCheckpoint
Hcs08BoardWire::reset_checkpoint(const Hcs08BoardResetRequest &request) const {
  return board_.reset_checkpoint(request);
}
} // namespace blofeld
