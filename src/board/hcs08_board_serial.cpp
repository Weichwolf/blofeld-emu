// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board_serial.hpp"

#include "board/gpio_pads.hpp"
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_clock.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"

#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace blofeld {
Hcs08BoardSerialCheckpoint::Hcs08BoardSerialCheckpoint(const Hcs08BoardCheckpoint &board,
                                                       const Hcs08SpiClockCheckpoint &spi,
                                                       const Hcs08SpiRunCheckpoint &run,
                                                       const Hcs08BoardSerialContract contract)
    : board_(board), spi_(spi), run_(run), contract_(contract) {}

Hcs08BoardSerial::Hcs08BoardSerial(const Hcs08BoardCheckpoint &board,
                                   const std::filesystem::path &specification,
                                   const Hcs08BoardSerialContract contract)
    : board_(board, specification),
      spi_(board_.memory_.spi_, Hcs08SpiClockContract::reference_half_period_and_final_edge),
      run_(board_.bdc_, spi_, contract.order, contract.launch, contract.program),
      contract_(contract) {
  if (board_.clock_.state().remaining_periods != 0 || !board_.cpu_.state().pc_known) {
    fail("requires a complete known-PC CPU boundary without a foreign instruction reservation");
  }
}

Hcs08BoardSerial::Hcs08BoardSerial(const Hcs08BoardSerialCheckpoint &checkpoint,
                                   const std::filesystem::path &specification)
    : board_(checkpoint.board_, specification),
      spi_(board_.memory_.spi_, checkpoint.spi_, context()),
      run_(board_.bdc_,
           spi_,
           checkpoint.contract_.order,
           checkpoint.contract_.launch,
           checkpoint.contract_.program),
      contract_(checkpoint.contract_) {
  run_.restore_checkpoint(checkpoint.run_);
}

ExecutionContext Hcs08BoardSerial::context() const {
  const auto cpu = board_.cpu_state();
  return {.pc = cpu.pc, .ccr = cpu.ccr};
}

[[noreturn]] void Hcs08BoardSerial::fail(const std::string &reason) const {
  const auto cpu = board_.cpu_state();
  throw Fault("HCS08 serial board operation address=" + hex(hcs08_spi::status) +
              " pc=" + (cpu.pc_known ? hex(cpu.pc) : "unknown pc-known=0") +
              " ccr=" + hex(cpu.ccr) + " ccr-known=" + hex(cpu.ccr_known) + " agent=SPI periods=" +
              std::to_string(board_.state().attempted_periods) + ": " + reason);
}

void Hcs08BoardSerial::healthy() const {
  if (faulted()) { fail("incomplete native board/serial operation requires coordinated recovery"); }
}

std::optional<Hcs08SpiPins>
Hcs08BoardSerial::advance_reference_period(const std::optional<bool> miso,
                                           const Hcs08BoardClockObservation observation) {
  healthy();
  healthy_ = false;
  board_.healthy_ = false;
  if (board_.state_.phase != Hcs08BoardPhase::boundary || board_.memory_.serial_period_owned_) {
    fail("reference period already has a board/serial owner");
  }
  if (board_.state_.attempted_periods == std::numeric_limits<std::uint64_t>::max()) {
    fail("board period horizon exhausted");
  }
  ++board_.state_.attempted_periods;
  board_.state_.phase = Hcs08BoardPhase::devices;
  board_.memory_.serial_period_owned_ = true;
  board_.memory_.observed_cpu_period_ = observation != Hcs08BoardClockObservation::unobserved;
  try {
    board_.memory_.advance_reference_bus(context(), observation);
    board_.state_.phase = Hcs08BoardPhase::cpu;
    const auto pins = run_.advance(miso);
    board_.memory_.serial_period_owned_ = false;
    board_.memory_.observed_cpu_period_ = false;
    ++board_.state_.completed_periods;
    board_.state_.phase = Hcs08BoardPhase::boundary;
    board_.healthy_ = true;
    healthy_ = true;
    return pins;
  } catch (const Fault &error) { fail(error.what()); }
}

void Hcs08BoardSerial::attach_pad_source(const Hcs08GpioPort port, GpioPadReader reader) {
  healthy();
  board_.attach_pad_source(port, std::move(reader));
}

Hcs08BoardSerialCheckpoint Hcs08BoardSerial::checkpoint() const {
  healthy();
  if (board_.memory_.serial_period_owned_) { fail("serial period has not reached its boundary"); }
  return {board_.checkpoint(), spi_.checkpoint(context()), run_.checkpoint(), contract_};
}

Hcs08BoardResetCheckpoint
Hcs08BoardSerial::reset_checkpoint(const Hcs08BoardResetRequest &request) const {
  return board_.reset_checkpoint(request);
}

Hcs08BoardState Hcs08BoardSerial::board_state() const {
  return board_.state();
}

Hcs08State Hcs08BoardSerial::cpu_state() const {
  return board_.cpu_state();
}

Hcs08SpiRunState Hcs08BoardSerial::run_state() const {
  return run_.state();
}

Hcs08SpiClockState Hcs08BoardSerial::clock_state() const {
  return spi_.state();
}

const Hcs08BoardMemory &Hcs08BoardSerial::memory() const {
  return board_.memory();
}

bool Hcs08BoardSerial::faulted() const {
  return !healthy_ || board_.faulted() || spi_.faulted() || run_.faulted();
}
} // namespace blofeld
