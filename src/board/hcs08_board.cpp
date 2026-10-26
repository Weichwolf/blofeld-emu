// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board.hpp"

#include "board/gpio_pads.hpp"
#include "board/hcs08_board_memory.hpp"
#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_run_clock.hpp"

#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <utility>

namespace blofeld {
namespace {
Hcs08BoardMemoryContract memory_contract(const Hcs08BoardContract contract) {
  if (contract != Hcs08BoardContract::candidate_gt8a_reference_retirement) {
    throw Fault("HCS08 board configure address=0xfffe pc=unknown agent=CPU: explicit reference "
                "contract required");
  }
  return Hcs08BoardMemoryContract::candidate_gt8a_reference_bus;
}

Hcs08 &reset(Hcs08 &cpu) {
  cpu.reset(Hcs08ResetEntry::normal, Hcs08ResetCause::power_on);
  return cpu;
}

Hcs08 &restore(Hcs08 &cpu, const Hcs08Checkpoint &checkpoint) {
  cpu.restore_checkpoint(checkpoint);
  return cpu;
}
} // namespace

Hcs08BoardCheckpoint::Hcs08BoardCheckpoint(Hcs08BoardMemory memory,
                                           const Hcs08Checkpoint &cpu,
                                           const Hcs08BdcCheckpoint &bdc,
                                           const Hcs08RunCheckpoint &clock,
                                           const Hcs08BoardState state)
    : memory_(std::move(memory)), cpu_(cpu), bdc_(bdc), clock_(clock), state_(state) {}

Hcs08Board::Hcs08Board(const Hcs08FlashImage &image,
                       const std::filesystem::path &specification,
                       const Hcs08BoardContract contract,
                       const Hcs08BoardInputs &inputs)
    : memory_(image, memory_contract(contract)),
      cpu_(memory_, specification),
      bdc_(reset(cpu_), memory_, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::normal),
      clock_(memory_, bdc_, Hcs08RunContract::reference_instruction_retirement) {
  if (inputs.adc_reference) { memory_.observe_adc_reference(*inputs.adc_reference); }
  for (unsigned channel = 0; channel < inputs.analog.size(); ++channel) {
    if (inputs.analog[channel]) { memory_.observe_adc_input(channel, inputs.analog[channel]); }
  }
  if (inputs.timer_phase) { memory_.observe_timer_phase(*inputs.timer_phase); }
}

Hcs08Board::Hcs08Board(const Hcs08BoardCheckpoint &checkpoint,
                       const std::filesystem::path &specification)
    : memory_(checkpoint.memory_),
      cpu_(memory_, specification),
      bdc_(restore(cpu_, checkpoint.cpu_),
           memory_,
           Hcs08FlashContract::candidate_gt8a,
           Hcs08BdcReset::normal),
      clock_(memory_, bdc_, Hcs08RunContract::reference_instruction_retirement),
      state_(checkpoint.state_) {
  bdc_.restore_checkpoint(checkpoint.bdc_);
  clock_.restore_checkpoint(checkpoint.clock_);
}

void Hcs08Board::attach_pad_source(const Hcs08GpioPort port, GpioPadReader reader) {
  healthy("attach pads");
  memory_.attach_pad_source(port, std::move(reader));
}

void Hcs08Board::advance_reference_period() {
  healthy("reference period");
  healthy_ = false;
  if (state_.phase != Hcs08BoardPhase::boundary || bdc_.state().active) {
    fail("reference period", "background/debug or incomplete phase clocks are unqualified");
  }
  const auto cpu = cpu_.state();
  if (!cpu.pc_known) { fail("reference period", "unknown CPU PC before clock admission"); }
  if (state_.attempted_periods == std::numeric_limits<std::uint64_t>::max()) {
    fail("reference period", "board period horizon exhausted");
  }
  ++state_.attempted_periods;
  state_.phase = Hcs08BoardPhase::devices;
  try {
    memory_.advance_reference_bus({.pc = cpu.pc, .ccr = cpu.ccr});
    state_.phase = Hcs08BoardPhase::cpu;
    clock_.advance();
  } catch (const Fault &error) {
    fail("reference period", "source detail: " + std::string(error.what()));
  }
  ++state_.completed_periods;
  state_.phase = Hcs08BoardPhase::boundary;
  healthy_ = true;
}

void Hcs08Board::receive_bdc(const std::uint8_t byte) {
  healthy("BDC receive");
  healthy_ = false;
  bdc_.receive(byte);
  healthy_ = true;
}

void Hcs08Board::service_bdc(const Hcs08BdcGrant grant) {
  healthy("BDC service");
  healthy_ = false;
  if (state_.phase != Hcs08BoardPhase::boundary || clock_.state().remaining_periods != 0 ||
      grant != Hcs08BdcGrant::reference_instruction_boundary) {
    fail("BDC service", "requires an explicit reference grant at a complete instruction boundary");
  }
  state_.phase = Hcs08BoardPhase::bdc;
  bdc_.service(grant);
  state_.phase = Hcs08BoardPhase::boundary;
  healthy_ = true;
}

std::uint8_t Hcs08Board::read_bdc_reply() {
  healthy("BDC reply");
  healthy_ = false;
  const auto result = bdc_.read_reply();
  healthy_ = true;
  return result;
}

Hcs08BoardCheckpoint Hcs08Board::checkpoint() const {
  healthy("checkpoint");
  if (state_.phase != Hcs08BoardPhase::boundary) {
    fail("checkpoint", "no successful snapshot of a partially completed board period");
  }
  // Copying the const source into the value parameter detaches external pad callbacks.
  return {memory_, cpu_.checkpoint(), bdc_.checkpoint(), clock_.checkpoint(), state_};
}

void Hcs08Board::healthy(const std::string &operation) const {
  if (faulted()) {
    fail(operation, "incomplete board period or transaction requires coordinated recovery");
  }
}

bool Hcs08Board::faulted() const {
  return !healthy_ || memory_.faulted() || cpu_.faulted() || bdc_.faulted() || clock_.faulted();
}

Hcs08BoardState Hcs08Board::state() const {
  return state_;
}

Hcs08State Hcs08Board::cpu_state() const {
  return cpu_.state();
}

Hcs08RunState Hcs08Board::run_state() const {
  return clock_.state();
}

Hcs08BdcState Hcs08Board::bdc_state() const {
  return bdc_.state();
}

const Hcs08BoardMemory &Hcs08Board::memory() const {
  return memory_;
}

[[noreturn]] void Hcs08Board::fail(const std::string &operation, const std::string &reason) const {
  const auto cpu = cpu_.state();
  throw Fault("HCS08 board owner " + operation +
              " address=" + (cpu.pc_known ? hex(cpu.pc) : "unknown pc-known=0") +
              " pc=" + (cpu.pc_known ? hex(cpu.pc) : "unknown pc-known=0") +
              " ccr=" + hex(cpu.ccr) + " ccr-known=" + hex(cpu.ccr_known) +
              " attempted=" + std::to_string(state_.attempted_periods) +
              " completed=" + std::to_string(state_.completed_periods) + " agent=CPU: " + reason);
}
} // namespace blofeld
