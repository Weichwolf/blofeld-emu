// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/own_dsp_fetch.hpp"

#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

namespace blofeld {
namespace {
constexpr unsigned cache_enable = 1U << 19U;
}

OwnDspFetch::OwnDspFetch(OwnDspMachine &machine)
    : machine_(machine), identity_(std::make_shared<const bool>(false)) {
  machine_.require_running();
  if (!machine_.captured_fetch_owner_.expired() || machine_.interrupts_.pending()) {
    throw Fault("Own DSP fetch cannot acquire CPU pc=" + hex(machine_.cpu_.registers().pc, 6));
  }
  check_eligible();
  const auto context = machine_.cpu_.instruction_fetch();
  fetch_.start(context.address, machine_.cpu_.registers().omr);
  machine_.captured_fetch_owner_ = identity_;
}

void OwnDspFetch::check_owner() const {
  if (machine_.captured_fetch_owner_.lock() != identity_) {
    throw Fault("Stale own DSP captured fetch pc=" + hex(machine_.cpu_.registers().pc, 6));
  }
  machine_.require_running();
}

void OwnDspFetch::check_eligible() const {
  const auto regs = machine_.cpu_.registers();
  if ((regs.sr & cache_enable) != 0) {
    throw Fault("Own DSP F2 cache ownership is unqualified read P:" + hex(regs.pc, 6) +
                " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
  const auto fetch = machine_.cpu_.instruction_fetch();
  if (fetch.latched_opcode) {
    throw Fault("Own DSP REP opcode latch is unqualified read P:" + hex(fetch.address, 6) +
                " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
}

void OwnDspFetch::check_sample_context() const {
  const auto regs = machine_.cpu_.registers();
  const auto state = fetch_.view();
  if ((regs.sr & cache_enable) != 0 || state.omr != regs.omr) {
    throw Fault("Unqualified own DSP F2 memory context read P:" +
                hex(state.pending_address.value_or(state.next_address), 6) +
                " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6));
  }
}

DspProgramFetchView OwnDspFetch::view() const {
  check_owner();
  return fetch_.view();
}

void OwnDspFetch::issue() {
  check_owner();
  check_sample_context();
  fetch_.issue();
}

void OwnDspFetch::capture() {
  check_owner();
  check_sample_context();
  fetch_.capture(machine_.bus_);
}

dsp56300::InstructionProgress OwnDspFetch::begin() {
  check_owner();
  if (in_flight_) { throw Fault("Own DSP F2 instruction awaits retirement"); }
  check_eligible();
  if (machine_.cpu_.progress().instructions == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("Own DSP F2 instruction counter exhausted pc=" +
                hex(machine_.cpu_.registers().pc, 6));
  }
  const auto context = machine_.cpu_.instruction_fetch();
  const auto state = fetch_.view();
  if (state.consume_address != context.address || state.omr != machine_.cpu_.registers().omr) {
    throw Fault("Own DSP F2 context differs from CPU read P:" + hex(context.address, 6) +
                " pc=" + hex(context.address, 6) + " omr=" + hex(state.omr, 6));
  }
  const auto opcode = fetch_.select(context.address);
  const auto shape = dsp56300::inspect_instruction(opcode.word.value);
  if (!shape) {
    throw Fault("Unsupported own DSP opcode " + hex(opcode.word.value, 6) +
                " read P:" + hex(context.address, 6) + " pc=" + hex(context.address, 6) +
                " omr=" + hex(state.omr, 6));
  }
  const auto captured = fetch_.instruction(opcode, shape->words);
  captured.check_front(fetch_);
  auto next = fetch_;
  static_cast<void>(next.consume(captured.opcode.word.address));
  if (captured.extension) { static_cast<void>(next.consume(captured.extension->word.address)); }
  captured.check_consumed(next.consumed_pair(), state.omr);
  const dsp56300::InstructionWords words{
      .address = captured.opcode.word.address,
      .opcode = captured.opcode.word.value,
      .extension =
          captured.extension ? std::optional(captured.extension->word.value) : std::nullopt};
  auto result =
      machine_.cpu_.begin_instruction(words, dsp56300::InstructionPublication::pcu_updates);
  if (result.fault.kind == dsp56300::FaultKind::none) {
    fetch_ = std::move(next);
    instructions_ = machine_.cpu_.progress().instructions;
    in_flight_ = true;
  }
  return result;
}

void OwnDspFetch::synchronize() {
  check_owner();
  if (!in_flight_ || machine_.cpu_.pending_instruction() ||
      machine_.cpu_.fault().kind != dsp56300::FaultKind::none ||
      machine_.cpu_.progress().instructions != instructions_ + 1) {
    throw Fault("Own DSP F2 synchronization requires its actual retirement pc=" +
                hex(machine_.cpu_.registers().pc, 6));
  }
  check_eligible();
  const auto context = machine_.cpu_.instruction_fetch();
  const auto regs = machine_.cpu_.registers();
  const auto state = fetch_.view();
  if (state.consume_address != context.address || state.omr != regs.omr) {
    fetch_.redirect(context.address, regs.omr);
  }
  in_flight_ = false;
}
}
