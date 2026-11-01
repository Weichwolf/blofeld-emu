// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_interrupt_fixture.hpp"

#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace blofeld::test::irq {
namespace {
constexpr unsigned word_mask = 0xffffff, priorities_address = 0xfffffe;
constexpr unsigned low_io = 0xffff80, high_io = 0xffffc0;
constexpr unsigned movep_low_x = 0x07f400, movep_high_x = 0x08f480;
constexpr unsigned mask_shift = 8, level_mask = 3;
constexpr unsigned word_bits = 24, vector_bound = 2;

void check(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

dsp56300::InstructionWords foreground(OwnDspMachine &machine) {
  const auto context = machine.cpu().instruction_fetch();
  const auto opcode = context.latched_opcode ? *context.latched_opcode
                                             : machine.bus().inspect_program(context.address);
  const auto shape = dsp56300::inspect_instruction(opcode);
  const auto extension = shape && shape->words == 2
                             ? std::optional{machine.bus().inspect_program(context.address + 1)}
                             : std::nullopt;
  return {.address = context.address, .opcode = opcode, .extension = extension};
}
}

void load_program(DspBus &bus, const unsigned start, const std::span<const std::uint32_t> words) {
  if (start >= dma_test::boot) {
    bus.install_rom({.start = start, .words = words});
  } else {
    for (std::size_t index = 0; index < words.size(); ++index) {
      bus.write({.address = start + static_cast<unsigned>(index), .value = words[index]});
    }
  }
}

void Program::emit(const std::uint32_t opcode, const std::optional<std::uint32_t> extension) {
  check(opcode <= word_mask && (!extension || *extension <= word_mask),
        "Authored IRQ diagnostic words remain24-bit");
  const auto shape = dsp56300::inspect_instruction(opcode);
  check(shape && (shape->words == 2) == extension.has_value(),
        "Literal IRQ instruction has its independently specified extension count");
  words_.push_back(opcode);
  if (extension) { words_.push_back(*extension); }
  ++instructions_;
}

void Program::write(const DspStore store) {
  check(store.space == DspSpace::x && store.address >= low_io && store.address <= word_mask,
        "IRQ diagnostic MOVEP writer supports its documented X peripheral group");
  const auto group = store.address < high_io ? low_io : high_io;
  emit((store.address < high_io ? movep_low_x : movep_high_x) | (store.address - group),
       store.value);
}

void Program::timer(const unsigned csr, const TimerValues values) {
  write({.space = DspSpace::x, .address = csr - 1, .value = values.load});
  write({.space = DspSpace::x, .address = csr - 2, .value = values.compare});
  write({.space = DspSpace::x, .address = csr, .value = 7});
}

void Program::setup(const Setup inputs) {
  // Define the fixture canary with a real MOVE, independently of unknown reset GPRs.
  emit(short_r7_zero);
  emit(move_x1, initial_x1);
  emit(move_sr, inputs.status);
  write({.space = DspSpace::x, .address = priorities_address, .value = inputs.priorities});
}

void Program::install(DspBus &bus, const unsigned start) const {
  load_program(bus, start, words_);
}

unsigned Program::pc() const {
  return dma_test::boot + static_cast<unsigned>(words_.size());
}

Driver::Driver(OwnDspMachine &machine) : machine_(machine) {
  hardware_reset();
}

void Driver::hardware_reset() {
  constexpr std::uint64_t hold = 90;
  constexpr std::uint64_t recovery = 90;
  machine_.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  check(machine_.advance_hardware_reset_ticks(hold) == hold, "IRQ fixture actual reset hold");
  machine_.release_hardware_reset();
  check(machine_.advance_hardware_reset_ticks(recovery) == recovery,
        "IRQ fixture actual reset recovery");
}

void Driver::step() {
  check(machine_.cpu().fault().kind == dsp56300::FaultKind::none,
        "Faulted native IRQ execution requires hardware reset");
  const auto before = state();
  const auto request = machine_.bus().interrupt_request((before.sr >> mask_shift) & level_mask);
  const bool admitted = request && machine_.admit_interrupt(foreground(machine_));
  try {
    dma_test::step_native(machine_);
    if (!admitted) { return; }
    for (unsigned word = 1;
         word < vector_bound && machine_.cpu().instruction_fetch().exception_vector;
         ++word) {
      dma_test::step_native(machine_);
    }
    check(!machine_.interrupt_pending() && !machine_.cpu().instruction_fetch().exception_vector,
          "Native IRQ completes vector ownership within its two real words");
  } catch (const Fault &error) {
    const auto fault = machine_.cpu().fault();
    const auto context =
        admitted ? "irq=" + hex(request->vector, 2) + " level=" + std::to_string(request->level)
                 : "native";
    throw Fault(context + " pc=" + hex(before.pc, 6) +
                " fault.kind=" + std::to_string(static_cast<unsigned>(fault.kind)) +
                " space=" + std::to_string(static_cast<unsigned>(fault.space)) +
                " address=" + hex(fault.address, 6) + " value=" + hex(fault.value, 6) +
                " fault.pc=" + hex(fault.pc, 6) + " sr=" + hex(state().sr, 6) + ": " +
                error.what());
  }
}

unsigned stack_pc(const dsp56300::Registers &state) {
  return static_cast<unsigned>(state.stack.at(state.sp & (dsp56300::stack_entries - 1)) >>
                               word_bits);
}

unsigned stack_sr(const dsp56300::Registers &state) {
  return static_cast<unsigned>(state.stack.at(state.sp & (dsp56300::stack_entries - 1))) &
         word_mask;
}
}
