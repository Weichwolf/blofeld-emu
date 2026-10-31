// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using namespace blofeld;
using namespace dsp56300;
constexpr Word boot_entry = 0xff0000;
constexpr Word cookie = 0x123456;
constexpr Word retained_ep = 0x1234;
constexpr Word retained_sz = 0x40;
constexpr std::uint64_t hold_ticks = 90;
constexpr std::uint64_t recovery_ticks = 135;
constexpr std::uint64_t reset_ticks = hold_ticks + recovery_ticks;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned timebase_hz = 540000000;
constexpr Word movec_ep = 0x05f42a;
constexpr Word movec_sr = 0x05f439;
constexpr Word irqa_vector = 0x10;
constexpr Word priority_register = 0xffffff;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

template <class Operation> void reject(Operation operation) {
  bool rejected = false;
  try {
    operation();
  } catch (const std::logic_error &) { rejected = true; } catch (const blofeld::Fault &) {
    rejected = true;
  }
  require(rejected, "Unsupported reset or instruction operation was accepted");
}

OwnDspResetInputs pins() {
  return {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3};
}

void check_registers(const OwnDspMachine &machine) {
  constexpr Word reset_status = 0xc00300;
  constexpr Word reset_omr = 0x305;
  const auto regs = machine.cpu().registers();
  require(regs.pc == boot_entry && regs.sr == reset_status && regs.omr == reset_omr &&
              regs.sp == 0 && regs.sc == 0 && regs.vba == 0,
          "Reset register values differ from independent manual expectations");
  for (const auto modifier : regs.m) {
    require(modifier == word_mask, "Reset did not restore linear address arithmetic");
  }
  require(machine.cpu().progress() == Progress{}, "Reset executed a firmware instruction");
}

void finish_reset(OwnDspMachine &machine) {
  reject([&] { static_cast<void>(machine.begin_fetch()); });
  require(machine.advance_hardware_reset_ticks(hold_ticks - 1) == hold_ticks - 1,
          "Partial assertion duration was changed");
  reject([&] { machine.release_hardware_reset(); });
  require(machine.advance_hardware_reset_ticks(1) == 1, "Assertion boundary was lost");
  machine.release_hardware_reset();
  reject([&] { static_cast<void>(machine.begin_fetch()); });
  require(machine.advance_hardware_reset_ticks(1) == 1, "Recovery fraction was lost");
  constexpr unsigned excess = 1000;
  require(machine.advance_hardware_reset_ticks(excess) == recovery_ticks - 1,
          "Recovery did not stop at its exact endpoint");
  check_registers(machine);
}

void retire(OwnDspMachine &machine, InstructionProgress step, const Word expected_pc) {
  for (unsigned effects = 0; effects < 16 && step.pending && !step.retired; ++effects) {
    step = machine.advance(step.pending->ticket);
  }
  if (!step.retired || step.fault.kind != FaultKind::none ||
      machine.cpu().registers().pc != expected_pc) {
    throw std::runtime_error("Own DSP retirement expected-pc=" + blofeld::hex(expected_pc, 6) +
                             " actual-pc=" + blofeld::hex(machine.cpu().registers().pc, 6) +
                             " fault=" + blofeld::hex(static_cast<unsigned>(step.fault.kind), 2) +
                             " retired=" + (step.retired ? "yes" : "no"));
  }
}

void run() {
  OwnDspMachine machine({.extal_hz = extal_hz,
                         .timebase_hz = timebase_hz,
                         .switch_timing = DspClockSwitch::reference_instruction_boundary});
  // FM table A-2 register fields and MOVEC immediate extension format.
  constexpr Word movec_sz = 0x05f438;
  const std::array<Word, 6> own_program{0, movec_ep, retained_ep, movec_sz, retained_sz, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = own_program});
  machine.bus().write({.space = DspSpace::x, .address = 0x10, .value = cookie});
  reject([&] { static_cast<void>(machine.begin_fetch()); });
  auto unknown = pins();
  unknown.pinit.reset();
  reject([&] { machine.begin_hardware_reset(unknown); });
  require(machine.clock().elapsed_ticks() == 0, "Rejected pins changed reset time");
  machine.begin_hardware_reset(pins());
  check_registers(machine);
  finish_reset(machine);
  auto step = machine.begin_fetch();
  if (!step.pending) { throw std::runtime_error("Boot fetch did not expose its native ticket"); }
  const auto stale = step.pending->ticket;
  retire(machine, step, boot_entry + 1);
  require(machine.clock().elapsed_ticks() == reset_ticks,
          "Untimed execution manufactured elapsed instruction time");
  retire(machine, machine.begin_fetch(), boot_entry + 3);
  retire(machine, machine.begin_fetch(), boot_entry + 5);
  require(machine.cpu().registers().ep == retained_ep &&
              machine.cpu().registers().sz == retained_sz,
          "Own MOVEC instructions did not initialize stack extension registers");
  machine.begin_hardware_reset(pins());
  finish_reset(machine);
  require(machine.cpu().registers().ep == retained_ep &&
              machine.cpu().registers().sz == retained_sz,
          "Warm reset initialized registers documented as unaffected by hardware reset");
  require(machine.clock().elapsed_ticks() == 2 * reset_ticks &&
              machine.bus().read({.space = DspSpace::x, .address = 0x10}) == cookie,
          "Warm reset lost elapsed time or retained RAM");
  constexpr Word reset_pll = 0x04401d;
  require(machine.bus().pll().read() == reset_pll, "PINIT-low did not reset the actual PLL");
  step = machine.begin_fetch();
  reject([&] { static_cast<void>(machine.advance(stale)); });
  retire(machine, step, boot_entry + 1);
}

void reset_during_register_write(const bool published) {
  OwnDspMachine machine({.extal_hz = extal_hz,
                         .timebase_hz = timebase_hz,
                         .switch_timing = DspClockSwitch::reference_instruction_boundary});
  const std::array<Word, 3> own_program{movec_ep, retained_ep, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = own_program});
  machine.begin_hardware_reset(pins());
  finish_reset(machine);
  const auto initial_ep = machine.cpu().registers().ep;
  auto step = machine.begin_fetch();
  for (unsigned effects = 0; effects < 16 && step.pending && !step.pending->effect.move_update;
       ++effects) {
    step = machine.advance(step.pending->ticket);
  }
  if (!step.pending || !step.pending->effect.move_update) {
    throw std::runtime_error("Own MOVEC did not expose its real register publication");
  }
  const auto move = step.pending->effect.move_update.value();
  require(move.count == 1 && move.updates[0].storage == RegisterStorage::ep &&
              move.updates[0].value == retained_ep && machine.cpu().registers().ep == initial_ep,
          "MOVEC register destination/result differs from the independent immediate encoding");
  if (published) {
    step = machine.advance(step.pending->ticket);
    require(machine.cpu().registers().ep == retained_ep, "Real EP publication was lost");
  }
  if (!step.pending) { throw std::runtime_error("MOVEC retired before the selected reset cut"); }
  const auto stale = step.pending->ticket;
  machine.begin_hardware_reset(pins());
  require(!machine.cpu().pending_instruction(), "Reset retained an in-flight instruction");
  finish_reset(machine);
  require(machine.cpu().registers().ep == (published ? retained_ep : initial_ep),
          "Reset retained a speculative value or rolled back a published register");
  step = machine.begin_fetch();
  reject([&] { static_cast<void>(machine.advance(stale)); });
  retire(machine, step, boot_entry + 2);
  require(machine.cpu().registers().ep == retained_ep &&
              machine.clock().elapsed_ticks() == 2 * reset_ticks,
          "Reset lost the boot program or added untimed instruction clocks");
}

void reset_during_interrupt_entry(const bool fetched) {
  OwnDspMachine machine({.extal_hz = extal_hz,
                         .timebase_hz = timebase_hz,
                         .switch_timing = DspClockSwitch::reference_instruction_boundary});
  const std::array<Word, 3> own_program{movec_sr, 0, 0};
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = own_program});
  machine.bus().write({.address = irqa_vector, .value = 0});
  machine.bus().write({.address = irqa_vector + 1, .value = 0});
  machine.begin_hardware_reset(pins());
  finish_reset(machine);
  retire(machine, machine.begin_fetch(), boot_entry + 2);
  machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  machine.bus().observe_irqa(false);
  const InstructionWords foreground{.address = boot_entry + 2, .opcode = 0};
  require(machine.admit_interrupt(foreground), "Real IRQA request was not admitted");
  auto step = machine.begin_fetch();
  if (!step.pending || step.pending->effect.kind != EffectKind::fetch_opcode ||
      step.pending->effect.address != irqa_vector) {
    throw std::runtime_error("Interrupt did not expose its actual first vector fetch");
  }
  if (fetched) { step = machine.advance(step.pending->ticket); }
  if (!step.pending) { throw std::runtime_error("Interrupt entry lost its selected reset cut"); }
  require(machine.bus().interrupt_request(0).has_value(), "First vector fetch acknowledged IRQA");
  const auto stale = step.pending->ticket;
  machine.begin_hardware_reset(pins());
  finish_reset(machine);
  require(!machine.bus().interrupt_request(0), "Reset retained an old board interrupt request");
  step = machine.begin_fetch();
  reject([&] { static_cast<void>(machine.advance(stale)); });
  retire(machine, step, boot_entry + 2);
  machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  machine.bus().observe_irqa(false);
  require(machine.admit_interrupt(foreground), "Old vector ownership blocked a fresh real IRQA");
  // FM 2.3.2.7/8: fast vectors divert fetches while the foreground PC stays unchanged.
  step = machine.begin_fetch();
  if (!step.pending || step.pending->effect.address != irqa_vector) {
    throw std::runtime_error("Fresh interrupt did not fetch its first real vector word");
  }
  retire(machine, step, foreground.address);
  step = machine.begin_fetch();
  if (!step.pending || step.pending->effect.address != irqa_vector + 1) {
    throw std::runtime_error("Fresh interrupt did not fetch its second real vector word");
  }
  retire(machine, step, foreground.address);
  require(!machine.bus().interrupt_request(0), "Fresh interrupt was not genuinely acknowledged");
}
}

int main() {
  try {
    run();
    reset_during_register_write(false);
    reset_during_register_write(true);
    reset_during_interrupt_entry(false);
    reset_during_interrupt_entry(true);
    std::cout << "Own DSP hardware reset and gated execution checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
