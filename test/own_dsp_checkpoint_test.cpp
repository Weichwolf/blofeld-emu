// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>

namespace {
using namespace blofeld;
using namespace dsp56300;
constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                   .timebase_hz = 540000000,
                                   .switch_timing = DspClockSwitch::reference_instruction_boundary};
constexpr Word boot_entry = 0xff0000;
constexpr Word movec_sr = 0x05f439;
constexpr Word immediate_x0 = 0x44f400;
constexpr Word clear_a = 0x200013;
constexpr Word repeat_three = 0x0603a0;
constexpr Word add_x0 = 0x200040;
constexpr Word store_short = 0x441000;
constexpr Word destination = 0x10;
constexpr Word cookie_address = 0x11;
constexpr Word unknown_address = 0x12;
constexpr Word cookie = 0x123456;
constexpr Word irqa_vector = 0x10;
constexpr Word priority_register = 0xffffff;
constexpr unsigned period_ticks = 45;
constexpr unsigned hold_ticks = 2 * period_ticks;
constexpr unsigned recovery_ticks = 3 * period_ticks;

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
  require(rejected, "Unsupported checkpoint operation was accepted");
}

void begin_reset(OwnDspMachine &machine) {
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3});
}

void boot(OwnDspMachine &machine, const std::span<const Word> program) {
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = program});
  begin_reset(machine);
  require(machine.advance_hardware_reset_ticks(hold_ticks) == hold_ticks, "Lost reset hold");
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset_ticks(recovery_ticks) == recovery_ticks,
          "Lost reset recovery");
}

PendingInstruction pending(const OwnDspMachine &machine) {
  const auto effect = machine.cpu().pending_instruction();
  if (!effect) { throw std::runtime_error("Expected a staged native effect"); }
  return *effect;
}

void retire(OwnDspMachine &machine) {
  for (unsigned effects = 0; effects < 32; ++effects) {
    const auto step = machine.advance(pending(machine).ticket);
    require(step.fault.kind == FaultKind::none, "Native effect failed");
    if (step.retired) { return; }
  }
  throw std::runtime_error("Native instruction did not retire");
}

void finish_program(OwnDspMachine &machine) {
  constexpr unsigned retirements = 8;
  for (unsigned instructions = 0; instructions < retirements; ++instructions) {
    if (machine.cpu().progress().instructions == retirements) { break; }
    if (!machine.cpu().pending_instruction()) { static_cast<void>(machine.begin_fetch()); }
    retire(machine);
  }
  constexpr Word end = boot_entry + 7;
  constexpr unsigned operand = 7;
  constexpr unsigned accumulated = 3 * operand;
  constexpr unsigned fractional_shift = 24;
  const auto regs = machine.cpu().registers();
  require(machine.cpu().progress().instructions == retirements && regs.pc == end &&
              regs.x == operand && regs.a == (std::uint64_t{accumulated} << fractional_shift) &&
              machine.bus().read({.space = DspSpace::x, .address = destination}) == operand,
          "Restored REP/ALU/store result differs from independent instruction expectations");
  require(machine.clock().elapsed_ticks() == hold_ticks + recovery_ticks,
          "Checkpoint manufactured untimed execution clocks");
}

void staged_cuts() {
  // Own encodings: immediate X0, CLR A, REP #3, ADD X0,A, MOVE X0,X:$10, NOP.
  const std::array<Word, 7> program{immediate_x0, 7, clear_a, repeat_three, add_x0, store_short, 0};
  OwnDspMachine source(clocks);
  boot(source, program);
  source.bus().write({.space = DspSpace::x, .address = cookie_address, .value = cookie});
  unsigned cuts = 0;
  while (source.cpu().progress().instructions < 8) {
    if (!source.cpu().pending_instruction()) { static_cast<void>(source.begin_fetch()); }
    const auto before = pending(source);
    const auto registers = source.cpu().registers();
    const auto saved = source.checkpoint();
    OwnDspMachine clone(clocks);
    clone.restore(saved);
    reject([&] { static_cast<void>(clone.advance(before.ticket)); });
    require(clone.cpu().registers() == registers && pending(clone).effect == before.effect,
            "Cross-owner restore changed published registers or pending effect");
    finish_program(clone);
    source.bus().write({.space = DspSpace::x, .address = cookie_address, .value = 0});
    source.bus().write({.space = DspSpace::x, .address = unknown_address, .value = 0});
    source.restore(saved);
    reject([&] { static_cast<void>(source.advance(before.ticket)); });
    require(source.bus().read({.space = DspSpace::x, .address = cookie_address}) == cookie,
            "Same-owner restore lost known RAM");
    reject([&] {
      static_cast<void>(source.bus().read({.space = DspSpace::x, .address = unknown_address}));
    });
    require(source.cpu().registers() == registers && pending(source).effect == before.effect,
            "Same-owner restore changed registers or pending effect");
    const auto step = source.advance(pending(source).ticket);
    require(step.fault.kind == FaultKind::none, "Original staged prefix failed");
    if (++cuts > 128) { throw std::runtime_error("Staged checkpoint matrix did not terminate"); }
  }
  finish_program(source);
  require(cuts > 16, "Staged test omitted native publication cuts");
  std::cout << "Staged checkpoint cuts: " << cuts << '\n';
}

void fractional_reset() {
  OwnDspMachine source(clocks);
  begin_reset(source);
  require(source.advance_hardware_reset_ticks(hold_ticks - 1) == hold_ticks - 1,
          "Reset assertion fraction was lost");
  const auto held = source.checkpoint();
  OwnDspMachine clone(clocks);
  clone.restore(held);
  reject([&] { clone.release_hardware_reset(); });
  reject([&] { static_cast<void>(clone.begin_fetch()); });
  require(clone.advance_hardware_reset_ticks(1) == 1, "Restored assertion endpoint changed");
  clone.release_hardware_reset();
  require(clone.advance_hardware_reset_ticks(1) == 1, "Recovery fraction was lost");
  const auto recovering = clone.checkpoint();
  source.restore(recovering);
  reject([&] { static_cast<void>(source.begin_fetch()); });
  require(source.advance_hardware_reset_ticks(recovery_ticks) == recovery_ticks - 1 &&
              source.clock().elapsed_ticks() == hold_ticks + recovery_ticks,
          "Restored fractional recovery did not stop at the literal endpoint");
  const OwnDspCheckpoint empty;
  reject([&] { source.restore(empty); });
  require(source.clock().elapsed_ticks() == hold_ticks + recovery_ticks,
          "Rejected empty handle changed the destination clock");
  auto other_clocks = clocks;
  other_clocks.timebase_hz *= 2;
  OwnDspMachine incompatible(other_clocks);
  reject([&] { incompatible.restore(recovering); });
  require(incompatible.clock().elapsed_ticks() == 0 && !incompatible.clock().hardware_reset_state(),
          "Rejected clock mismatch changed destination state");
}

void vector_cut(const bool fetched) {
  OwnDspMachine source(clocks);
  const std::array<Word, 3> program{movec_sr, 0, 0};
  boot(source, program);
  static_cast<void>(source.begin_fetch());
  retire(source);
  source.bus().write({.address = irqa_vector, .value = 0});
  source.bus().write({.address = irqa_vector + 1, .value = 0});
  source.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  source.bus().observe_irqa(false);
  require(source.admit_interrupt({.address = boot_entry + 2, .opcode = 0}), "IRQA not admitted");
  static_cast<void>(source.begin_fetch());
  require(pending(source).effect.address == irqa_vector, "Wrong first vector read");
  try {
    static_cast<void>(source.candidate_checkpoint(pending(source).ticket));
    throw std::runtime_error("Pending vector candidate was accepted");
  } catch (const blofeld::Fault &error) {
    require(std::string(error.what()).contains("interrupt vector ownership"),
            "Candidate did not reject unresolved vector ownership");
  }
  if (fetched) { static_cast<void>(source.advance(pending(source).ticket)); }
  const auto saved = source.checkpoint();
  OwnDspMachine clone(clocks);
  clone.restore(saved);
  source.restore(saved);
  for (auto *machine : {&source, &clone}) {
    require(machine->bus().interrupt_request(0).has_value(), "Checkpoint acknowledged first word");
    if (!fetched) { static_cast<void>(machine->advance(pending(*machine).ticket)); }
    retire(*machine);
    static_cast<void>(machine->begin_fetch());
    require(pending(*machine).effect.address == irqa_vector + 1, "Lost second vector ownership");
    static_cast<void>(machine->advance(pending(*machine).ticket));
    require(!machine->bus().interrupt_request(0), "Second real vector read was not acknowledged");
    retire(*machine);
    require(machine->cpu().registers().pc == boot_entry + 2, "Fast vector changed foreground PC");
  }
}

void pad_binding() {
  OwnDspMachine source(clocks);
  auto &port = source.bus().gpio(DspPort::c);
  port.write({.reg = DspGpioRegister::control, .value = 1});
  unsigned calls = 0;
  port.attach_pad_source([&calls](const std::uint32_t mask) {
    ++calls;
    return GpioPadSample{.known = mask, .levels = mask};
  });
  require(port.read(DspGpioRegister::data, 1) == 1, "Original pad source not sampled");
  const auto saved = source.checkpoint();
  OwnDspMachine clone(clocks);
  clone.restore(saved);
  source.restore(saved);
  for (auto *machine : {&source, &clone}) {
    auto &restored = machine->bus().gpio(DspPort::c);
    reject([&] { static_cast<void>(restored.read(DspGpioRegister::data, 1)); });
    restored.attach_pad_source(
        [](const std::uint32_t mask) { return GpioPadSample{.known = mask}; });
    require(restored.read(DspGpioRegister::data, 1) == 0, "Reconnected pad source was ignored");
  }
  require(calls == 1, "Checkpoint called or retained an external board reader");
}

void failure_latch() {
  OwnDspMachine source(clocks);
  const std::array<Word, 2> program{0, 0};
  boot(source, program);
  const auto healthy = source.checkpoint();
  // Restore removes the later ROM installation, including its knowledge bits.
  source.bus().install_rom({.space = DspSpace::p, .start = boot_entry + 2, .words = program});
  static_cast<void>(source.begin_fetch());
  retire(source);
  static_cast<void>(source.begin_fetch());
  retire(source);
  source.restore(healthy);
  static_cast<void>(source.begin_fetch());
  retire(source);
  static_cast<void>(source.begin_fetch());
  retire(source);
  static_cast<void>(source.begin_fetch());
  const auto failed = source.advance(pending(source).ticket);
  require(failed.fault.kind == FaultKind::memory_read && source.io().failure(),
          "Absent restored ROM word did not fail visibly");
  const auto faulted = source.checkpoint();
  OwnDspMachine clone(clocks);
  clone.restore(faulted);
  const auto failure = clone.io().failure();
  require(failure && failure->operation == OwnDspIo::Operation::read &&
              failure->space == Space::program && failure->address == boot_entry + 2 &&
              failure->pc == boot_entry + 2 && failure->cause &&
              clone.cpu().fault() == source.cpu().fault(),
          "Restored I/O failure lost context or became healthy");
  source.restore(healthy);
  require(!source.io().failure() && source.cpu().fault().kind == FaultKind::none,
          "Healthy checkpoint did not recover CPU and bus adapter together");
  static_cast<void>(source.begin_fetch());
  retire(source);
}

void callback_guard() {
  constexpr Word load_x0_absolute = 0x44f000;
  constexpr Word port_c_data = 0xffffbd;
  constexpr Word pins = 0xfff;
  constexpr Word incoming = 0x5a5;
  OwnDspMachine source(clocks);
  const std::array program{load_x0_absolute, port_c_data};
  boot(source, program);
  const auto idle = source.checkpoint();
  auto &port = source.bus().gpio(DspPort::c);
  port.write({.reg = DspGpioRegister::control, .value = pins});
  unsigned calls = 0;
  port.attach_pad_source([&](const std::uint32_t mask) {
    ++calls;
    reject([&] { static_cast<void>(source.checkpoint()); });
    reject([&] { source.restore(idle); });
    return GpioPadSample{.known = mask, .levels = incoming & mask};
  });
  static_cast<void>(source.begin_fetch());
  retire(source);
  require(calls == 1 && source.cpu().registers().x == incoming && !source.io().failure(),
          "Rejected callback reentry changed the actual native read");
}

void candidate_continuation() {
  const std::array<Word, 4> program{immediate_x0, 7, clear_a, 0};
  OwnDspMachine source(clocks);
  boot(source, program);
  source.bus().write({.space = DspSpace::x, .address = cookie_address, .value = cookie});
  static_cast<void>(source.begin_fetch());
  const auto fetch = pending(source);
  reject([&] { static_cast<void>(source.candidate_checkpoint(fetch.ticket)); });
  for (unsigned effects = 0; effects < 32; ++effects) {
    const auto current = pending(source);
    if (current.effect.kind == EffectKind::retire) { break; }
    const auto result = source.advance(current.ticket);
    require(result.fault.kind == FaultKind::none && !result.retired,
            "Candidate preparation must preserve native ownership");
  }
  const auto retirement = pending(source);
  require(retirement.effect.kind == EffectKind::retire, "Candidate requires actual retirement");
  const auto visible = source.cpu().registers();
  const auto ticks = source.clock().elapsed_ticks();
  const auto saved = source.candidate_checkpoint(retirement.ticket);
  OwnDspMachine successor(clocks);
  successor.restore(saved);
  require(source.cpu().registers() == visible && source.cpu().progress().instructions == 0 &&
              source.clock().elapsed_ticks() == ticks &&
              pending(source).effect == retirement.effect,
          "Capturing a candidate must not publish or clock the original owner");
  require(successor.cpu().registers().x == 7 && successor.cpu().registers().pc == boot_entry + 2 &&
              successor.cpu().progress().instructions == 1 &&
              successor.clock().elapsed_ticks() == ticks &&
              successor.bus().read({.space = DspSpace::x, .address = cookie_address}) == cookie,
          "Candidate must contain real CPU, bus and clock state");
  static_cast<void>(successor.begin_fetch());
  reject([&] { static_cast<void>(successor.candidate_checkpoint(retirement.ticket)); });
  retire(successor);
  require(successor.cpu().registers().pc == boot_entry + 3 &&
              successor.cpu().progress().instructions == 2,
          "Candidate executes the real successor instruction");
  require(source.advance(retirement.ticket).retired && source.cpu().registers().x == 7,
          "The original owner retains its actual retirement ticket");
  reject([&] { static_cast<void>(source.candidate_checkpoint(retirement.ticket)); });
}

void candidate_omr_guard() {
  constexpr Word movec_omr = 0x05f43a;
  constexpr Word changed_omr = 0x304;
  const std::array<Word, 2> program{movec_omr, changed_omr};
  OwnDspMachine source(clocks);
  boot(source, program);
  static_cast<void>(source.begin_fetch(InstructionPublication::retirement));
  for (unsigned effects = 0; effects < 32; ++effects) {
    const auto effect = pending(source);
    if (effect.effect.kind == EffectKind::retire) { break; }
    const auto step = source.advance(effect.ticket);
    require(step.fault.kind == FaultKind::none && !step.retired, "OMR instruction failed");
  }
  const auto retirement = pending(source);
  require(retirement.effect.kind == EffectKind::retire && retirement.computed.omr == changed_omr &&
              source.cpu().registers().omr != changed_omr,
          "OMR guard requires a computed but unpublished transition");
  try {
    static_cast<void>(source.candidate_checkpoint(retirement.ticket));
    throw std::runtime_error("Unpublished OMR candidate was accepted");
  } catch (const blofeld::Fault &error) {
    require(std::string(error.what()).contains("published OMR boundary"),
            "Candidate did not reject an unpublished OMR transition");
  }
  require(source.advance(retirement.ticket).retired && source.cpu().registers().omr == changed_omr,
          "Rejected candidate must leave the actual OMR retirement intact");
}
}

int main() {
  try {
    staged_cuts();
    fractional_reset();
    vector_cut(false);
    vector_cut(true);
    pad_binding();
    failure_latch();
    callback_guard();
    candidate_continuation();
    candidate_omr_guard();
    std::cout << "Own DSP coordinated checkpoint checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
