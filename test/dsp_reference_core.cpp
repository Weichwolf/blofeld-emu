// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_pll.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_timers.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <variant>

namespace blofeld::test::reference {
namespace {
using native_access::fault;

void unknown_read_and_recovery() {
  constexpr unsigned address = 0x100;
  DspMachine machine;
  DspProgram code;
  code.emit("move x:>$100,x0");
  code.install(machine.bus());
  auto progress = machine.begin_reference_instruction(boundaries);
  progress = machine.advance_reference_access(ticket(progress));
  progress = machine.advance_reference_access(ticket(progress));
  expect(progress,
         {.kind = DspNativeAccessKind::read,
          .target = {.space = DspSpace::x, .address = address},
          .ordinal = 2});
  const auto saved = machine.checkpoint();
  const auto denied = ticket(progress);
  const auto error = fault([&] { static_cast<void>(machine.advance_reference_access(denied)); });
  require(error.contains("Unknown DSP memory read at X:0x000100") &&
              error.contains("pc=0xff0000") && error.contains("ordinal=2"),
          "Unknown data fails at actual admission with instruction and access context");
  static_cast<void>(fault([&] { static_cast<void>(machine.reference_pending()); }));
  static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
  static_cast<void>(fault([&] { machine.cpu().step(); }));
  machine.restore(saved);
  machine.bus().write({.space = DspSpace::x, .address = address, .value = first});
  static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(denied)); }));
  progress = machine.advance_reference_access(ticket(machine));
  require(progress.retired && completed(progress).value == first &&
              machine.cpu().state().x == first && machine.cpu().state().pc == boot + 2,
          "Trusted restore recovers the suspended read and executes its real later-supplied data");
}

void protected_destination() {
  constexpr unsigned rom = 0x4000;
  constexpr unsigned writable = 0x100;
  DspMachine machine;
  DspProgram code;
  code.immediate("x0", first);
  const auto body = code.pc();
  code.emit("move x0,x:>$4000");
  code.install(machine.bus());
  const std::array rom_words{second};
  machine.bus().install_rom({.space = DspSpace::x, .start = rom, .words = rom_words});
  machine.cpu().step();
  auto progress = machine.begin_reference_instruction(boundaries);
  progress = machine.advance_reference_access(ticket(progress));
  const auto before_extension = machine.checkpoint();
  progress = machine.advance_reference_access(ticket(progress));
  expect(progress,
         {.kind = DspNativeAccessKind::write,
          .target = {.space = DspSpace::x, .address = rom},
          .value = first,
          .ordinal = 2,
          .pc = body});
  require(fault([&] {
            static_cast<void>(machine.advance_reference_access(ticket(progress)));
          }).contains("Protected DSP ROM write"),
          "Reference admission never bypasses real ROM write protection");
  require(machine.bus().read({.space = DspSpace::x, .address = rom}) == second,
          "Rejected destination admission preserves the actual ROM word");
  machine.restore(before_extension);
  const std::array repaired{writable};
  machine.bus().install_rom({.start = body + 1, .words = repaired});
  progress = machine.advance_reference_access(ticket(machine));
  progress = machine.advance_reference_access(ticket(progress));
  require(progress.retired &&
              machine.bus().read({.space = DspSpace::x, .address = writable}) == first &&
              machine.bus().read({.space = DspSpace::x, .address = rom}) == second,
          "Repair before actual extension consumption redirects only the resumed real store");
}

void parallel_alu_and_modulo() {
  constexpr unsigned address = 7;
  constexpr std::uint64_t accumulator_unit = std::uint64_t{1} << word_bits;
  constexpr unsigned ccr_mask = 0xff;
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", address);
  code.immediate("m0", address);
  code.immediate("x1", 2);
  code.immediate("a1", 1);
  code.immediate("sr", masked_status | carry | overflow | zero | negative);
  const auto body = code.pc();
  code.emit("add x1,a x:(r0)+,x0");
  code.emit("move x:(r0)+,x0");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = address, .value = first});
  machine.bus().write({.space = DspSpace::x, .address = 0, .value = third});
  for (unsigned index = 0; index < 5; ++index) { machine.cpu().step(); }
  const auto before = machine.cpu().debug_state();
  auto progress = machine.begin_reference_instruction(boundaries);
  progress = machine.advance_reference_access(ticket(progress));
  expect(progress,
         {.kind = DspNativeAccessKind::read,
          .target = {.space = DspSpace::x, .address = address},
          .ordinal = 1,
          .pc = body});
  require(machine.cpu().debug_state() == before,
          "A suspended parallel load restores the already computed ALU/CCR and AGU state");
  const auto saved = machine.checkpoint();
  for (const auto value : {first, second}) {
    machine.restore(saved);
    machine.bus().write({.space = DspSpace::x, .address = address, .value = value});
    progress = machine.advance_reference_access(ticket(machine));
    const auto after = machine.cpu().debug_state();
    require(progress.retired && after.cpu.a == 3U * accumulator_unit &&
                after.cpu.x == (std::uint64_t{2} << word_bits) + value &&
                (after.cpu.sr & ccr_mask) == unnormalized && after.cpu.r[0] == 0 &&
                after.cpu.m[0] == address &&
                after.native_instructions == before.native_instructions + 1,
            "Independent ADD value/flags and modulo-eight update retire exactly once");
    execute(machine);
    require(machine.cpu().state().r[0] == 1 &&
                machine.cpu().state().x == (std::uint64_t{2} << word_bits) + third,
            "Restored derived AGU state also selects the next real modulo data word");
  }
}

void unqualified_cache_controls() {
  // FM13-156..159: absolute EA extension (MMM=110/RRR=000), and PC-relative forms.
  constexpr unsigned plock_absolute = 0x0bf081;
  constexpr unsigned punlock_absolute = 0x0af081;
  for (const auto opcode : {3U, 1U, 2U, plock_absolute, 15U, punlock_absolute, 14U}) {
    DspMachine machine;
    DspProgram code;
    code.word(opcode);
    code.install(machine.bus());
    auto progress = machine.begin_reference_instruction(boundaries);
    const auto saved = machine.checkpoint();
    const auto before = machine.cpu().debug_state();
    const auto error =
        fault([&] { static_cast<void>(machine.advance_reference_access(ticket(progress))); });
    require(error.contains("Unqualified instruction-cache control") &&
                error.contains("pc=0xff0000") &&
                machine.cpu().debug_state().native_instructions == before.native_instructions,
            "Uncheckpointed physical instruction-cache controls reject before retirement");
    static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
    machine.restore(saved);
    const std::array nop{0U};
    machine.bus().install_rom({.start = boot, .words = nop});
    progress = machine.advance_reference_access(ticket(machine));
    require(progress.retired && machine.cpu().state().pc == boot + 1,
            "A trusted prefetch snapshot recovers after replacing the unqualified own opcode");
  }
}

void bit_admission() {
  constexpr unsigned direction = 0xffffd6;
  constexpr unsigned data = 0xffffd5;
  DspMachine machine;
  DspProgram code;
  code.emit("bset #1,x:<<$ffffd5");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = direction, .value = 3});
  auto progress = machine.begin_reference_instruction(boundaries);
  progress = machine.advance_reference_access(ticket(progress));
  expect(progress,
         {.kind = DspNativeAccessKind::bit,
          .target = {.space = DspSpace::x, .address = data},
          .ordinal = 1});
  require(machine.bus().read({.space = DspSpace::x, .address = data}) == 0,
          "Preparing a specialized bit operation does not modify the output latch");
  const auto saved = machine.checkpoint();
  for (const auto previous : {0U, 1U}) {
    machine.restore(saved);
    machine.bus().write({.space = DspSpace::x, .address = data, .value = 1U | (previous << 1U)});
    progress = machine.advance_reference_access(ticket(machine));
    require(progress.retired && completed(progress).value == previous &&
                (machine.cpu().state().sr & carry) == previous &&
                machine.bus().read({.space = DspSpace::x, .address = data}) == 3,
            "One real admitted RMW captures the then-current prior bit and preserves its neighbor");
  }
}

void software_reset() {
  constexpr unsigned reset_opcode = 0x84; // FM RESET instruction format.
  constexpr unsigned shi_control = 0xffff91;
  constexpr unsigned shi_configuration = 0xa9;
  DspMachine machine;
  DspProgram code;
  code.immediate("x0", first);
  const auto body = code.pc();
  code.word(reset_opcode);
  code.emit("nop");
  code.install(machine.bus());
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x, .address = shi_control, .value = shi_configuration});
  send(machine.bus().shi(), first);
  send(machine.bus().shi(), second);
  machine.cpu().step();
  auto progress = machine.begin_reference_instruction(boundaries);
  progress = machine.advance_reference_access(ticket(progress));
  require(progress.pending &&
              std::holds_alternative<DspPeripheralReset>(progress.pending->effect) &&
              machine.bus().read({.space = DspSpace::x, .address = fifo}) == first,
          "A pending software reset leaves the actual FIFO intact until it is admitted");
  const auto held = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(held);
    progress = machine.advance_reference_access(ticket(machine));
    require(progress.retired && progress.completed &&
                std::holds_alternative<DspPeripheralReset>(*progress.completed) &&
                machine.cpu().state().x == first && machine.cpu().state().pc == body + 1,
            "RESET clears peripherals at its explicit effect while retaining processor state");
    require(fault([&] {
              static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = fifo}));
            }).contains("empty DSP SHI FIFO"),
            "The admitted reset clears the remaining real FIFO word");
  }
}

void loaded_repeat() {
  constexpr unsigned address = 0x10;
  constexpr unsigned extended_opcode = 0x44f400;
  DspMachine machine;
  DspProgram code;
  code.emit("rep #3");
  const auto body = code.pc();
  code.emit("move x:<$10,x0");
  code.emit("nop");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = address, .value = first});
  machine.cpu().step();
  execute(machine);
  require(machine.cpu().state().lc == 2 && machine.cpu().state().x == first,
          "The first legal REP body fetches and executes once");
  const std::array changed{extended_opcode};
  machine.bus().install_rom({.start = body, .words = changed});
  auto progress = machine.begin_reference_instruction(boundaries);
  expect(progress,
         {.kind = DspNativeAccessKind::read,
          .target = {.space = DspSpace::x, .address = address},
          .pc = body});
  const auto saved = machine.checkpoint();
  for (const auto value : {second, third}) {
    machine.restore(saved);
    machine.bus().write({.space = DspSpace::x, .address = address, .value = value});
    progress = machine.advance_reference_access(ticket(machine));
    require(progress.retired && machine.cpu().state().lc == 1 && machine.cpu().state().x == value,
            "A loaded valid REP reads actual data without fetching changed backing opcode");
    execute(machine);
    require(machine.cpu().state().lc == 0 && machine.cpu().state().pc == body + 1 &&
                machine.cpu().debug_state().native_instructions == 4,
            "The independently expected three REP bodies retire through suspension and replay");
  }
  DspMachine pure;
  DspProgram nops;
  nops.emit("rep #2");
  nops.emit("nop");
  nops.install(pure.bus());
  pure.cpu().step();
  execute(pure);
  progress = pure.begin_reference_instruction(boundaries);
  require(progress.retired && !progress.pending && !progress.completed &&
              pure.cpu().debug_state().native_instructions == 3,
          "A loaded pure-register body retires without inventing a bus access");
}

void profile_guards() {
  DspMachine plain;
  DspProgram code;
  code.emit("nop");
  code.install(plain.bus());
  static_cast<void>(fault([&] {
    static_cast<void>(plain.begin_reference_instruction(DspReferenceExecution::unqualified));
  }));
  require(!plain.reference_pending(), "An unqualified contract cannot start speculative execution");
  constexpr unsigned extal = 12000000;
  constexpr unsigned timebase = 540000000;
  DspMachine clocked(
      DspRunContract{.clock = {.extal_hz = extal,
                               .timebase_hz = timebase,
                               .switch_timing = DspClockSwitch::reference_instruction_boundary},
                     .timing = DspRunTiming::native_internal_estimates});
  code.install(clocked.bus());
  clocked.bus().pll().observe_pinit(false);
  clocked.cpu().reset_spi_boot();
  const auto before = clocked.cpu().debug_state();
  static_cast<void>(
      fault([&] { static_cast<void>(clocked.begin_reference_instruction(boundaries)); }));
  require(clocked.cpu().debug_state() == before && clocked.run_clock().elapsed_ticks() == 0,
          "Access suspension cannot silently change the existing clocked completion contract");
  clocked.cpu().step();
  require(clocked.run_clock().native_cycle_units() == 1,
          "The rejected contract leaves the established clocked execution usable");
}

void interrupt_entry_guard() {
  constexpr unsigned status = 0xc00000;
  constexpr unsigned vector = 0x54;
  constexpr unsigned priority = 0xfffffe;
  constexpr unsigned load = 0xffff8e;
  constexpr unsigned compare = 0xffff8d;
  constexpr unsigned control = 0xffff8f;
  DspMachine machine;
  DspProgram code;
  code.immediate("sr", status);
  code.emit("nop");
  code.install(machine.bus());
  machine.bus().write({.address = vector});
  machine.bus().write({.address = vector + 1});
  for (const auto store : {DspStore{.space = DspSpace::x, .address = priority, .value = 0x100},
                           DspStore{.space = DspSpace::x, .address = load, .value = 1},
                           DspStore{.space = DspSpace::x, .address = compare, .value = 1},
                           DspStore{.space = DspSpace::x, .address = control, .value = 7}}) {
    machine.bus().write(store);
  }
  machine.cpu().step();
  machine.bus().timers().advance_clk2(1);
  const auto before = machine.cpu().debug_state();
  require(machine.bus().interrupt_request(0).has_value(), "A real timer IRQ is pending");
  const auto error =
      fault([&] { static_cast<void>(machine.begin_reference_instruction(boundaries)); });
  require(
      error.contains("Unqualified native reference interrupt entry") &&
          error.contains("vector=0x000054") && machine.cpu().debug_state() == before &&
          machine.bus().interrupt_request(0).has_value() && !machine.reference_pending(),
      "Unqualified vector suspension is visible before any acknowledgement or native execution");
  machine.cpu().step();
  require(!machine.bus().interrupt_request(0),
          "The established atomic vector path still acknowledges the actual timer source");
  execute(machine);
  require(machine.cpu().state().pc == boot + 3,
          "Ordinary access suspension remains usable after the explicitly dispatched vector");
}
}

void core_checks() {
  unknown_read_and_recovery();
  protected_destination();
  parallel_alu_and_modulo();
  unqualified_cache_controls();
  bit_admission();
  software_reset();
  loaded_repeat();
  profile_guards();
  interrupt_entry_guard();
}
}
