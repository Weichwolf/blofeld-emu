// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_retirement_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <variant>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::retirement;
namespace access = blofeld::test::reference;
using native_access::fault;
constexpr unsigned address = 7;
constexpr unsigned ccr_mask = 0xff;
constexpr std::uint64_t unit = std::uint64_t{1} << word_bits;

void parallel_load_result() {
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", address);
  code.immediate("m0", address);
  code.immediate("x1", 2);
  code.immediate("a1", 1);
  code.immediate("sr", masked_status | carry | overflow | zero | negative);
  const auto body = code.pc();
  code.emit("add x1,a x:(r0)+,x0");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = address, .value = access::first});
  checkpoint::run(machine, 5);
  const auto before = machine.cpu().debug_state();
  auto progress = machine.begin_reference_instruction(execution);
  progress = machine.advance_reference_access(access::ticket(progress));
  require(access::request(progress).kind == DspNativeAccessKind::read &&
              access::request(progress).target.address == address,
          "The real parallel instruction reaches its actual modulo-addressed data load");
  progress = machine.advance_reference_access(access::ticket(progress));
  const auto held = result(progress);
  require(access::completed(progress).value == access::first && held.pc == body &&
              held.next_pc == body + 1 && held.omr == reset_omr && held.next_omr == reset_omr &&
              !held.redirected && machine.cpu().debug_state() == before,
          "Sampled memory, ALU, CCR and modulo AGU result remain separate from CPU publication");
  const auto saved = machine.checkpoint();
  const auto stale = access::ticket(progress);
  DspMachine fork;
  fork.restore(saved);
  static_cast<void>(fault([&] { static_cast<void>(fork.advance_reference_access(stale)); }));
  const auto forked = fork.advance_reference_access(access::ticket(fork));
  require(forked.retired && fork.cpu().state().a == 3 * unit &&
              fork.cpu().state().x == 2 * unit + access::first && fork.cpu().state().r[0] == 0,
          "A trusted fresh-machine fork retains the actual CPU candidate with a fresh owner");
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(stale)); }));
    static_cast<void>(fault([&] { machine.cpu().step(); }));
    static_cast<void>(
        fault([&] { static_cast<void>(machine.begin_reference_instruction(execution)); }));
    machine.bus().write({.space = DspSpace::x, .address = address, .value = access::second});
    const std::array changed{0U};
    machine.bus().install_rom({.start = body, .words = changed});
    const auto current = access::ticket(machine);
    const auto retired = machine.advance_reference_access(current);
    const auto after = machine.cpu().debug_state();
    const auto *completed =
        retired.completed ? std::get_if<DspInstructionRetire>(&*retired.completed) : nullptr;
    require(retired.retired && !retired.pending && completed != nullptr && *completed == held &&
                after.cpu.pc == body + 1 && after.cpu.a == 3 * unit &&
                after.cpu.x == 2 * unit + access::first && after.cpu.r[0] == 0 &&
                after.cpu.m[0] == address && (after.cpu.sr & ccr_mask) == unnormalized &&
                after.native_instructions == before.native_instructions + 1 &&
                after.native_cycles == before.native_cycles + 1,
            "Retirement publishes independently expected held data/arithmetic/flags/AGU once, "
            "without rereading overwritten code/data");
    static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(current)); }));
    require(machine.cpu().debug_state() == after,
            "A duplicate retirement cannot repeat the instruction");
  }
}

void committed_store_is_not_replayed() {
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", address);
  code.immediate("m0", address);
  code.immediate("x0", access::first);
  code.immediate("x1", 2);
  code.immediate("a1", 1);
  const auto body = code.pc();
  code.emit("add x1,a x0,x:(r0)+");
  code.install(machine.bus());
  checkpoint::run(machine, 5);
  const auto before = machine.cpu().debug_state();
  const auto held = complete_effects(machine, machine.begin_reference_instruction(execution));
  require(result(held).pc == body && machine.cpu().debug_state() == before &&
              machine.bus().read({.space = DspSpace::x, .address = address}) == access::first,
          "The actual store commits while the complete CPU result is still held");
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    machine.bus().write({.space = DspSpace::x, .address = address, .value = access::third});
    const auto retired = machine.advance_reference_access(access::ticket(machine));
    require(retired.retired && machine.cpu().state().a == 3 * unit &&
                machine.cpu().state().r[0] == 0 && machine.cpu().state().pc == body + 1 &&
                machine.bus().read({.space = DspSpace::x, .address = address}) == access::third,
            "Retiring a saved candidate cannot replay its earlier real store over a later write");
  }
}

void destructive_read_is_not_replayed() {
  constexpr unsigned shi_control = 0xffff91;
  constexpr unsigned shi_configuration = 0xa9;
  DspMachine machine;
  DspProgram code;
  code.immediate("x0", access::third);
  code.emit("movep x:<<$ffff94,x0");
  code.install(machine.bus());
  machine.cpu().step();
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x, .address = shi_control, .value = shi_configuration});
  access::send(machine.bus().shi(), access::first);
  access::send(machine.bus().shi(), access::second);
  const auto before = machine.cpu().debug_state();
  const auto held = complete_effects(machine, machine.begin_reference_instruction(execution));
  static_cast<void>(result(held));
  require(machine.cpu().debug_state() == before, "The destructive read does not publish X0 early");
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    require(machine.bus().read({.space = DspSpace::x, .address = access::fifo}) == access::second,
            "Exactly the first actual FIFO word was consumed before retirement");
    const auto retired = machine.advance_reference_access(access::ticket(machine));
    require(retired.retired && machine.cpu().state().x == access::first,
            "The held actual FIFO result retires even after the FIFO has become empty");
    require(fault([&] {
              static_cast<void>(
                  machine.bus().read({.space = DspSpace::x, .address = access::fifo}));
            }).contains("empty DSP SHI FIFO"),
            "Retirement performs neither another FIFO read nor a peripheral reset");
  }
}

void loaded_repeat_has_retirement_boundary() {
  DspMachine machine;
  DspProgram code;
  code.emit("rep #3");
  const auto body = code.pc();
  code.emit("nop");
  code.emit("nop");
  code.install(machine.bus());
  machine.cpu().step();
  for (const unsigned remaining : {2U, 1U, 0U}) {
    const auto before = machine.cpu().debug_state();
    const auto begun = machine.begin_reference_instruction(execution);
    if (remaining != 2) {
      static_cast<void>(result(begun));
      require(!begun.completed, "A cached NOP REP body reaches retirement without a bus access");
    }
    const auto held = complete_effects(machine, begun);
    require(machine.cpu().debug_state() == before,
            "Loaded REP does not bypass explicit retirement");
    const auto saved = machine.checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      machine.restore(saved);
      const auto retired = machine.advance_reference_access(access::ticket(machine));
      require(retired.retired && machine.cpu().state().lc == remaining &&
                  machine.cpu().state().pc == (remaining == 0 ? body + 1 : body) &&
                  machine.cpu().debug_state().native_instructions == before.native_instructions + 1,
              "Each saved real REP continuation publishes its literal counter/PC once");
    }
    static_cast<void>(held);
  }
}

void unknown_access_and_reset() {
  DspMachine machine;
  DspProgram code;
  code.emit("move x:>$660,x0");
  code.install(machine.bus());
  auto progress = machine.begin_reference_instruction(execution);
  progress = machine.advance_reference_access(access::ticket(progress));
  progress = machine.advance_reference_access(access::ticket(progress));
  const auto before_read = machine.checkpoint();
  const auto error =
      fault([&] { static_cast<void>(machine.advance_reference_access(access::ticket(progress))); });
  require(error.contains("Unknown DSP memory read at X:0x000660") &&
              error.contains("pc=0xff0000") && error.contains("omr=0x000305"),
          "Deferred execution retains the actual unknown access and instruction context");
  static_cast<void>(fault([&] { static_cast<void>(machine.checkpoint()); }));
  machine.restore(before_read);
  machine.bus().write({.space = DspSpace::x, .address = access::source, .value = access::first});
  progress = machine.advance_reference_access(access::ticket(machine));
  static_cast<void>(result(progress));
  const auto held = machine.checkpoint();
  const auto stale = access::ticket(progress);
  machine.cpu().reset_spi_boot();
  static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(stale)); }));
  require(!machine.reference_pending() && machine.cpu().state().pc == boot,
          "Actual reset discards the CPU candidate and invalidates its pending retirement");
  machine.restore(held);
  const auto retired = machine.advance_reference_access(access::ticket(machine));
  require(retired.retired && machine.cpu().state().x == access::first &&
              machine.cpu().state().pc == boot + 2,
          "Only trusted restore can recover the prior held native result after reset");
}

void interrupt_acknowledgement_is_not_replayed() {
  constexpr unsigned timer_vector = 0x54;
  constexpr unsigned control = 0xffff8f;
  constexpr unsigned load = 0xffff8e;
  constexpr unsigned compare = 0xffff8d;
  constexpr unsigned priorities = 0xfffffe;
  constexpr unsigned priority0 = 0x100;
  constexpr unsigned unmasked = 0xc00000;
  constexpr unsigned first_result = 0x12;
  constexpr unsigned second_result = 0x34;
  DspMachine machine;
  DspProgram code;
  code.immediate("sr", unmasked);
  code.emit("nop");
  code.install(machine.bus());
  machine.cpu().step();
  const auto first_move = checkpoint::assemble("move #<$12,r0");
  const auto second_move = checkpoint::assemble("move #<$34,r4");
  require(first_move.size() == 1 && second_move.size() == 1,
          "The own fast vector consists of two actual single-word native moves");
  const std::array vector{first_move.front(), second_move.front()};
  checkpoint::ram(machine.bus(), timer_vector, vector);
  const auto raise = [&] {
    for (const auto store :
         {DspStore{.space = DspSpace::x, .address = control},
          DspStore{.space = DspSpace::x, .address = priorities, .value = priority0},
          DspStore{.space = DspSpace::x, .address = load, .value = 1},
          DspStore{.space = DspSpace::x, .address = compare, .value = 1},
          DspStore{.space = DspSpace::x, .address = control, .value = 7}}) {
      machine.bus().write(store);
    }
    machine.bus().timers().advance_clk2(1);
  };
  raise();
  const auto before = machine.cpu().debug_state();
  const auto held = complete_effects(machine, machine.begin_reference_interrupt(execution));
  static_cast<void>(result(held));
  require(machine.cpu().debug_state() == before && !machine.bus().interrupt_request(0),
          "The actual vector acknowledgement completes while both native results remain held");
  const auto saved = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    raise();
    const auto retired = machine.advance_reference_access(access::ticket(machine));
    const auto after = machine.cpu().debug_state();
    require(
        retired.retired && after.cpu.pc == before.cpu.pc && after.cpu.r[0] == first_result &&
            after.cpu.r[4] == second_result && after.stack_high == before.stack_high &&
            after.stack_low == before.stack_low &&
            after.native_instructions == before.native_instructions + 2 &&
            machine.bus().interrupt_request(0).has_value(),
        "Held fast-vector retirement preserves literal results and the later real pending source");
  }
}
}

int main() {
  try {
    parallel_load_result();
    committed_store_is_not_replayed();
    destructive_read_is_not_replayed();
    loaded_repeat_has_retirement_boundary();
    unknown_access_and_reset();
    interrupt_acknowledgement_is_not_replayed();
    fetch_checks();
    std::cout << "Deferred native DSP retirement checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
