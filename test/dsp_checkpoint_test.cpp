// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pll.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <utility>

namespace blofeld::test::checkpoint {
namespace {
void test_registers_memory_and_branch_isolation() {
  constexpr unsigned initial_address = data + 6;
  constexpr unsigned wrapped_address = data + 1;
  constexpr unsigned known_pll = 0x04601d;
  DspMachine machine;
  DspProgram program;
  program.immediate("a0", 1);
  program.immediate("a1", 0);
  program.immediate("a2", 0);
  program.immediate("x0", 2);
  program.immediate("r0", initial_address);
  program.immediate("n0", 3);
  program.immediate("m0", 7);
  program.immediate("sr", masked_status);
  const auto add = program.pc();
  program.emit("add x0,a");
  program.emit("move a0,x:(r0)+n0");
  program.emit("move x0,y:(r0)");
  program.install(machine.bus());
  until(machine, add);
  machine.cpu().step();
  // Capture immediately after arithmetic, before a debug read canonicalizes its lazy flags.
  const auto saved = machine.checkpoint();
  const auto observed = machine.cpu().debug_state();
  constexpr std::uint64_t expected_sum = 0x2000001;
  require(observed.cpu.a == expected_sum && observed.cpu.sr == (masked_status | unnormalized),
          "Checkpoint capture must retain independently expected ADD value and flags");
  run(machine, 2);
  const auto first = machine.cpu().debug_state();
  require(machine.bus().read({.space = DspSpace::x, .address = initial_address}) == 1 &&
              machine.bus().read({.space = DspSpace::y, .address = wrapped_address}) == 2 &&
              first.cpu.r.at(0) == wrapped_address && first.cpu.n.at(0) == 3 &&
              first.cpu.m.at(0) == 7,
          "Native store/modulo addressing must use the original derived AGU state");
  machine.bus().write({.space = DspSpace::x, .address = initial_address, .value = cookie});
  machine.bus().pll().write(known_pll);
  machine.restore(saved);
  require(machine.cpu().debug_state() == observed,
          "Restore must retain every exposed CPU/stack field");
  static_cast<void>(
      rejects([&machine] { static_cast<void>(machine.bus().pll().read()); },
              "Snapshot must retain unknown reset PINIT/PEN rather than a later PLL write"));
  static_cast<void>(rejects(
      [&machine] {
        static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = initial_address}));
      },
      "An unknown checkpoint RAM cell must not inherit later writes"));
  run(machine, 2);
  require(machine.cpu().debug_state() == first &&
              machine.bus().read({.space = DspSpace::x, .address = initial_address}) == 1,
          "An immutable checkpoint must replay its stores independently of later branch mutations");
  DspMachine branch;
  branch.restore(saved);
  run(branch, 2);
  require(branch.cpu().debug_state() == first,
          "A trusted checkpoint must support an independent machine branch");
}

void test_latched_repeat_and_code_mutation() {
  DspMachine machine;
  DspProgram program;
  program.immediate("x0", 0);
  program.immediate("x1", 0);
  program.immediate("lc", 0);
  program.emit("jmp $0");
  program.install(machine.bus());
  ram(machine.bus(), 0, assemble("rep #3"));
  ram(machine.bus(), 1, assemble("move #>$123456,x0"));
  ram(machine.bus(), 3, assemble("jmp $1"));
  until(machine, 0);
  machine.cpu().step();
  const auto before_body = machine.checkpoint();
  machine.cpu().step();
  require(machine.cpu().state().pc == 1 && machine.cpu().state().lc == 2,
          "One native REP body leaves two repetitions at the latched body address");
  ram(machine.bus(), 1, assemble("move #>$654321,x1"));
  const auto latched = machine.checkpoint();
  run(machine, 2);
  require(
      machine.cpu().state().pc == 3 && machine.cpu().state().lc == 0 &&
          machine.cpu().state().x == cookie,
      "Remaining repetitions must execute the saved opcode/extension rather than changed P RAM");
  run(machine, 2);
  const auto completed = machine.cpu().debug_state();
  constexpr std::uint64_t expected_x = (std::uint64_t{other_cookie} << word_bits) | cookie;
  require(completed.cpu.x == expected_x && completed.cpu.pc == 3,
          "Normal execution after REP must see the changed opcode and immediate");
  machine.restore(latched);
  run(machine, 4);
  require(machine.cpu().debug_state() == completed,
          "Restored loaded REP must preserve latched code and later normal-fetch coherence");
  machine.restore(before_body);
  run(machine, 3);
  require(machine.cpu().state().x == cookie && machine.cpu().state().lc == 0,
          "A checkpoint before REP's first body fetch must retain the unloaded latch state");
}

void test_nested_do_and_fault_recovery() {
  DspMachine machine;
  DspProgram program;
  program.immediate("a0", 0);
  program.immediate("a1", 0);
  program.immediate("a2", 0);
  program.immediate("lc", 0);
  program.immediate("la", cookie);
  program.emit("jmp $0");
  program.install(machine.bus());
  // Assembler labels name the word after a DO body; architectural LA is label-1.
  const auto outer = assemble("do #2,$6");
  const auto inner = assemble("do #2,$5");
  require(outer.size() == 2 && outer.at(1) == 5 && inner.size() == 2 && inner.at(1) == 4,
          "Own nested DO encodings must retain the independently chosen LA endpoints");
  ram(machine.bus(), 0, outer);
  ram(machine.bus(), 2, inner);
  ram(machine.bus(), 4, assemble("inc a"));
  ram(machine.bus(), 5, assemble("nop"));
  ram(machine.bus(), 6, assemble("move x0,y:>$2000"));
  until(machine, 0);
  run(machine, 3);
  require(machine.cpu().state().a == 1 && machine.cpu().state().sp == 4 &&
              machine.cpu().state().lc == 1,
          "Nested DO must retain two independent native stack frames after its first increment");
  const auto saved = machine.checkpoint();
  until(machine, 6);
  const auto completed = machine.cpu().debug_state();
  require(completed.cpu.a == 4 && completed.cpu.sp == 0 && completed.cpu.la == cookie &&
              completed.cpu.lc == 0,
          "Two-by-two native DO loops must unwind to four increments and the original LA/LC: a=" +
              number(completed.cpu.a) + " sp=" + number(completed.cpu.sp) +
              " la=" + number(completed.cpu.la) + " lc=" + number(completed.cpu.lc));
  const auto fault = rejects([&machine] { machine.cpu().step(); }, "Reserved Y write must fail");
  require(fault.contains("Y:0x002000") && fault.contains("pc=0x000006"),
          "Post-checkpoint native faults must retain their exact bus/PC context");
  static_cast<void>(rejects([&machine] { static_cast<void>(machine.checkpoint()); },
                            "A rejected native step must not become a trusted checkpoint"));
  machine.restore(saved);
  until(machine, 6);
  require(machine.cpu().debug_state() == completed,
          "A valid snapshot must recover a faulted native core");
  auto copy = saved;
  const auto moved = std::move(copy);
  machine.restore(moved);
  until(machine, 6);
  require(machine.cpu().debug_state() == completed,
          "Moved checkpoint must remain independently usable");
}
}
}

int main() {
  try {
    blofeld::test::checkpoint::test_registers_memory_and_branch_isolation();
    blofeld::test::checkpoint::test_latched_repeat_and_code_mutation();
    blofeld::test::checkpoint::test_nested_do_and_fault_recovery();
    blofeld::test::checkpoint::test_checkpoint_interrupts();
    blofeld::test::checkpoint::test_checkpoint_peripherals();
    std::cout << "Native DSP checkpoint registers, memory, REP/DO, IRQ, peripheral phases and "
                 "recovery passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
