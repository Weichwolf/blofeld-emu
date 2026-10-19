// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

namespace blofeld::test::checkpoint {
void test_checkpoint_interrupts() {
  DspMachine machine;
  DspProgram program;
  program.immediate("sr", ordinary_status);
  program.emit("movep #>$100,x:<<$fffffe");
  program.emit("movep #>1,x:<<$ffff8e");
  program.emit("movep #>1,x:<<$ffff8d");
  program.emit("movep #>7,x:<<$ffff8f");
  program.emit("movep #>1,x:<<$ffff86");
  program.emit("movep #>1,x:<<$ffff85");
  const auto foreground = program.pc();
  program.emit("nop");
  program.emit("nop");
  program.install(machine.bus());
  ram(machine.bus(), timer_vector, assemble("jsr $300"));
  ram(machine.bus(), timer_vector + 1, assemble("nop"));
  ram(machine.bus(), second_vector, assemble("jsr $320"));
  ram(machine.bus(), second_vector + 1, assemble("nop"));
  const auto priority = assemble("movep #>$300,x:<<$fffffe");
  const auto enable = assemble("movep #>7,x:<<$ffff87");
  require(priority.size() == 2 && enable.size() == 2,
          "Own handler setup retains its two extension words");
  ram(machine.bus(), handler, priority);
  ram(machine.bus(), handler + 2, enable);
  ram(machine.bus(), handler + 4, assemble("inc a"));
  ram(machine.bus(), handler + 5, assemble("rti"));
  ram(machine.bus(), nested_handler, assemble("inc b"));
  ram(machine.bus(), nested_handler + 1, assemble("rti"));
  until(machine, foreground);
  machine.bus().timers().advance_clk2(1);
  require(machine.bus().interrupt_request(0).has_value(),
          "Explicit timer clock must supply a real pending source");
  const auto pending = machine.checkpoint();
  machine.cpu().step();
  require(machine.cpu().state().pc == handler && machine.cpu().state().sp == 1 &&
              (machine.cpu().state().sr & mask_bits) == 0x100,
          "Native low-priority vector must enter its own long handler and save foreground PC/SR");
  const auto low = machine.checkpoint();
  // All timers share one priority field. Native code raises it and enables timer two.
  run(machine, 2);
  machine.bus().timers().advance_clk2(1);
  const auto nested_pending = machine.checkpoint();
  machine.cpu().step();
  require(machine.cpu().state().pc == nested_handler && machine.cpu().state().sp == 2 &&
              (machine.cpu().state().sr & mask_bits) == mask_bits,
          "Higher-priority native vector must stack the first handler rather than replace it");
  const auto nested = machine.checkpoint();
  run(machine, 4);
  const auto completed = machine.cpu().debug_state();
  require(
      completed.cpu.pc == foreground && completed.cpu.sp == 0 && completed.cpu.a == 1 &&
          completed.cpu.b == 1 && completed.cpu.sr == ordinary_status,
      "Nested native INC/RTI pairs must restore the exact foreground status and both increments");
  machine.restore(nested);
  run(machine, 4);
  require(machine.cpu().debug_state() == completed,
          "Snapshot inside nested IRQ must preserve every native frame and counter");
  machine.restore(nested_pending);
  machine.cpu().step();
  run(machine, 4);
  require(machine.cpu().debug_state() == completed,
          "Pending nested source must replay the same native entry and unwind");
  machine.restore(low);
  run(machine, 4);
  require(machine.cpu().state().pc == foreground && machine.cpu().state().a == 1 &&
              machine.cpu().state().b == 0,
          "Earlier snapshot must not inherit later timer clock events or nested entry");
  machine.restore(pending);
  run(machine, 5);
  require(machine.cpu().state().pc == foreground && machine.cpu().state().sp == 0 &&
              machine.cpu().state().a == 1,
          "Snapshot before acknowledgement must retain the source and execute its native vector");
  machine.restore(pending);
  ram(machine.bus(), timer_vector, assemble("nop"));
  ram(machine.bus(), timer_vector + 1, assemble("nop"));
  machine.cpu().step();
  require(machine.cpu().state().pc == foreground && machine.cpu().state().sp == 0,
          "A fast native vector must return without a long frame");
  const auto fast = machine.checkpoint();
  machine.cpu().step();
  const auto after_fast = machine.cpu().debug_state();
  machine.restore(fast);
  machine.cpu().step();
  require(machine.cpu().debug_state() == after_fast,
          "Fast-entry continuation mode and diagnostic counters must replay");
}
}
