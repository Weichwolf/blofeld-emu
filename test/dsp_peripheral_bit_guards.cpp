// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_bit_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "native_access_fixture.hpp"

#include <cstdint>
#include <string>

namespace blofeld::test::peripheral_bit {
namespace {
constexpr unsigned unknown_once_bit = 0x0abc09;  // FM13-24: BCLR #9,X:fffffc.
constexpr unsigned reserved_gpio_set = 0x013d37; // FM13-36: BSET #23,X:ffffbd.
constexpr unsigned pdr_c = 0xffffbd;
constexpr unsigned pin_mask = 0xfff;
constexpr unsigned source = 0x400;
constexpr unsigned destination = 0x410;
constexpr unsigned once = 0xfffffc;
constexpr unsigned canary = 0x234567;
constexpr unsigned body_pc = boot + 2;
constexpr unsigned padding = 24;
constexpr unsigned bit_index_mask = 31;

DspDebugState unit(DspMachine &machine, const unsigned word, const bool following_read = false) {
  DspProgram code;
  code.immediate("sr", status);
  code.word(word);
  if (following_read) { code.emit("movep x:<<$ffffbd,x0"); }
  for (unsigned index = 0; index < padding; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, 1);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body_pc, "One native SR seed enters the exact own bit body");
  return before;
}

void gpio(DspMachine &machine) {
  auto &port = machine.bus().gpio(DspPort::c);
  port.write({.reg = DspGpioRegister::control});
  port.write({.reg = DspGpioRegister::direction, .value = pin_mask});
  port.write({.reg = DspGpioRegister::data, .value = pin_mask});
}

void encoding(const Operation operation,
              const DspSpace space,
              const unsigned address,
              const unsigned bit) {
  const bool high = address >= high_begin;
  const auto base = high ? operation.high : operation.low;
  const auto word = base | ((address - (high ? high_begin : peripheral_begin)) << 8U) |
                    (space == DspSpace::y ? 0x40U : 0U) | bit;
  const auto form = decode_memory_stage(word);
  require(form && form->peripheral && form->peripheral_bit && form->peripheral->space == space &&
              form->peripheral->address == address && form->peripheral_bit->bit == bit &&
              form->peripheral_bit->operation == operation.kind && !form->peripheral_register,
          "All48 Freescale-pinned encodings retain exact bit/operation/space/address");
  constexpr unsigned over_word = 0x1000000;
  require(!decode_memory_stage(word | over_word), "Extra opcode bits cannot alias native IO");
  for (unsigned invalid = word_bits; invalid < 32; ++invalid) {
    require(!decode_memory_stage((word & ~bit_index_mask) | invalid),
            "Bits24..31 are not a DSP56300 memory operand");
  }
}

void encodings() {
  unsigned checked = 0;
  for (const auto operation : operations) {
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      for (const unsigned address : {0xffff90U, 0xfffffcU}) {
        for (const unsigned bit : {0U, 9U, 23U}) {
          encoding(operation, space, address, bit);
          ++checked;
        }
      }
    }
  }
  constexpr unsigned expected = 48;
  require(checked == expected, "Every independently assembled short-I/O family was checked");
}

void selective() {
  for (unsigned operation = 0; operation < operations.size(); ++operation) {
    for (const bool old : {false, true}) {
      DspMachine machine;
      auto expected = unit(machine, opcode({.operation = operation}));
      auto &port = machine.bus().gpio(DspPort::c);
      port.write({.reg = DspGpioRegister::control, .value = 1, .mask = 1});
      port.set_drive({.mask = 1, .levels = old ? 1U : 0U});
      static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
      DspFetchRun run(contract(), machine);
      pipeline::clocks(run, machine, 7);
      ++expected.cpu.pc;
      expected.cpu.sr = status | static_cast<unsigned>(old);
      ++expected.native_instructions;
      expected.native_cycles += 2;
      require(
          machine.cpu().debug_state() == expected,
          "The real bit callback needs only its observed GPIO input and preserves all other state");
      const auto error = native_access::fault(
          [&] { static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = pdr_c})); });
      require(error.contains("Unknown DSP GPIO data read"),
              "A selective bit operation must not invent observations for other GPIO pins");
    }
  }
}

void timer_w1c() {
  constexpr unsigned set_timer_enable = 0x010f20; // FM13-36: BSET #0,X:ffff8f.
  constexpr unsigned timer_control = 0xffff8f;
  constexpr unsigned wrap_load = 0xfffffe;
  constexpr unsigned flags = 0x300000; // DSP56371UM TCSR TOF/TCF, both W1C.
  DspMachine machine;
  auto expected = unit(machine, set_timer_enable);
  auto &timers = machine.bus().timers();
  timers.write({.address = timer_control - 1, .value = wrap_load});
  timers.write({.address = timer_control - 2, .value = word_mask});
  timers.write({.address = timer_control, .value = 1});
  timers.advance_clk2(3);
  require(timers.read(timer_control, flags) == flags,
          "Real compare and overflow clocks establish both independent W1C flags");
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 5);
  const auto saved = run.checkpoint(machine);
  ++expected.cpu.pc;
  expected.cpu.sr = status | 1U;
  ++expected.native_instructions;
  expected.native_cycles += 2;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    require(cycle.bus.memory && cycle.bus.memory->data.kind == DspNativeAccessKind::bit &&
                cycle.bus.memory->data.value == 1 &&
                machine.bus().timers().read(timer_control, flags) == 0 &&
                machine.bus().timers().read(timer_control, 1) == 1,
            "Native timer RMW writes back both unselected W1C flags and returns the old TE bit");
    pipeline::clocks(run, machine, 1);
    require(machine.cpu().debug_state() == expected,
            "Actual bit carry and two-clock retirement preserve complete CPU and stack state");
  }
}

void carry_branch(const unsigned operation, const bool old) {
  constexpr unsigned carry_branch_word = 0x058410; // FM13-18: BCS, signed displacement16.
  constexpr unsigned displacement = 16;
  constexpr unsigned false_marker = 0x123456;
  constexpr unsigned true_marker = 0x654321;
  constexpr unsigned budget = 96;
  const Input input{.operation = operation, .bit = 9, .old = old, .carry_in = !old};
  DspMachine machine;
  DspProgram code;
  code.immediate("sr", status | static_cast<unsigned>(!old));
  code.word(opcode(input));
  const auto branch = code.pc();
  code.word(carry_branch_word);
  code.emit("movep #>" + number(false_marker) + ",x:<<" + number(once));
  while (code.pc() < branch + displacement) { code.word(0); }
  code.emit("movep #>" + number(true_marker) + ",x:<<" + number(once));
  for (unsigned index = 0; index < padding; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, 1);
  gpio(machine);
  machine.bus().gpio(DspPort::c).write({.reg = DspGpioRegister::data, .value = seed(input)});
  auto expected = machine.cpu().debug_state();
  expected.cpu.pc = branch + (old ? displacement : 1U);
  expected.cpu.sr = status | static_cast<unsigned>(old);
  expected.native_instructions += 2;
  expected.native_cycles += 6;
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 6);
  const auto saved = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    bool reached = false;
    for (unsigned cycle = 0; cycle < budget; ++cycle) {
      const auto point = run.advance_cycle(machine, fetch_clock::input());
      if (!point.bus.memory || point.bus.memory->data.target.address != once) { continue; }
      require(machine.bus().once_latch() == (old ? true_marker : false_marker) &&
                  machine.cpu().debug_state() == expected &&
                  machine.bus().read({.space = DspSpace::x, .address = pdr_c}) == value(input),
              "The genuine adjacent BCS uses the bit's old value, opposite the incoming carry");
      reached = true;
      break;
    }
    require(reached, "Own bit/BCS composition must reach its independently selected native output");
  }
}

enum class Failure : std::uint8_t { unknown, reserved, selected, overlap, dma };

void failure(const Failure kind) {
  DspMachine machine;
  unsigned word = opcode({});
  if (kind == Failure::unknown) { word = unknown_once_bit; }
  if (kind == Failure::reserved) { word = reserved_gpio_set; }
  const auto before = unit(machine, word, kind == Failure::overlap);
  if (kind != Failure::selected) { gpio(machine); }
  if (kind == Failure::selected) {
    machine.bus().gpio(DspPort::c).write({.reg = DspGpioRegister::control, .value = 1, .mask = 1});
  }
  if (kind == Failure::unknown) {
    machine.bus().write({.space = DspSpace::x, .address = once, .value = canary});
  }
  if (kind == Failure::dma) {
    for (const auto store :
         {DspStore{.space = DspSpace::x, .address = dma_test::source0, .value = source},
          DspStore{.space = DspSpace::x, .address = dma_test::destination0, .value = destination},
          DspStore{.space = DspSpace::x, .address = dma_test::counter0, .value = 0},
          DspStore{
              .space = DspSpace::x, .address = dma_test::control0, .value = dma_test::word_copy}}) {
      machine.bus().write(store);
    }
    require(machine.bus().has_active_dma(), "The negative control has a real enabled DMA channel");
  }
  const auto outputs = machine.bus().gpio(DspPort::c).outputs();
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 5);
  const auto saved = run.checkpoint(machine);
  std::string first;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = native_access::fault(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); });
    require(error.contains("pc=") && error.contains("omr=") && run.faulted() &&
                run.completed_cycles() == 5 && machine.cpu().debug_state() == before &&
                machine.bus().gpio(DspPort::c).outputs().levels == outputs.levels,
            "Each strict bit failure retains context, CPU/clock/GPIO state and quarantine");
    switch (kind) {
      case Failure::unknown:
        require(error.contains("X:0xfffffc") && error.contains("bit E1 callback/route") &&
                    machine.bus().once_latch() == canary,
                "An unsupported native bit route fails before any fallback whole-word read/write");
        break;
      case Failure::reserved:
        require(error.contains("reserved-bit write"), "Reserved GPIO write remains strict");
        break;
      case Failure::selected:
        require(error.contains("Unknown DSP GPIO data read"),
                "Unknown selected GPIO bit remains strict");
        break;
      case Failure::overlap:
        require(error.contains("atomic peripheral bit/early-read overlap"),
                "Unqualified bit RMW/early-read ties reject before either peripheral effect");
        break;
      case Failure::dma:
        require(error.contains("with active DMA"),
                "Unqualified peripheral/DMA resource remains strict");
        break;
      default: throw Fault("Invalid own bit failure scenario");
    }
    if (replay == 0) { first = error; }
    require(error == first, "Joint restoration reproduces the exact native failure frontier");
  }
}
}

void guard_checks() {
  encodings();
  selective();
  timer_w1c();
  for (unsigned operation = 0; operation < operations.size(); ++operation) {
    carry_branch(operation, false);
    carry_branch(operation, true);
  }
  for (const auto kind :
       {Failure::unknown, Failure::reserved, Failure::selected, Failure::overlap, Failure::dma}) {
    failure(kind);
  }
}
}
