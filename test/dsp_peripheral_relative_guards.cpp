// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_peripheral_read_form.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_relative_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <iostream>
#include <optional>
#include <string>

namespace blofeld::test::peripheral_relative {
namespace {
void unknown_timer() {
  const Input input{.bit = timer_bit, .timer = true};
  DspMachine machine;
  const auto before = prepare(machine, input);
  machine.bus().timers().set_tio_drive(1, std::nullopt);
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 3);
  const auto saved = run.checkpoint(machine);
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    run.restore(saved, machine);
    const auto message = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "Unknown TIO must not become a fabricated Ready branch");
    require(message.contains("Unknown DSP timer DI read") && message.contains("X:0xffff8b") &&
                message.contains("pc=" + hex(entry, 6)) && message.contains("omr=") &&
                run.faulted() && run.completed_cycles() == 3 &&
                machine.cpu().debug_state() == before,
            "Unknown timer read fails before PC/CCR/native success with full access context");
    run.restore(saved, machine);
    machine.bus().timers().set_tio_drive(1, true);
    pipeline::clocks(run, machine, 10);
    auto expected = branch_result(before, {.bit = timer_bit, .level = true, .timer = true});
    ++expected.cpu.pc;
    ++expected.native_instructions;
    ++expected.native_cycles;
    require(machine.cpu().debug_state() == expected,
            "A trusted restore plus a real known TIO observation enables the actual false branch");
  }
}

void extension_failure() {
  DspMachine machine;
  const Input input{.bit = timer_bit, .timer = true};
  const std::array code{opcode(input)};
  fetch_clock::prepare(machine, code, {.execution = pipeline::execution});
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 2);
  const auto saved = run.checkpoint(machine);
  for (unsigned repeat = 0; repeat < 2; ++repeat) {
    run.restore(saved, machine);
    const auto message = checkpoint::rejects(
        [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
        "Missing actual extension must fail before a peripheral sample");
    require(message.contains("P:0x000101") && !message.contains("timer DI") && run.faulted() &&
                run.completed_cycles() == 2 && machine.cpu().debug_state() == before,
            "Extension F2 failure precedes the unknown TIO read and all architectural effects");
  }
}

void unused_target() {
  constexpr unsigned unknown = 0x800;
  const Input input{.level = true, .displacement = unknown - entry};
  DspMachine machine;
  const auto before = prepare(machine, input);
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 13);
  auto expected = branch_result(before, input);
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  require(machine.cpu().debug_state() == expected,
          "A false relative branch never fetches its unmapped target");
}

void partial_gpio(const bool level) {
  const Input input{.level = level, .flags = true};
  DspMachine machine;
  const auto before = prepare(machine, input);
  auto &gpio = machine.bus().gpio(DspPort::c);
  gpio.write({.reg = DspGpioRegister::control, .value = 1});
  gpio.write({.reg = DspGpioRegister::direction});
  gpio.set_drive({.mask = 1, .levels = static_cast<unsigned>(level)});
  static_cast<void>(
      checkpoint::rejects([&] { static_cast<void>(gpio.read(DspGpioRegister::data)); },
                          "The fixture must retain unknown neighboring input bits"));
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  pipeline::clocks(run, machine, 7);
  require(machine.cpu().debug_state() == branch_result(before, input),
          "Native branch tests only its known selected pin without inventing neighboring inputs");
}

void fresh_busy_poll() {
  const Input input{.bit = timer_bit, .flags = true, .timer = true, .displacement = 0};
  DspMachine machine;
  auto expected = prepare(machine, input);
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(contract(), machine);
  unsigned captures = 0;
  unsigned branches = 0;
  constexpr unsigned end = 23;
  for (unsigned count = 1; count <= end; ++count) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = pipeline::stages(cycle);
    if (cycle.bus.peripheral_read) { ++captures; }
    if (count == 12) { machine.bus().timers().set_tio_drive(1, true); }
    if (!phase.retirement_published) { continue; }
    if (phase.execute2 && phase.execute2->address == entry) {
      ++branches;
      const Input observed{.bit = timer_bit,
                           .level = branches == 3,
                           .flags = true,
                           .timer = true,
                           .displacement = 0};
      expected = branch_result(expected, observed);
    } else {
      require(count == end, "Only the selected fallthrough NOP follows three actual busy polls");
      ++expected.cpu.pc;
      ++expected.native_instructions;
      ++expected.native_cycles;
    }
    require(machine.cpu().debug_state() == expected,
            "Every poll consumes a fresh real sample; Ready changes no CPU state by itself");
  }
  require(captures == 3 && branches == 3 && expected.cpu.pc == entry + 3,
          "Two taken busy polls followed by one false Ready poll have exact5/6-period timing");
}

struct Encoding {
  bool set = false, y = false;
  unsigned address = 0, bit = 0;
};

void encoding(const Encoding input) {
  const auto high = input.address >= high_begin;
  const auto base = high ? high_clear : low_clear;
  const auto prefix = base | (input.set ? set_bit : 0U) | (input.y ? space_bit : 0U) |
                      ((input.address - (high ? high_begin : low_begin)) << operand_shift);
  const auto form = decode_peripheral_read(prefix | input.bit);
  require(form && form->relative && form->words == 2 && form->set == input.set &&
              form->bit == input.bit && form->target.address == input.address &&
              form->target.space == (input.y ? DspSpace::y : DspSpace::x),
          "Each independent relative format retains its exact fixed operand");
  constexpr unsigned first_invalid = 24;
  for (unsigned invalid = first_invalid; invalid < word_bits + 8; ++invalid) {
    require(!decode_peripheral_read(prefix | invalid),
            "Reserved bit indices remain unqualified instead of aliasing a valid bit");
  }
}

void encodings() {
  // Independent48 NXP encodings: pp/qq endpoints,X/Y,bits0,12,23,clear/set.
  for (const bool set : {false, true}) {
    for (const bool y : {false, true}) {
      for (const auto address : {0xffff80U, 0xffffbfU, 0xffffc0U, 0xffffffU}) {
        for (const auto bit : {0U, 12U, 23U}) {
          encoding({.set = set, .y = y, .address = address, .bit = bit});
        }
      }
    }
  }
}
}

void guard_checks() {
  encodings();
  unknown_timer();
  extension_failure();
  unused_target();
  for (const bool level : {false, true}) { partial_gpio(level); }
  fresh_busy_poll();
  std::cout << "Relative encoding/unknown/F2/partial-bit/fresh TIO busy-poll controls passed\n";
}
}
