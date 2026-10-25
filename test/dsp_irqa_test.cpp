// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>

namespace {
using namespace blofeld;
using blofeld::test::DspProgram;
using blofeld::test::require;
using namespace blofeld::test::checkpoint;
constexpr unsigned iprc = 0xffffff;
constexpr unsigned irqa_vector = 0x10;
constexpr unsigned shi_vector = 0x44;
constexpr unsigned edge_ipl1 = 6;
constexpr unsigned irq_mask = 3;
constexpr unsigned handler_address = 0x300;
constexpr unsigned absolute_jsr = 0x0bf080;
constexpr unsigned timer_ipl2 = 0x300;

void check_priorities(const DspInterrupts &controller,
                      const unsigned external,
                      const unsigned shi,
                      const unsigned timer,
                      const unsigned mask) {
  constexpr std::array<std::uint32_t, 3> pending{timer_vector, shi_vector, irqa_vector};
  std::optional<DspInterruptRequest> expected;
  if (external > mask && external >= shi && external >= timer) {
    expected = DspInterruptRequest{.vector = irqa_vector, .level = external - 1};
  } else if (shi > mask && shi >= timer) {
    expected = DspInterruptRequest{.vector = shi_vector, .level = shi - 1};
  } else if (timer > mask) {
    expected = DspInterruptRequest{.vector = timer_vector, .level = timer - 1};
  }
  require(controller.select(pending, mask) == expected,
          "NXP tables2-3/2-4/2-6: greatest eligible IPL, then IRQA before SHI before TIM");
}

void test_driver_resolution() {
  DspBus bus;
  bus.write({.space = DspSpace::x, .address = iprc, .value = 1});
  bus.observe_irqa_driver({});
  require(!bus.interrupt_request(0), "Table2-6 internal pull-up deasserts a released IRQA pad");
  bus.observe_irqa_driver({.active = 1, .known = 1});
  require(bus.interrupt_request(0) == DspInterruptRequest{.vector = irqa_vector, .level = 0},
          "A known strong low dominates the internal pull-up");
  bus.observe_irqa_driver({.active = 1, .known = 1, .levels = 1, .reduced = 1});
  require(!bus.interrupt_request(0),
          "A known reduced-drive high still resolves against passive bias");
  bus.observe_irqa_driver({.active = 1});
  static_cast<void>(rejects([&] { static_cast<void>(bus.interrupt_request(0)); },
                            "Unknown active drive must not become the pull-up voltage"));
  bus.observe_irqa_driver({.pull_down = 1});
  static_cast<void>(rejects([&] { static_cast<void>(bus.interrupt_request(0)); },
                            "Opposed passive pulls have no invented voltage winner"));
}

void test_priority_matrix() {
  DspInterrupts controller;
  for (unsigned external = 0; external < 4; ++external) {
    controller.write(iprc, external);
    for (unsigned shi = 0; shi < 4; ++shi) {
      for (unsigned timer = 0; timer < 4; ++timer) {
        controller.write(iprp, (shi << 2U) | (timer << 8U));
        for (unsigned mask = 0; mask < 4; ++mask) {
          check_priorities(controller, external, shi, timer, mask);
        }
      }
    }
  }
}

void test_level_and_disabled_detection() {
  DspInterrupts controller;
  require(!controller.irqa_request(0), "Disabled IRQA must not probe an unknown input");
  controller.write(iprc, 1);
  require(!controller.irqa_request(irq_mask), "A masked source does not probe its unknown pin");
  const auto fault = rejects([&] { static_cast<void>(controller.irqa_request(0)); },
                             "Unmasked unknown IRQA must fail");
  require(fault.contains("Unknown DSP IRQA input") && fault.contains("X:0xffffff"),
          "An unobserved active-low input retains its register context");
  controller.observe_irqa(true);
  require(!controller.irqa_request(0), "High is inactive for a level-sensitive IRQA");
  controller.observe_irqa(false);
  require(controller.irqa_request(0) == DspInterruptRequest{.vector = irqa_vector},
          "Low requests vector10 at encoded IPL0");
  controller.acknowledge_irqa();
  require(controller.irqa_request(0).has_value(),
          "NXP 2.3.2.1: service cannot clear a held low level");
  controller.observe_irqa(true);
  require(!controller.irqa_request(0), "External release removes the level request");
  controller.write(iprc, 0);
  controller.observe_irqa(false);
  controller.write(iprc, edge_ipl1);
  require(!controller.irqa_request(0),
          "An edge observed while disabled cannot replay when priority is enabled");
  controller.observe_irqa(true);
  controller.observe_irqa(false);
  require(controller.irqa_request(0) == DspInterruptRequest{.vector = irqa_vector, .level = 1},
          "Only the actual high-to-low transition latches the edge request");
}

void test_edge_retention_unknown_history_and_reset() {
  DspInterrupts controller;
  controller.observe_irqa(true);
  controller.write(iprc, edge_ipl1);
  controller.observe_irqa(false);
  require(!controller.irqa_request(irq_mask), "CPU masking does not service the detection latch");
  controller.observe_irqa(true);
  require(controller.irqa_request(0).has_value(), "An edge remains pending after pin release");
  controller.acknowledge_irqa();
  require(!controller.irqa_request(0), "Acknowledgement clears only the captured edge");
  controller.observe_irqa(false);
  controller.acknowledge_irqa();
  controller.observe_irqa(false);
  require(!controller.irqa_request(0), "A held low does not invent a second falling edge");
  controller.observe_irqa(std::nullopt);
  controller.observe_irqa(true);
  const auto fault = rejects([&] { static_cast<void>(controller.irqa_request(0)); },
                             "Unknown edge history cannot become an inactive pin");
  require(fault.contains("edge history") && fault.contains("vector=0x10"),
          "Withdrawn edge observations retain their explicit diagnostic");
  controller.write(iprc, 0);
  controller.write(iprc, edge_ipl1);
  require(!controller.irqa_request(0), "Disabling resets the detection latch before rearming");
  controller.observe_irqa(false);
  controller.observe_irqa(std::nullopt);
  require(controller.irqa_request(0).has_value(),
          "A known captured edge remains serviceable despite subsequent unknown input");
  controller.acknowledge_irqa();
  static_cast<void>(rejects([&] { static_cast<void>(controller.irqa_request(0)); },
                            "Service does not invent later external pin observations"));
  controller.observe_irqa(false);
  controller.reset();
  controller.write(iprc, 1);
  require(controller.irqa_request(0).has_value(), "Reset retains the explicitly held low pin");
  controller.write(iprc, 0);
  controller.write(iprc, edge_ipl1);
  require(!controller.irqa_request(0), "Reset/disable erase the old edge, not the held input");
}

void test_register_rejections() {
  DspInterrupts controller;
  controller.observe_irqa(true);
  controller.write(iprc, edge_ipl1);
  for (const auto value : {8U, 0x40U, 0x1000000U, 2U}) {
    static_cast<void>(rejects([&] { controller.write(iprc, value); },
                              "Unsupported fields/live trigger-mode changes must reject"));
    require(controller.read(iprc) == edge_ipl1, "A rejected change preserves IPRC");
  }
  static_cast<void>(rejects([&] { controller.acknowledge_irqa(); },
                            "Acknowledgement cannot fabricate a pending edge"));
  static_cast<void>(rejects([&] { static_cast<void>(controller.irqa_request(4)); },
                            "Invalid CPU mask must fail before pin selection"));
}

void test_bus_arbitration_and_masked_pin() {
  DspBus bus;
  bus.observe_irqa(false);
  bus.write({.space = DspSpace::x, .address = iprc, .value = 1});
  bus.write({.space = DspSpace::x, .address = iprp, .value = 0x100});
  bus.timers().write({.address = csr0 - 1, .value = 5});
  bus.timers().write({.address = csr0 - 2, .value = 5});
  bus.timers().write({.address = csr0, .value = 7});
  bus.timers().advance_clk2(1);
  require(bus.interrupt_request(0) == DspInterruptRequest{.vector = irqa_vector},
          "Actual pending IRQA outranks an equal-level pending timer");
  bus.observe_irqa(true);
  require(bus.interrupt_request(0) == DspInterruptRequest{.vector = timer_vector},
          "Releasing IRQA leaves the actual timer request pending");
  bus.observe_irqa(std::nullopt);
  bus.write({.space = DspSpace::x, .address = iprp, .value = timer_ipl2});
  require(bus.interrupt_request(1) == DspInterruptRequest{.vector = timer_vector, .level = 2},
          "A masked unknown external pin cannot hide an eligible higher-level timer");
}

void test_admitted_source_contract() {
  DspBus bus;
  bus.observe_irqa(true);
  bus.write({.space = DspSpace::x, .address = iprc, .value = edge_ipl1});
  bus.observe_irqa(false);
  const DspInterruptRequest admitted{.vector = irqa_vector, .level = 1};
  bus.validate_admitted_interrupt(admitted);
  bus.write({.space = DspSpace::x, .address = iprp, .value = timer_ipl2});
  bus.timers().write({.address = csr0 - 1, .value = 5});
  bus.timers().write({.address = csr0 - 2, .value = 5});
  bus.timers().write({.address = csr0, .value = 7});
  bus.timers().advance_clk2(1);
  bus.acknowledge_admitted_interrupt(admitted);
  require(bus.interrupt_request(0) == DspInterruptRequest{.vector = timer_vector, .level = 2},
          "Acknowledging an admitted edge leaves a later higher-level timer pending");
  bus.write({.space = DspSpace::x, .address = iprc, .value = 0});
  bus.write({.space = DspSpace::x, .address = iprc, .value = 2});
  bus.validate_admitted_interrupt(admitted);
  bus.observe_irqa(true);
  const auto fault = rejects([&] { bus.acknowledge_admitted_interrupt(admitted); },
                             "The conservative admitted contract must reject source withdrawal");
  require(fault.contains("withdrew") && fault.contains("000010"),
          "An unqualified source-hold change must retain its actual vector context");
}

unsigned prepare(DspMachine &machine, const unsigned setting, const bool long_vector) {
  DspProgram program;
  program.immediate("sr", ordinary_status);
  program.emit("movep #>" + blofeld::test::number(setting) + ",x:<<$ffffff");
  const auto foreground = program.pc();
  program.immediate("y0", 9);
  program.emit("nop");
  program.install(machine.bus());
  machine.bus().observe_irqa(true);
  if (long_vector) {
    // DSP56300FM JSR absolute extension: independent own diagnostic encoding.
    constexpr std::array call{absolute_jsr, handler_address};
    ram(machine.bus(), irqa_vector, call);
    ram(machine.bus(), handler_address, assemble("move #>7,x0"));
    ram(machine.bus(), handler_address + 2, assemble("rti"));
  } else {
    ram(machine.bus(), irqa_vector, assemble("nop"));
    ram(machine.bus(), irqa_vector + 1, assemble("nop"));
  }
  until(machine, foreground);
  return foreground;
}

void test_native_long_entry_and_checkpoint() {
  DspMachine machine;
  const auto foreground = prepare(machine, edge_ipl1, true);
  const auto before = machine.cpu().state();
  machine.bus().observe_irqa(false);
  const auto captured = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    if (replay != 0) { machine.restore(captured); }
    machine.cpu().step();
    const auto entry = machine.cpu().state();
    require(entry.pc == handler_address && entry.sp == 1 && entry.stack_pc == foreground &&
                entry.stack_sr == before.sr && entry.sr == (ordinary_status | 0x200U),
            "IRQA uses its real vector JSR/frame and raises IPL1 to native mask2");
    require(!machine.bus().interrupt_request(0), "Vector acknowledgement consumes the edge once");
    machine.cpu().step();
    machine.cpu().step();
    const auto returned = machine.cpu().state();
    require(returned.pc == foreground && returned.sp == 0 && returned.sr == before.sr &&
                (returned.x & blofeld::test::word_mask) == 7,
            "Actual native handler and RTI restore foreground state without a host return");
    machine.cpu().step();
    require((machine.cpu().state().y & blofeld::test::word_mask) == 9,
            "Original foreground continuation runs after the real return");
  }
}

void test_native_masked_edge_release() {
  DspMachine machine;
  DspProgram program;
  program.immediate("sr", ordinary_status | mask_bits);
  program.emit("movep #>6,x:<<$ffffff");
  const auto unmask = program.pc();
  program.immediate("sr", ordinary_status);
  const auto foreground = program.pc();
  program.emit("nop");
  program.install(machine.bus());
  constexpr std::array call{absolute_jsr, handler_address};
  ram(machine.bus(), irqa_vector, call);
  machine.bus().observe_irqa(true);
  until(machine, unmask);
  machine.bus().observe_irqa(false);
  machine.bus().observe_irqa(true);
  machine.cpu().step();
  require(machine.cpu().state().pc == foreground && machine.cpu().state().sp == 0,
          "A masked edge cannot bypass the actual native SR-unmask instruction");
  machine.cpu().step();
  require(machine.cpu().state().pc == handler_address && machine.cpu().state().sp == 1 &&
              machine.cpu().state().stack_pc == foreground,
          "The retained edge enters its real vector at the following logical boundary");
}

void test_native_level_and_missing_vector() {
  DspMachine level;
  const auto foreground = prepare(level, 1, false);
  level.bus().observe_irqa(false);
  for (unsigned count = 0; count < 2; ++count) {
    level.cpu().step();
    require(level.cpu().state().pc == foreground && level.cpu().state().sp == 0,
            "Held-level IRQA repeats its two-word fast vector before foreground executes");
  }
  level.bus().observe_irqa(true);
  level.cpu().step();
  require((level.cpu().state().y & blofeld::test::word_mask) == 9,
          "Releasing the level permits the real foreground instruction");

  DspMachine missing;
  DspProgram program;
  program.immediate("sr", ordinary_status);
  program.emit("movep #>6,x:<<$ffffff");
  const auto waiting = program.pc();
  program.emit("nop");
  program.install(missing.bus());
  missing.bus().observe_irqa(true);
  ram(missing.bus(), irqa_vector, assemble("nop"));
  until(missing, waiting);
  missing.bus().observe_irqa(false);
  const auto before = missing.cpu().state();
  const auto fault = rejects([&] { missing.cpu().step(); }, "Missing second vector word must fail");
  require(fault.contains("000011") && missing.cpu().state() == before &&
              missing.bus().interrupt_request(0).has_value(),
          "Both vector words must exist before IRQA is acknowledged or CPU/frame state changes");
}
}

int main() {
  try {
    test_driver_resolution();
    test_priority_matrix();
    test_level_and_disabled_detection();
    test_edge_retention_unknown_history_and_reset();
    test_register_rejections();
    test_bus_arbitration_and_masked_pin();
    test_admitted_source_contract();
    test_native_long_entry_and_checkpoint();
    test_native_masked_edge_release();
    test_native_level_and_missing_vector();
    std::cout
        << "IRQA priorities, level/edge/reset/history, native vector/RTI and checkpoint passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
