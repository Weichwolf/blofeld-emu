// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test::dma_test;
constexpr unsigned iprc = 0xffffff;
constexpr unsigned irqa_vector = 0x10;
constexpr unsigned shi_vector = 0x44;
constexpr unsigned edge_ipl1 = 6;
constexpr unsigned irq_mask = 3;
constexpr unsigned handler_address = 0x300;
constexpr unsigned absolute_jsr = 0x0bf080;
constexpr unsigned timer_ipl2 = 0x300;

constexpr unsigned iprp = 0xfffffe, csr0 = 0xffff8f, timer_vector = 0x54;
constexpr unsigned ordinary_status = 0xc00000, mask_bits = 0x300;
// FM MOVEC immediate SR, MOVE immediate X0/Y0 and RTI.
constexpr unsigned move_sr = 0x05f439, move_x0 = 0x44f400, move_y0 = 0x46f400;
constexpr unsigned rti = 4, word_bits = 24, effect_bound = 32;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

std::string rejects(const auto &operation, const char *message) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault(message);
}

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

struct Setup {
  unsigned priority;
  bool long_vector = false;
  bool foreground_nop = false;
  bool complete_vector = true;
};

void vectors(OwnDspMachine &machine, const Setup setup) {
  auto &bus = machine.bus();
  bus.write({.address = irqa_vector, .value = setup.long_vector ? absolute_jsr : 0});
  if (setup.complete_vector) {
    bus.write({.address = irqa_vector + 1, .value = setup.long_vector ? handler_address : 0});
  }
  if (setup.long_vector) {
    bus.write({.address = handler_address, .value = move_x0});
    bus.write({.address = handler_address + 1, .value = 7});
    bus.write({.address = handler_address + 2, .value = rti});
  }
}

unsigned prepare(OwnDspMachine &machine, const Setup setup) {
  NativeProgram program;
  program.word(move_sr);
  program.word(ordinary_status);
  program.write({.space = DspSpace::x, .address = iprc, .value = setup.priority});
  const auto foreground = program.pc();
  if (setup.foreground_nop) { program.word(0); }
  program.word(move_y0);
  program.word(9);
  program.word(0);
  reset_native(machine, program);
  vectors(machine, setup);
  step_native(machine);
  step_native(machine);
  require(machine.cpu().registers().pc == foreground &&
              machine.cpu().registers().sr == ordinary_status &&
              machine.cpu().progress().instructions == 2,
          "Real reset and two literal MOVEC/MOVEP instructions establish the IRQA foreground");
  return foreground;
}

void service(OwnDspMachine &machine,
             const dsp56300::InstructionWords foreground,
             const DspInterruptRequest request,
             const bool long_vector) {
  const auto before = machine.cpu().progress();
  const auto ticks = machine.clock().elapsed_ticks();
  require(machine.bus().interrupt_request(0) == request && machine.admit_interrupt(foreground),
          "The actual eligible IRQA source admits the installed foreground instruction");
  unsigned reads = 0;
  for (unsigned instruction = 0;
       instruction < 2 && machine.cpu().instruction_fetch().exception_vector;
       ++instruction) {
    if (instruction == 1) {
      require(machine.cpu().instruction_fetch().latched_opcode == 0 && !machine.interrupt_pending(),
              "The second fast NOP consumes its accepted opcode after peripheral acknowledgement");
    }
    auto progress = machine.begin_fetch();
    for (unsigned effect = 0; effect < effect_bound && progress.pending && !progress.retired;
         ++effect) {
      const auto pending = progress.pending.value();
      const bool fetch = pending.effect.kind == dsp56300::EffectKind::fetch_opcode ||
                         pending.effect.kind == dsp56300::EffectKind::fetch_extension;
      if (fetch) {
        require(reads < 2 && pending.effect.space == dsp56300::Space::program &&
                    pending.effect.address == irqa_vector + reads && machine.interrupt_pending() &&
                    machine.bus().interrupt_request(0) == request,
                "Both actual vector reads precede acknowledgement of the selected IRQA source");
      }
      progress = machine.advance(pending.ticket);
      require(progress.fault.kind == dsp56300::FaultKind::none,
              "Actual IRQA vector remains healthy");
      if (fetch) {
        ++reads;
        require(machine.interrupt_pending() == (reads == 1),
                "IRQA acknowledgement ownership ends only after the second actual vector read");
      }
    }
    require(progress.retired, "Each genuine IRQA vector instruction retires within its bound");
  }
  require(reads == 2 && !machine.interrupt_pending() &&
              machine.cpu().progress().instructions ==
                  before.instructions + (long_vector ? 1U : 2U) &&
              machine.clock().elapsed_ticks() == ticks,
          "Real JSR or both fast NOPs retire without inventing physical clock periods");
}

void long_service(OwnDspMachine &machine, const dsp56300::InstructionWords foreground) {
  auto expected = machine.cpu().registers();
  service(machine, foreground, {.vector = irqa_vector, .level = 1}, true);
  expected.stack[1] = (std::uint64_t{expected.pc} << word_bits) | expected.sr;
  auto entry = expected;
  entry.pc = handler_address;
  entry.sp = 1;
  entry.sc = 1;
  entry.sr |= 0x200U; // FM IPL1 entry executes at mask2.
  require(machine.cpu().registers() == entry && !machine.bus().interrupt_request(0),
          "Real IRQA JSR stacks literal PC/SR and changes only PC/SP/SC/frame/mask");
  step_native(machine);
  entry.pc += 2;
  entry.x = 7;
  require(machine.cpu().registers() == entry,
          "The actual handler MOVE changes only X0 and its instruction PC");
  step_native(machine);
  expected.x = 7;
  require(machine.cpu().registers() == expected,
          "Actual RTI restores complete foreground state and retains the genuine stack frame");
}

struct NativeResult {
  dsp56300::Registers registers;
  dsp56300::Progress progress;
  std::uint64_t ticks;
  bool operator==(const NativeResult &) const = default;
};

void replay(OwnDspMachine &machine, const auto &continuation) {
  const auto saved = machine.checkpoint();
  std::optional<NativeResult> completed;
  for (unsigned iteration = 0; iteration < 4; ++iteration) {
    OwnDspMachine fresh(native_clocks);
    auto &owner = iteration == 3 ? fresh : machine;
    owner.restore(saved);
    const auto ticks = owner.clock().elapsed_ticks();
    continuation(owner);
    const NativeResult result{.registers = owner.cpu().registers(),
                              .progress = owner.cpu().progress(),
                              .ticks = owner.clock().elapsed_ticks()};
    require(result.ticks == ticks, "Untimed native IRQA service cannot manufacture board time");
    if (completed) { require(result == *completed, "Same/fresh owners replay all native state"); }
    completed = result;
  }
}

void test_native_long_entry_and_checkpoint() {
  OwnDspMachine machine(native_clocks);
  const auto foreground = prepare(machine, {.priority = edge_ipl1, .long_vector = true});
  machine.bus().observe_irqa(false);
  machine.bus().observe_irqa(true); // The captured edge must survive actual pin release.
  replay(machine, [foreground](OwnDspMachine &owner) {
    auto expected = owner.cpu().registers();
    const auto before = owner.cpu().progress();
    long_service(owner, {.address = foreground, .opcode = move_y0, .extension = 9});
    expected.stack[1] = (std::uint64_t{foreground} << word_bits) | expected.sr;
    expected.x = 7;
    expected.y = 9;
    expected.pc += 2;
    step_native(owner);
    require(owner.cpu().registers() == expected &&
                owner.cpu().progress().instructions == before.instructions + 4,
            "Real vector/handler/RTI/foreground execution preserves every other register");
  });
}

void test_native_masked_edge_release() {
  OwnDspMachine machine(native_clocks);
  NativeProgram program;
  program.word(move_sr);
  program.word(ordinary_status | mask_bits);
  program.write({.space = DspSpace::x, .address = iprc, .value = edge_ipl1});
  const auto unmask = program.pc();
  program.word(move_sr);
  program.word(ordinary_status);
  const auto foreground = program.pc();
  program.word(0);
  reset_native(machine, program);
  vectors(machine, {.priority = edge_ipl1, .long_vector = true});
  step_native(machine);
  step_native(machine);
  machine.bus().observe_irqa(false);
  machine.bus().observe_irqa(true);
  replay(machine, [unmask, foreground](OwnDspMachine &owner) {
    auto expected = owner.cpu().registers();
    const auto before = owner.cpu().progress();
    require(!owner.admit_interrupt(
                {.address = unmask, .opcode = move_sr, .extension = ordinary_status}) &&
                owner.cpu().registers() == expected,
            "Masked IRQA cannot bypass or mutate the actual SR-unmask instruction");
    step_native(owner);
    expected.pc = foreground;
    expected.sr = ordinary_status;
    require(owner.cpu().registers() == expected, "Actual MOVEC changes only PC/SR before entry");
    long_service(owner, {.address = foreground, .opcode = 0});
    expected.stack[1] = (std::uint64_t{foreground} << word_bits) | expected.sr;
    expected.x = 7;
    ++expected.pc;
    step_native(owner);
    require(owner.cpu().registers() == expected &&
                owner.cpu().progress().instructions == before.instructions + 5,
            "Released masked edge executes actual unmask/JSR/MOVE/RTI/foreground NOP");
  });
}

void test_native_level_repeat() {
  OwnDspMachine machine(native_clocks);
  const auto foreground = prepare(machine, {.priority = 1, .foreground_nop = true});
  machine.bus().observe_irqa(false);
  replay(machine, [foreground](OwnDspMachine &owner) {
    auto expected = owner.cpu().registers();
    const auto before = owner.cpu().progress();
    const DspInterruptRequest request{.vector = irqa_vector};
    service(owner, {.address = foreground, .opcode = 0}, request, false);
    require(
        owner.cpu().registers() == expected && owner.bus().interrupt_request(0) == request &&
            !owner.admit_interrupt({.address = foreground, .opcode = 0}),
        "Service retains a held low level; the conservative post-vector hold consumes no event");
    step_native(
        owner); // Explicit normal-stream NOP; physical arbitration latency remains unqualified.
    ++expected.pc;
    service(owner, {.address = foreground + 1, .opcode = move_y0, .extension = 9}, request, false);
    require(
        owner.cpu().registers() == expected && owner.bus().interrupt_request(0) == request,
        "The same held level drives a second genuine fast vector without a fabricated new edge");
    owner.bus().observe_irqa(true);
    require(!owner.bus().interrupt_request(0), "Actual pin release removes the level request");
    step_native(owner);
    expected.pc += 2;
    expected.y = 9;
    require(owner.cpu().registers() == expected &&
                owner.cpu().progress().instructions == before.instructions + 6 &&
                owner.cpu().progress().estimated_cycles == before.estimated_cycles + 7,
            "Released foreground changes only Y0/PC after four fast NOPs and two real foreground "
            "instructions");
  });
}

void test_native_missing_vector() {
  OwnDspMachine machine(native_clocks);
  const auto foreground = prepare(machine, {.priority = edge_ipl1, .complete_vector = false});
  machine.bus().observe_irqa(false);
  replay(machine, [foreground](OwnDspMachine &owner) {
    auto expected = owner.cpu().registers();
    const auto before = owner.cpu().progress();
    const auto trusted = owner.checkpoint();
    const DspInterruptRequest request{.vector = irqa_vector, .level = 1};
    require(owner.admit_interrupt({.address = foreground, .opcode = move_y0, .extension = 9}),
            "Actual pending edge admits the known foreground before vector reads");
    auto progress = owner.begin_fetch();
    if (!progress.pending) { throw Fault("Missing actual first-vector fetch"); }
    progress = owner.advance(progress.pending->ticket);
    if (!progress.pending) { throw Fault("Missing actual second-vector fetch"); }
    require(progress.pending->effect.kind == dsp56300::EffectKind::fetch_extension &&
                progress.pending->effect.address == irqa_vector + 1,
            "Both actual vector reads precede any first-slot retirement");
    progress = owner.advance(progress.pending->ticket);
    require(progress.fault.kind == dsp56300::FaultKind::memory_read &&
                progress.fault.space == dsp56300::Space::program &&
                progress.fault.address == irqa_vector + 1 && progress.fault.pc == irqa_vector &&
                owner.cpu().registers() == expected && owner.cpu().progress() == before &&
                owner.interrupt_pending() && owner.bus().interrupt_request(0) == request &&
                owner.io().failure() && !owner.cpu().instruction_outcome(),
            "Refused second read reports its actual address/PC without acknowledging the edge or "
            "retiring");
    owner.restore(trusted);
    owner.bus().write({.address = irqa_vector + 1, .value = 0});
    service(owner, {.address = foreground, .opcode = move_y0, .extension = 9}, request, false);
    require(owner.cpu().registers() == expected && !owner.bus().interrupt_request(0),
            "Trusted recovery needs both actual reads before clearing the captured edge");
    step_native(owner);
    expected.pc += 2;
    expected.y = 9;
    require(owner.cpu().registers() == expected &&
                owner.cpu().progress().instructions == before.instructions + 3,
            "Recovered fast vector permits the real foreground continuation");
  });
}

void test_native_level_and_missing_vector() {
  test_native_level_repeat();
  test_native_missing_vector();
  std::cout << "PASS IRQA_NATIVE contexts=4 continuations=16 missing-vector=4\n";
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
