// SPDX-License-Identifier: GPL-3.0-only
#include "dma_fixture.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "dsp_dma_interrupt_fixture.hpp"
#include "isa.hpp"

#include <cstdint>
#include <iostream>
#include <optional>

namespace blofeld::test::dma_irq {
namespace {
using namespace dma_test;
constexpr unsigned ordinary_status = 0xc00000, handler = 0x300;
// Independently authored native setup; selectors and EA fields remain explicit.
constexpr unsigned sr_selector = 57, r7_selector = 23;
constexpr unsigned selector_mask = 0x370000, high_selector_mask = 24;
constexpr unsigned immediate_ea = 0xb400, absolute_ea = 0x3000;
constexpr auto move_sr = dsp56300::isa::opcode<dsp56300::isa::Opcode::movec_effective>().front() |
                         immediate_ea | (sr_selector & 31U);
constexpr auto move_r7 =
    (dsp56300::isa::opcode<dsp56300::isa::Opcode::move_x_effective>().front() & ~selector_mask) |
    ((r7_selector & 7U) << 16U) | ((r7_selector & high_selector_mask) << 17U) | immediate_ea;
constexpr auto absolute_jsr =
    dsp56300::isa::opcode<dsp56300::isa::Opcode::jsr_effective>().front() | absolute_ea;
constexpr auto nop = dsp56300::isa::nop.front(), rti = dsp56300::isa::rti.front();
constexpr unsigned word_bits = 24;
constexpr unsigned payload = 0x123456, sentinel = 0x654321, foreground_value = 0x77;
constexpr unsigned dma_control = word_copy | interrupt_enable;

void write(DspBus &bus, const unsigned address, const unsigned value) {
  bus.write({.space = DspSpace::x, .address = address, .value = value});
}

unsigned prepare(DspMachine &machine,
                 const unsigned channel,
                 const bool long_vector,
                 const bool complete_vector = true) {
  NativeProgram program;
  program.word(move_sr);
  program.word(ordinary_status);
  program.write(
      {.space = DspSpace::x, .address = iprc, .value = 1U << (priority_shift + (2 * channel))});
  program.write({.space = DspSpace::x, .address = source0 - (4 * channel), .value = ring_start});
  program.write(
      {.space = DspSpace::x, .address = destination0 - (4 * channel), .value = destination});
  program.write({.space = DspSpace::x, .address = counter0 - (4 * channel), .value = 0});
  program.write({.space = DspSpace::x, .address = control0 - (4 * channel), .value = dma_control});
  const auto foreground = program.pc();
  program.word(move_r7);
  program.word(foreground_value);
  reset_native(machine, program);
  write(machine.bus(), ring_start, payload);
  machine.bus().write({.space = DspSpace::y, .address = destination, .value = sentinel});
  const auto vector = first_vector + (2 * channel);
  machine.bus().write({.address = vector, .value = long_vector ? absolute_jsr : nop});
  if (complete_vector) {
    machine.bus().write({.address = vector + 1, .value = long_vector ? handler : nop});
  }
  if (long_vector) {
    machine.bus().write({.address = handler, .value = nop});
    machine.bus().write({.address = handler + 1, .value = rti});
  }
  static_cast<void>(
      dma(machine, {.cycle = 0, .known_requests = 1, .timing = timing, .grant = grant}));
  for (unsigned instruction = 0; instruction < 6; ++instruction) { step_native(machine); }
  require(machine.cpu().registers().pc == foreground &&
              machine.cpu().registers().sr == ordinary_status &&
              machine.cpu().progress().instructions == 6,
          "Actual reset and six literal MOVEC/MOVEP instructions establish the foreground");
  return foreground;
}

std::optional<DspDmaWord> tick(DspMachine &machine, const std::uint64_t cycle) {
  return dma(
      machine,
      {.cycle = cycle, .known_requests = 1, .requests = 1, .timing = timing, .grant = grant});
}

void admit(DspMachine &machine) {
  const auto pc = machine.cpu().registers().pc;
  require(
      machine.admit_interrupt({.address = pc, .opcode = move_r7, .extension = foreground_value}) &&
          machine.interrupt_pending(),
      "Actual unmasked DMA event admits the independently authored foreground instruction");
}

void vector_service(DspMachine &machine, const DspInterruptRequest request) {
  admit(machine);
  const auto first = machine.cpu().instruction_fetch();
  require(first.exception_vector && first.address == request.vector &&
              machine.bus().interrupt_request(0) == request,
          "Admitted vector entry retains the actual source before native execution");
  step_native(machine);
  require(!machine.interrupt_pending() && !machine.bus().interrupt_request(0) &&
              machine.bus().read({.space = DspSpace::x, .address = status}) == all_done,
          "The actual two-word vector fetch acknowledges one source without clearing DTD");
  const auto second = machine.cpu().instruction_fetch();
  if (second.exception_vector) {
    require(second.address == request.vector + 1 && second.latched_opcode == nop,
            "Fast entry retains the genuinely fetched second NOP after acknowledgement");
    step_native(machine);
  }
}

void completed_service(DspMachine &machine, const unsigned channel, const bool long_vector) {
  auto expected = machine.cpu().registers();
  const auto before = machine.cpu().progress();
  const auto ticks = machine.clock().elapsed_ticks();
  require(machine.bus().read({.space = DspSpace::y, .address = destination}) == sentinel &&
              !machine.bus().interrupt_request(0),
          "Held source snapshots retain the old destination and no completion");
  const auto word = tick(machine, 2);
  const DspInterruptRequest request{.vector = first_vector + (2 * channel)};
  require(word && word->channel == channel && word->value == payload &&
              machine.bus().read({.space = DspSpace::y, .address = destination}) == payload &&
              machine.bus().interrupt_request(0) == request,
          "Only the actual destination store raises this channel's interrupt");
  vector_service(machine, request);
  if (long_vector) {
    const auto frame = (std::uint64_t{expected.pc} << word_bits) | expected.sr;
    expected.stack[1] = frame;
    auto entry = expected;
    entry.pc = handler;
    entry.sp = 1;
    entry.sc = 1;       // FM SC counts the one stacked interrupt frame.
    entry.sr |= 0x100U; // FM IPL0 entry executes at mask1.
    require(machine.cpu().registers() == entry,
            "Real JSR preserves every register and stacks literal foreground PC/SR at mask1");
    step_native(machine);
    step_native(machine);
  }
  require(machine.cpu().registers() == expected &&
              machine.cpu().progress().instructions ==
                  before.instructions + (long_vector ? 3U : 2U),
          "Actual fast NOPs or long NOP/RTI restore the complete foreground register set");
  step_native(machine);
  expected.pc += 2;
  expected.r[7] = foreground_value;
  require(machine.cpu().registers() == expected && machine.clock().elapsed_ticks() == ticks,
          "Real foreground MOVE changes only R7/PC; untimed native service invents no clock");
}

void native_service(const unsigned channel, const bool long_vector) {
  DspMachine machine(native_clocks);
  prepare(machine, channel, long_vector);
  require(!tick(machine, 1) && !machine.bus().interrupt_request(0),
          "Actual source capture alone cannot signal DMA completion");
  completed_service(machine, channel, long_vector);
}

void generate(DspBus &bus, const unsigned channel) {
  write(bus, source0 - (4 * channel), ring_start);
  write(bus, destination0 - (4 * channel), destination);
  write(bus, counter0 - (4 * channel), 0);
  write(bus, control0 - (4 * channel), dma_control);
}

void bus_arbitration() {
  DspBus bus;
  write(bus, ring_start, payload);
  static_cast<void>(
      bus.advance_dma({.cycle = 0, .known_requests = 1, .timing = timing, .grant = grant}));
  generate(bus, 0);
  generate(bus, 5);
  constexpr unsigned mixed = 0xc01003; // IRQA IPL2, DMA0 IPL0, DMA5 IPL2.
  write(bus, iprc, mixed);
  bus.observe_irqa(false);
  constexpr unsigned timer_csr = 0xffff8f;
  constexpr unsigned timer_priority = 0x300;
  write(bus, iprp, timer_priority);
  bus.timers().write({.address = timer_csr - 1, .value = 5});
  bus.timers().write({.address = timer_csr - 2, .value = 5});
  bus.timers().write({.address = timer_csr, .value = 7});
  bus.timers().advance_clk2(1);
  for (unsigned cycle = 1; cycle <= 4; ++cycle) {
    static_cast<void>(bus.advance_dma(
        {.cycle = cycle, .known_requests = 1, .requests = 1, .timing = timing, .grant = grant}));
  }
  const DspInterruptRequest first{.vector = first_vector};
  const DspInterruptRequest last{.vector = first_vector + 10, .level = 2};
  constexpr unsigned last_priority = 0xc00000;
  constexpr unsigned timer_vector = 0x54;
  require(bus.interrupt_request(0) == DspInterruptRequest{.vector = 0x10, .level = 2},
          "Real IRQA precedes simultaneous DMA and TIM at the same IPL");
  bus.observe_irqa(true);
  require(bus.interrupt_request(0) == last && !bus.interrupt_request(3),
          "Released IRQA leaves DMA5 ahead of an equal-IPL timer and all lower DMA requests");
  static_cast<void>(rejects([&] { bus.acknowledge_interrupt(first); }));
  require(bus.interrupt_request(0) == last,
          "A mismatched ordinary acknowledgement cannot consume either DMA event");
  bus.validate_admitted_interrupt(first);
  bus.acknowledge_admitted_interrupt(first);
  require(bus.interrupt_request(0) == last,
          "Service of an already admitted lower DMA event preserves later higher requests");
  write(bus, iprc, mixed & ~last_priority);
  static_cast<void>(rejects([&] { bus.acknowledge_admitted_interrupt(last); }));
  write(bus, iprc, mixed);
  require(bus.interrupt_request(0) == last, "Priority changes do not erase a latched DMA event");
  bus.acknowledge_interrupt(last);
  require(bus.interrupt_request(0) == DspInterruptRequest{.vector = timer_vector, .level = 2},
          "DMA acknowledgement preserves the independently pending real timer");
}

void missing_vector_and_recovery() {
  DspMachine machine(native_clocks);
  const auto foreground = prepare(machine, 1, false, false);
  static_cast<void>(tick(machine, 1));
  require(tick(machine, 2).has_value(), "Actual DMA1 destination store establishes the IRQ");
  constexpr unsigned vector = first_vector + 2;
  const DspInterruptRequest request{.vector = vector};
  const auto before = machine.cpu().registers();
  const auto count = machine.cpu().progress();
  admit(machine);
  const auto result = machine.run_unclocked();
  require(result.retired == 0 && result.fault.kind == dsp56300::FaultKind::memory_read &&
              result.fault.space == dsp56300::Space::program &&
              result.fault.address == vector + 1 && machine.cpu().registers() == before &&
              machine.cpu().progress() == count && machine.interrupt_pending() &&
              machine.bus().interrupt_request(0) == request && machine.io().failure(),
          "A refused second vector word preserves the genuine source and retires no instruction");
  machine.bus().write({.address = vector + 1, .value = nop});
  const auto refused = machine.run_unclocked();
  require(refused.retired == 0 && refused.fault == result.fault &&
              machine.cpu().registers() == before && machine.cpu().progress() == count &&
              machine.interrupt_pending() && machine.bus().interrupt_request(0) == request,
          "Repairing RAM alone cannot clear a sticky core fault or acknowledge the source");
  require(prepare(machine, 1, false) == foreground,
          "Actual hardware reset reinitializes the original native setup");
  require(!tick(machine, 1) && tick(machine, 2).has_value(),
          "Recovery raises a new completion only after another real destination store");
  vector_service(machine, request);
  require(machine.cpu().registers() == before,
          "Reset recovery fetches both real vector words and restores the complete foreground");
}

void disabled_delivery_entry() {
  DspMachine machine(native_clocks);
  prepare(machine, 1, false);
  static_cast<void>(tick(machine, 1));
  require(tick(machine, 2).has_value(), "Actual DMA1 completion precedes native entry");
  admit(machine);
  write(machine.bus(), control0 - 4, word_copy & ~enable);
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto result = machine.run_unclocked();
  constexpr unsigned second = first_vector + 3;
  require(result.retired == 0 && result.fault.kind == dsp56300::FaultKind::memory_read &&
              result.fault.space == dsp56300::Space::program && result.fault.address == second &&
              machine.cpu().registers() == before && machine.cpu().progress() == progress &&
              machine.interrupt_pending(),
          "A disabled delivery gate refuses acknowledgement at the actual second vector fetch");
  write(machine.bus(), control0 - 4, dma_control & ~enable);
  require(machine.bus().interrupt_request(0) == DspInterruptRequest{.vector = first_vector + 2},
          "Rejected acknowledgement preserves the original DMA latch behind its delivery gate");
}

}

void native_checks() {
  for (unsigned channel = 0; channel < channels; ++channel) {
    for (const bool long_vector : {false, true}) { native_service(channel, long_vector); }
  }
  bus_arbitration();
  missing_vector_and_recovery();
  disabled_delivery_entry();
  std::cout << "PASS DMA_NATIVE_IRQ contexts=12 reset-recovery=1 disabled-ack=1\n";
}
}
