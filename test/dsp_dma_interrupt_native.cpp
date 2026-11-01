// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_dma_interrupt_fixture.hpp"

#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>

namespace blofeld::test::dma_irq {
namespace {
using namespace dma_test;
constexpr unsigned ordinary_status = 0xc00000, handler = 0x300;
// FM MOVEC immediate SR, MOVE immediate R7, absolute JSR, NOP and RTI.
constexpr unsigned move_sr = 0x05f439, move_r7 = 0x67f400, absolute_jsr = 0x0bf080;
constexpr unsigned nop = 0, rti = 4, effect_bound = 32;
constexpr unsigned word_bits = 24;
constexpr unsigned payload = 0x123456, sentinel = 0x654321, foreground_value = 0x77;
constexpr unsigned dma_control = word_copy | interrupt_enable;

void write(DspBus &bus, const unsigned address, const unsigned value) {
  bus.write({.space = DspSpace::x, .address = address, .value = value});
}

unsigned prepare(OwnDspMachine &machine,
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

std::optional<DspDmaWord> tick(OwnDspMachine &machine, const std::uint64_t cycle) {
  return dma(
      machine,
      {.cycle = cycle, .known_requests = 1, .requests = 1, .timing = timing, .grant = grant});
}

void admit(OwnDspMachine &machine) {
  const auto pc = machine.cpu().registers().pc;
  require(
      machine.admit_interrupt({.address = pc, .opcode = move_r7, .extension = foreground_value}) &&
          machine.interrupt_pending(),
      "Actual unmasked DMA event admits the independently authored foreground instruction");
}

void drain(OwnDspMachine &machine, dsp56300::InstructionProgress progress) {
  const auto before = machine.cpu().progress().instructions;
  for (unsigned effect = 0; effect < effect_bound && progress.pending && !progress.retired &&
                            progress.fault.kind == dsp56300::FaultKind::none;
       ++effect) {
    progress = machine.advance(progress.pending->ticket);
  }
  require(progress.retired && progress.fault.kind == dsp56300::FaultKind::none &&
              machine.cpu().progress().instructions == before + 1,
          "Actual native continuation retires once without a substituted service result");
}

void vector_service(OwnDspMachine &machine, const DspInterruptRequest request) {
  admit(machine);
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
                    pending.effect.address == request.vector + reads &&
                    machine.interrupt_pending() && machine.bus().interrupt_request(0) == request,
                "Actual vector reads preserve address, source latch and acknowledgement ownership");
      }
      progress = machine.advance(pending.ticket);
      require(progress.fault.kind == dsp56300::FaultKind::none,
              "Native vector execution must not fault");
      if (fetch) {
        ++reads;
        require(machine.interrupt_pending() == (reads == 1) &&
                    machine.bus().interrupt_request(0).has_value() == (reads == 1),
                "Only the second accepted native vector read acknowledges the DMA latch");
      }
    }
    require(progress.retired, "Each real vector instruction retires within its bound");
  }
  require(reads == 2 && !machine.interrupt_pending() && !machine.bus().interrupt_request(0) &&
              machine.bus().read({.space = DspSpace::x, .address = status}) == all_done,
          "Two genuine vector reads consume one IRQ while retaining all DMA DTD bits");
}

struct Result {
  dsp56300::Registers registers;
  dsp56300::Progress progress;
  std::uint64_t ticks;
  bool operator==(const Result &) const = default;
};

Result completed_service(OwnDspMachine &machine, const unsigned channel, const bool long_vector) {
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
  const auto pending = machine.checkpoint();
  machine.restore(pending);
  require(machine.bus().interrupt_request(0) == request,
          "Joint restore retains the real pending event independently of DTD");
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
  return {
      .registers = machine.cpu().registers(), .progress = machine.cpu().progress(), .ticks = ticks};
}

void native_service(const unsigned channel, const bool long_vector) {
  OwnDspMachine machine(native_clocks);
  prepare(machine, channel, long_vector);
  require(!tick(machine, 1) && !machine.bus().interrupt_request(0),
          "Actual source capture alone cannot signal DMA completion");
  const auto held = machine.checkpoint();
  std::optional<Result> completed;
  for (unsigned replay = 0; replay < 4; ++replay) {
    OwnDspMachine fresh(native_clocks);
    auto &owner = replay == 3 ? fresh : machine;
    owner.restore(held);
    const auto result = completed_service(owner, channel, long_vector);
    if (completed) { require(result == *completed, "Same/fresh owners replay all native state"); }
    completed = result;
  }
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
  OwnDspMachine machine(native_clocks);
  prepare(machine, 1, false, false);
  static_cast<void>(tick(machine, 1));
  require(tick(machine, 2).has_value(), "Actual DMA1 destination store establishes the IRQ");
  constexpr unsigned vector = first_vector + 2;
  const DspInterruptRequest request{.vector = vector};
  const auto saved = machine.checkpoint();
  const auto foreground = machine.cpu().registers();
  admit(machine);
  const auto before = machine.cpu().registers();
  const auto count = machine.cpu().progress();
  auto progress = machine.begin_fetch();
  if (!progress.pending) { throw Fault("Missing native first-vector fetch"); }
  progress = machine.advance(progress.pending->ticket);
  if (!progress.pending) { throw Fault("Missing native second-vector fetch"); }
  require(progress.pending->effect.address == vector + 1,
          "Missing second word remains an actual native program fetch");
  progress = machine.advance(progress.pending->ticket);
  require(progress.fault.kind == dsp56300::FaultKind::memory_read &&
              progress.fault.space == dsp56300::Space::program &&
              progress.fault.address == vector + 1 && machine.cpu().registers() == before &&
              machine.cpu().progress() == count && machine.interrupt_pending() &&
              machine.bus().interrupt_request(0) == request && machine.io().failure() &&
              !machine.cpu().instruction_outcome(),
          "Refused second vector fetch preserves the source and retires no instruction");
  machine.restore(saved);
  machine.bus().write({.address = vector + 1, .value = nop});
  vector_service(machine, request);
  require(
      machine.cpu().registers() == foreground,
      "Trusted recovery needs two real vector reads before returning to the original foreground");
}

void suspended_acknowledgement() {
  OwnDspMachine machine(native_clocks);
  prepare(machine, 1, false);
  static_cast<void>(tick(machine, 1));
  require(tick(machine, 2).has_value(), "Actual DMA1 completion precedes suspended entry");
  admit(machine);
  auto progress = machine.begin_fetch();
  if (!progress.pending) { throw Fault("Missing native first-vector fetch"); }
  progress = machine.advance(progress.pending->ticket);
  constexpr unsigned second = first_vector + 3;
  if (!progress.pending) { throw Fault("Missing native acknowledgement boundary"); }
  require(progress.pending->effect.kind == dsp56300::EffectKind::fetch_extension &&
              progress.pending->effect.address == second && machine.interrupt_pending(),
          "The second actual vector read remains pending before acknowledgement");
  const auto saved = machine.checkpoint();
  const auto stale = progress.pending->ticket;
  write(machine.bus(), control0 - 4, word_copy & ~enable);
  static_cast<void>(rejects([&] { static_cast<void>(machine.advance(stale)); }));
  require(machine.interrupt_pending(), "Refused source-hold validation cannot consume ownership");
  write(machine.bus(), control0 - 4, dma_control & ~enable);
  const DspInterruptRequest request{.vector = first_vector + 2};
  require(
      machine.bus().interrupt_request(0) == request,
      "Refused acknowledgement retains the genuine DMA event behind its disabled delivery gate");
  OwnDspMachine fresh(native_clocks);
  fresh.restore(saved);
  const auto pending = fresh.cpu().pending_instruction();
  if (!pending) { throw Fault("Trusted restore lost the actual vector read"); }
  bool rejected = false;
  try {
    static_cast<void>(fresh.advance(stale));
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected && fresh.bus().interrupt_request(0) == request && fresh.interrupt_pending(),
          "A foreign pre-restore ticket cannot acknowledge the recovered DMA event");
  progress = fresh.advance(pending->ticket);
  require(!fresh.interrupt_pending() && !fresh.bus().interrupt_request(0),
          "Fresh actual second-vector read consumes the recovered event once");
  drain(fresh, progress);
}
}

void native_checks() {
  for (unsigned channel = 0; channel < channels; ++channel) {
    for (const bool long_vector : {false, true}) { native_service(channel, long_vector); }
  }
  bus_arbitration();
  missing_vector_and_recovery();
  suspended_acknowledgement();
  std::cout << "PASS DMA_NATIVE_IRQ contexts=12 replays=48 missing-vector=1 suspended-ack=1\n";
}
}
