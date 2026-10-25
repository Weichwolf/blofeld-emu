// SPDX-License-Identifier: GPL-3.0-only
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_dma_interrupt_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>

namespace blofeld::test::dma_irq {
namespace {
using checkpoint::assemble;
using checkpoint::ram;
using checkpoint::until;
using dma_test::control0;
using dma_test::counter0;
using dma_test::destination;
using dma_test::destination0;
using dma_test::enable;
using dma_test::grant;
using dma_test::ring_start;
using dma_test::source0;
using dma_test::status;
using dma_test::timing;
using dma_test::word_copy;
constexpr unsigned ordinary_status = 0xc00000;
constexpr unsigned handler = 0x300;
constexpr unsigned absolute_jsr = 0x0bf080;
constexpr unsigned payload = 0x123456;
constexpr unsigned sentinel = 0x654321;
constexpr unsigned foreground_value = 0x77;
constexpr unsigned dma_control = word_copy | interrupt_enable;

void write(DspBus &bus, const unsigned address, const unsigned value) {
  bus.write({.space = DspSpace::x, .address = address, .value = value});
}

unsigned prepare(DspMachine &machine,
                 const unsigned channel,
                 const bool long_vector,
                 const bool complete_vector = true) {
  DspProgram program;
  program.immediate("sr", ordinary_status);
  const auto put = [&](const unsigned address, const unsigned value) {
    program.emit("movep #>" + number(value) + ",x:<<" + number(address));
  };
  put(iprc, 1U << (priority_shift + (2 * channel)));
  put(source0 - (4 * channel), ring_start);
  put(destination0 - (4 * channel), destination);
  put(counter0 - (4 * channel), 0);
  put(control0 - (4 * channel), dma_control);
  const auto foreground = program.pc();
  program.immediate("r7", foreground_value);
  program.install(machine.bus());
  write(machine.bus(), ring_start, payload);
  machine.bus().write({.space = DspSpace::y, .address = destination, .value = sentinel});
  const auto vector = first_vector + (2 * channel);
  if (long_vector) {
    constexpr std::array call{absolute_jsr, handler};
    ram(machine.bus(), vector, call);
    ram(machine.bus(), handler, assemble("nop"));
    ram(machine.bus(), handler + 1, assemble("rti"));
  } else {
    ram(machine.bus(), vector, assemble("nop"));
    if (complete_vector) { ram(machine.bus(), vector + 1, assemble("nop")); }
  }
  static_cast<void>(
      machine.advance_dma({.cycle = 0, .known_requests = 1, .timing = timing, .grant = grant}));
  until(machine, foreground);
  return foreground;
}

std::optional<DspDmaWord> tick(DspMachine &machine, const std::uint64_t cycle) {
  return machine.advance_dma(
      {.cycle = cycle, .known_requests = 1, .requests = 1, .timing = timing, .grant = grant});
}

void native_service(const unsigned channel, const bool long_vector) {
  DspMachine machine;
  const auto foreground = prepare(machine, channel, long_vector);
  const auto before = machine.cpu().state();
  require(!tick(machine, 1) && !machine.bus().interrupt_request(0),
          "Native DMA enable plus an actual source read has not signalled completion");
  const auto held = machine.checkpoint();
  const DspInterruptRequest request{.vector = first_vector + (2 * channel)};
  std::optional<DspDebugState> completed;
  for (unsigned replay = 0; replay < 3; ++replay) {
    if (replay != 0) { machine.restore(held); }
    require(machine.bus().read({.space = DspSpace::y, .address = destination}) == sentinel,
            "A joint held-source restore retains the original destination and no completion");
    const auto word = tick(machine, 2);
    require(word && word->channel == channel && word->value == payload &&
                machine.bus().read({.space = DspSpace::y, .address = destination}) == payload &&
                machine.bus().interrupt_request(0) == request,
            "Only the actual RAM destination store requests this channel's native IRQ vector");
    const auto pending = machine.checkpoint();
    machine.restore(pending);
    require(machine.bus().interrupt_request(0) == request,
            "Joint restore retains pending DMA event and priority independently of DTD");
    machine.cpu().step();
    require(!machine.bus().interrupt_request(0) &&
                machine.bus().read({.space = DspSpace::x, .address = status}) == dma_test::all_done,
            "Real native second-vector fetch clears only the DMA request, preserving DTD");
    if (long_vector) {
      const auto entry = machine.cpu().state();
      require(entry.pc == handler && entry.sp == 1 && entry.stack_pc == foreground &&
                  entry.stack_sr == before.sr && entry.sr == (ordinary_status | 0x100U),
              "DMA IPL0 enters the genuine long vector and literal native frame at mask1");
      machine.cpu().step();
      machine.cpu().step();
    }
    require(machine.cpu().state() == before,
            "Fast NOP vectors or real long NOP/RTI preserve every foreground register");
    machine.cpu().step();
    require(machine.cpu().state().r[7] == foreground_value,
            "Foreground resumes only after the actual DMA vector service and return");
    const auto after = machine.cpu().debug_state();
    if (completed) { require(after == *completed, "Joint restores reproduce all native counters"); }
    completed = after;
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
  DspMachine machine;
  prepare(machine, 1, false, false);
  static_cast<void>(tick(machine, 1));
  require(tick(machine, 2).has_value(), "A real DMA1 destination store establishes the IRQ");
  constexpr unsigned dma1 = first_vector + 2;
  const auto saved = machine.checkpoint();
  const auto before = machine.cpu().state();
  const auto reason = rejects([&] { machine.cpu().step(); });
  require(reason.contains("00001b") && machine.cpu().state() == before &&
              machine.bus().interrupt_request(0) == DspInterruptRequest{.vector = dma1},
          "Missing second vector word rejects before CPU mutation or DMA acknowledgement");
  machine.restore(saved);
  ram(machine.bus(), dma1 + 1, assemble("nop"));
  machine.cpu().step();
  require(machine.cpu().state() == before && !machine.bus().interrupt_request(0),
          "Joint recovery retains the event until both genuine vector words can execute");
}

void suspended_acknowledgement() {
  DspMachine machine;
  prepare(machine, 1, false);
  static_cast<void>(tick(machine, 1));
  require(tick(machine, 2).has_value(), "Actual DMA1 completion precedes suspended native entry");
  const auto saved = machine.checkpoint();
  auto progress = machine.begin_reference_interrupt(DspReferenceExecution::access_boundaries);
  unsigned accesses = 0;
  while (progress.pending &&
         std::get_if<DspInterruptAcknowledge>(&progress.pending->effect) == nullptr) {
    require(++accesses <= 4, "Both vector reads must reach the bounded native acknowledgement");
    progress = machine.advance_reference_access(progress.pending->ticket);
  }
  require(progress.pending && !progress.retired && accesses == 2,
          "Two actual vector fetches precede the separate DMA acknowledge effect");
  write(machine.bus(), control0 - 4, word_copy & ~enable);
  static_cast<void>(rejects(
      [&] { static_cast<void>(machine.advance_reference_access(progress.pending->ticket)); }));
  machine.restore(saved);
  require(machine.bus().interrupt_request(0) == DspInterruptRequest{.vector = first_vector + 2},
          "A failed conservative source-hold contract cannot consume the saved DMA event");
  machine.cpu().step();
  require(!machine.bus().interrupt_request(0),
          "Fresh genuine service consumes the recovered event");
}
}

void native_checks() {
  for (unsigned channel = 0; channel < channels; ++channel) {
    for (const bool long_vector : {false, true}) { native_service(channel, long_vector); }
  }
  bus_arbitration();
  missing_vector_and_recovery();
  suspended_acknowledgement();
}
}
