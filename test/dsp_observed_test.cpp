// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <tuple>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
namespace native = test::dma_test;
constexpr unsigned hcsr = 0xffff91, hrx = 0xffff94, htx = 0xffff93;
constexpr unsigned config = 0x1029, idle = 0x800000;
constexpr unsigned data = 0x123456, sentinel = 0x654321, word_bits = 24;
constexpr unsigned immediate_x0 = 0x44f400, read_x0 = 0x44f000, immediate_sr = 0x05f439;
constexpr unsigned vector = 0x10, priority_register = 0xffffff;
constexpr unsigned bound = 32;

auto state(const OwnDspMachine &machine) {
  return std::tuple{machine.cpu().registers(),
                    machine.cpu().progress(),
                    machine.cpu().fault(),
                    machine.clock().elapsed_ticks(),
                    machine.interrupt_pending()};
}

void clocked_step(OwnDspMachine &machine) {
  const auto before = machine.cpu().progress().estimated_cycles;
  native::step_native(machine);
  const auto cycles = machine.cpu().progress().estimated_cycles - before;
  for (std::uint64_t cycle = 0; cycle < cycles; ++cycle) {
    static_cast<void>(machine.advance_reference_cycle());
  }
}

void setup_fifo(OwnDspMachine &machine) {
  native::NativeProgram program;
  program.word(immediate_x0);
  program.word(sentinel);
  program.write({.space = DspSpace::x, .address = hcsr, .value = config});
  program.write({.space = DspSpace::x, .address = htx, .value = idle});
  program.word(read_x0);
  program.word(hrx);
  native::reset_native(machine, program);
  machine.bus().shi().observe_spi({.mosi = false});
  for (unsigned instruction = 0; instruction < 3; ++instruction) { clocked_step(machine); }
  auto &shi = machine.bus().shi();
  shi.observe_spi({.selected = true, .mosi = false});
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    const bool level = ((data >> (word_bits - 1 - bit)) & 1U) != 0;
    shi.observe_spi({.selected = true, .clock = true, .mosi = level});
    shi.observe_spi({.selected = true, .clock = false, .mosi = level});
  }
  shi.observe_spi({.mosi = false});
  require(!shi.interrupt_vectors().empty(), "Actual SPI edges leave a pending SHI receive word");
}

struct Reader {
  OwnDspMachine &machine;
  unsigned calls = 0, fifo_reads = 0;
  bool fail = false;

  static void completed(void *context, const DspIoReadAccess &access) {
    auto &self = *static_cast<Reader *>(context);
    ++self.calls;
    if (access.address.space != DspSpace::x || access.address.address != hrx) { return; }
    ++self.fifo_reads;
    require(access.value == data && access.pc == native::boot + 6 &&
                self.machine.bus().shi().interrupt_vectors().empty() &&
                self.machine.cpu().progress().instructions == 3 &&
                (self.machine.cpu().registers().x & dsp56300::word_mask) == sentinel,
            "Read observer sees the consumed physical FIFO value before CPU publication");
    const auto before = state(self.machine);
    static_cast<void>(rejects([&] { static_cast<void>(self.machine.run_unclocked()); }));
    static_cast<void>(rejects([&] { static_cast<void>(self.machine.checkpoint()); }));
    require(state(self.machine) == before, "Read callback cannot reenter or capture its owner");
    if (self.fail) { throw Fault("Authored observer rejection after destructive FIFO read"); }
  }

  [[nodiscard]] DspIoReadObserver borrowed() { return {.context = this, .completed = completed}; }
};

dsp56300::InstructionProgress finish(OwnDspMachine &machine,
                                     dsp56300::InstructionProgress progress,
                                     const DspIoReadObserver observer) {
  for (unsigned effect = 0; effect < bound && progress.pending && !progress.retired &&
                            progress.fault.kind == dsp56300::FaultKind::none;
       ++effect) {
    progress = machine.advance_observed(progress.pending->ticket, observer);
  }
  return progress;
}

void fifo_observation(const bool fail) {
  OwnDspMachine machine(native::native_clocks);
  setup_fifo(machine);
  const auto healthy = machine.checkpoint();
  const auto ticks = machine.clock().elapsed_ticks();
  Reader reader{.machine = machine, .fail = fail};
  auto progress = machine.begin_fetch(dsp56300::InstructionPublication::retirement);
  if (!progress.pending) { throw Fault("Actual FIFO instruction lacks its staged fetch"); }
  const auto first = progress.pending.value().ticket;
  progress = machine.advance_observed(first, reader.borrowed());
  const auto after_fetch = state(machine);
  const auto calls = reader.calls;
  static_cast<void>(
      rejects([&] { static_cast<void>(machine.advance_observed(first, reader.borrowed())); }));
  require(reader.calls == calls && state(machine) == after_fetch,
          "Stale tickets neither replay a callback nor mutate the native state");
  progress = finish(machine, progress, reader.borrowed());
  require(reader.fifo_reads == 1 && machine.bus().shi().interrupt_vectors().empty() &&
              machine.clock().elapsed_ticks() == ticks,
          "One actual read consumes the FIFO without advancing reference time");
  if (fail) {
    const auto &failure = machine.io().failure();
    // Staged ordinary reads report memory_read, including peripheral addresses.
    require(
        progress.fault.kind == dsp56300::FaultKind::memory_read && failure &&
            failure->operation == OwnDspIo::Operation::read && failure->address == hrx &&
            failure->pc == native::boot + 6 && failure->cause &&
            machine.cpu().progress().instructions == 3 &&
            (machine.cpu().registers().x & dsp56300::word_mask) == sentinel,
        "Rejected observation retains consumption and contextual fault without destination write");
    const auto diagnostic = machine.checkpoint();
    const auto failed = state(machine);
    OwnDspMachine fresh(native::native_clocks);
    fresh.restore(diagnostic);
    require(state(fresh) == failed && fresh.bus().shi().interrupt_vectors().empty(),
            "Fresh diagnostic restore retains the actual fault and destructive read");
  } else {
    require(progress.retired && progress.fault.kind == dsp56300::FaultKind::none &&
                machine.cpu().progress().instructions == 4 &&
                (machine.cpu().registers().x & dsp56300::word_mask) == data,
            "Accepted observation publishes the actual read value and one retirement");
  }
  const auto observed = reader.calls;
  machine.restore(healthy);
  native::step_native(machine);
  require(reader.calls == observed && machine.bus().shi().interrupt_vectors().empty() &&
              (machine.cpu().registers().x & dsp56300::word_mask) == data,
          "Healthy restoration has no expired observer and consumes its real saved FIFO word");
}

void vector_and_owner_guards() {
  OwnDspMachine machine(native::native_clocks);
  native::NativeProgram program;
  program.word(immediate_sr);
  program.word(0);
  program.word(0);
  program.word(0);
  native::reset_native(machine, program);
  native::step_native(machine);
  auto pending = machine.begin_fetch(dsp56300::InstructionPublication::retirement);
  if (!pending.pending) { throw Fault("Foreground NOP lacks its real ticket"); }
  const auto stale = pending.pending.value().ticket;
  pending = finish(machine, pending, {});
  require(pending.retired, "The real foreground NOP retires");
  {
    const OwnDspFetch fetch(machine);
    const auto before = state(machine);
    const auto error = rejects([&] { static_cast<void>(machine.advance_observed(stale, {})); });
    require(error.contains("observed effect requires an inactive unclocked owner") &&
                state(machine) == before,
            "Captured fetch refuses observation before ticket admission");
  }
  {
    const OwnDspTimeline timeline(machine, OwnDspAutomaticTiming::one_effect_per_cycle, 0);
    const auto before = state(machine);
    const auto error = rejects([&] { static_cast<void>(machine.advance_observed(stale, {})); });
    require(error.contains("clocked timeline owns") && state(machine) == before,
            "Clocked owner refuses observation before ticket admission");
  }
  machine.bus().write({.address = vector, .value = 0});
  machine.bus().write({.address = vector + 1, .value = 0});
  machine.bus().observe_irqa(true);
  machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  machine.bus().observe_irqa(false);
  require(machine.admit_interrupt({.address = native::boot + 3, .opcode = 0}),
          "Actual unmasked foreground admits an IRQA vector pair");
  const auto healthy = machine.checkpoint();
  unsigned reads = 0;
  const DspIoReadObserver observer{
      .context = &reads, .completed = [](void *context, const DspIoReadAccess &access) {
        auto &count = *static_cast<unsigned *>(context);
        require(access.address.space == DspSpace::p && access.address.address == vector + count &&
                    access.value == 0,
                "Only actual ordered vector reads reach the observer");
        if (++count == 2) { throw Fault("Refused observation of the second actual vector word"); }
      }};
  const auto fault = finish(machine, machine.begin_fetch(), observer);
  require(reads == 2 && fault.fault.kind == dsp56300::FaultKind::memory_read &&
              fault.fault.address == vector + 1 && machine.interrupt_pending() &&
              machine.bus().interrupt_request(0),
          "An observer rejection cannot acknowledge the peripheral's pending vector pair");
  machine.restore(healthy);
  native::step_native(machine);
  native::step_native(machine);
  require(
      reads == 2 && !machine.interrupt_pending() && !machine.bus().interrupt_request(0) &&
          machine.cpu().registers().pc == native::boot + 3,
      "Restored vector entry completes through real unobserved reads without a retained callback");
}
}

int main() {
  try {
    fifo_observation(false);
    fifo_observation(true);
    vector_and_owner_guards();
    std::cout << "Observed DSP effects: actual FIFO consumption, callback faults/lifetime, stale "
                 "tickets, vector acknowledgement and fetch/timeline ownership passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
