// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "esai_interrupt_fixture.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

namespace blofeld::test::esai_irq {
namespace {
constexpr unsigned boot = 0xff0000, handler = 0x300, ordinary_sr = 0xc00000, ipl_mask = 0x300;
constexpr unsigned absolute_jsr = 0x0bf080, secondary_priority = 0x400;
constexpr unsigned move_sr = 0x05f439, rti = 4, instruction_limit = 64;
constexpr std::uint64_t reset_hold = 90, reset_recovery = 90;
constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                   .timebase_hz = 540000000,
                                   .switch_timing = DspClockSwitch::reference_cycle_boundary};
constexpr auto timing =
    OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts;

struct Scenario {
  bool second;
  unsigned mode;
  bool reference;
};

struct CpuView {
  dsp56300::Registers cpu;
  dsp56300::Progress progress;
  std::uint64_t ticks;
  std::optional<std::uint64_t> dma;
  bool operator==(const CpuView &) const = default;
};

struct NativeFixture {
  NativeFixture() {
    // Authored MOVEC SR and foreground NOPs; reset itself executes no instruction.
    const std::array<std::uint32_t, 8> words{move_sr, ordinary_sr};
    machine.bus().install_rom({.start = boot, .words = words});
    machine.begin_hardware_reset(
        {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
    require(machine.advance_hardware_reset_ticks(reset_hold) == reset_hold, "Actual reset hold");
    machine.release_hardware_reset();
    require(machine.advance_hardware_reset_ticks(reset_recovery) == reset_recovery,
            "Actual reset recovery");
    step();
    require(machine.cpu().registers().pc == boot + 2 && machine.cpu().registers().sr == ordinary_sr,
            "Actual MOVEC unmasks the foreground before any transmit request exists");
  }

  [[nodiscard]] CpuView view() {
    return {.cpu = machine.cpu().registers(),
            .progress = machine.cpu().progress(),
            .ticks = machine.clock().elapsed_ticks(),
            .dma = machine.bus().last_dma_cycle()};
  }

  void start_timeline() { timeline = std::make_unique<OwnDspTimeline>(machine, timing, 0); }

  void step() {
    const auto before = machine.cpu().progress().instructions;
    if (timeline) {
      for (unsigned cycle = 0;
           cycle < instruction_limit && machine.cpu().progress().instructions == before;
           ++cycle) {
        const auto last = machine.bus().last_dma_cycle();
        const auto result = timeline->advance_one(timeline->next_deadline(),
                                                  OwnDspBoundary::through,
                                                  {.cycle = last ? *last + 1 : 0,
                                                   .known_requests = 15,
                                                   .timing = DspDmaTiming::reference_sampled_cycles,
                                                   .grant = DspDmaGrant::available});
        require(result &&
                    (!result->native || result->native->fault.kind == dsp56300::FaultKind::none),
                "Automatic native ESAI instruction remains healthy");
      }
    } else {
      auto progress = machine.begin_fetch();
      for (unsigned effect = 0; effect < instruction_limit && progress.pending && !progress.retired;
           ++effect) {
        progress = machine.advance(progress.pending->ticket);
        require(progress.fault.kind == dsp56300::FaultKind::none,
                "Actual native ESAI effect remains healthy");
      }
      require(progress.retired, "Manual native ESAI instruction retires");
    }
    require(machine.cpu().progress().instructions == before + 1,
            "Exactly one actual native instruction retires");
  }

  OwnDspMachine machine{clocks};
  std::unique_ptr<OwnDspTimeline> timeline;
};

void install(NativeFixture &fixture, const Scenario input) {
  // FM MOVEP low-X/low-Y immediate and split-qq reads, plus literal RTI.
  constexpr unsigned primary_write = 0x07f420;
  constexpr unsigned secondary_write = 0x07b480;
  constexpr unsigned primary_status = 0x0444d3;
  constexpr unsigned secondary_status = 0x044433;
  auto &bus = fixture.machine.bus();
  unsigned address = handler;
  if (input.mode == 1) {
    bus.write({.address = address++, .value = input.second ? secondary_status : primary_status});
  }
  const auto write = input.second ? secondary_write : primary_write;
  for (const auto value : std::array{write, next_first, write + 5, next_second, rti}) {
    bus.write({.address = address++, .value = value});
  }
  const auto vector = expected(input.second, input.mode, {.even = true, .error = input.mode == 1});
  if (!vector) { throw Fault("Actual ESAI scenario lacks its independently expected vector"); }
  bus.write({.address = vector.value(), .value = absolute_jsr});
  bus.write({.address = vector.value() + 1, .value = handler});
  setup(bus, input.second, input.mode);
  bus.write({.space = DspSpace::x,
             .address = priority_register,
             .value = input.second ? secondary_priority : 1U});
  require(word(bus, input.second) == std::array{first_data, second_data},
          "Actual serial priming words");
  if (input.mode == 1) { static_cast<void>(word(bus, input.second)); }
  if (!input.reference) { fixture.start_timeline(); }
}

void reference_entry(OwnDspMachine &machine, const DspInterruptRequest request) {
  const auto before = machine.cpu().progress();
  const auto foreground = machine.cpu().registers().pc;
  require(machine.bus().inspect_program(foreground) == 0 &&
              machine.admit_interrupt({.address = foreground, .opcode = 0}) &&
              machine.interrupt_pending(),
          "Real unmasked ESAI source admits the known authored foreground");
  auto progress = machine.begin_fetch();
  unsigned vectors = 0;
  unsigned acknowledgements = 0;
  for (unsigned effect = 0; effect < instruction_limit && !progress.retired; ++effect) {
    if (!progress.pending) { throw Fault("ESAI entry lost its genuine native ticket"); }
    const auto pending = progress.pending.value();
    const bool vector = pending.effect.kind == dsp56300::EffectKind::fetch_opcode ||
                        pending.effect.kind == dsp56300::EffectKind::fetch_extension;
    require(machine.cpu().progress() == before && machine.bus().interrupt_request(0) == request,
            "No entry retirement or level-source service precedes actual native effects");
    if (vector) {
      require(pending.effect.space == dsp56300::Space::program &&
                  pending.effect.address == request.vector + vectors && vectors < 2,
              "Both actual vector words retain their independently expected addresses");
      require(machine.interrupt_pending(),
              "Vector ownership remains pending until the second real read");
    }
    progress = machine.advance(pending.ticket);
    require(progress.fault.kind == dsp56300::FaultKind::none,
            "Native vector/PCU/stack effects remain healthy");
    if (vector) {
      ++vectors;
      if (!machine.interrupt_pending()) { ++acknowledgements; }
      require(machine.interrupt_pending() == (vectors == 1),
              "Acknowledgement follows only the second actual vector read");
      if (vectors == 1) {
        const auto registers = machine.cpu().registers();
        static_cast<void>(rejects([&] { static_cast<void>(machine.advance(pending.ticket)); }));
        require(machine.interrupt_pending() && machine.cpu().registers() == registers,
                "Stale ticket cannot consume the second vector or mutate architectural state");
      }
    }
  }
  require(progress.retired && vectors == 2 && acknowledgements == 1 && !machine.interrupt_pending(),
          "Actual JSR entry retires after two vector reads and one acknowledgement");
}

void service(NativeFixture &fixture, const Scenario input) {
  auto &machine = fixture.machine;
  const auto before = fixture.view();
  const auto vector = expected(input.second, input.mode, {.even = true, .error = input.mode == 1});
  if (!vector) { throw Fault("Expected ESAI request is missing"); }
  const DspInterruptRequest request{.vector = vector.value(), .level = 0};
  require(machine.bus().interrupt_request(0) == request, "Actual peripheral supplies the vector");
  if (input.reference) {
    reference_entry(machine, request);
  } else {
    fixture.step();
  }
  const auto entry = machine.cpu().registers();
  const auto frame = (std::uint64_t{boot + 2} << word_bits) | ordinary_sr;
  require(entry.pc == handler && entry.sp == 1 && entry.stack[1] == frame &&
              (entry.sr & ipl_mask) == 0x100 && machine.bus().interrupt_request(0) == request &&
              !machine.interrupt_pending(),
          "Real JSR stacks foreground PC/SR; vector acknowledgement retains the level request");
  if (input.mode == 1) {
    fixture.step();
    require((machine.cpu().registers().x & status_underrun) != 0,
            "ISR reads actual underrun status before refill");
  }
  fixture.step();
  require(machine.bus().interrupt_request(0) == request,
          "First native MOVEP leaves the other empty lane pending");
  fixture.step();
  require(!machine.bus().interrupt_request(0), "Second native MOVEP services the actual level IRQ");
  fixture.step();
  require(machine.cpu().registers().pc == boot + 2 && machine.cpu().registers().sp == 0 &&
              machine.cpu().registers().sr == ordinary_sr &&
              machine.cpu().progress().instructions - before.progress.instructions ==
                  (input.mode == 1 ? 5U : 4U),
          "Actual ISR/RTI restores independent PC/SR/SP and instruction count");
  require(word(machine.bus(), input.second) == std::array{next_first, next_second},
          "Actual ISR writes become the next independent24-bit serial words");
}

void native_case(const Scenario input) {
  NativeFixture fixture;
  install(fixture, input);
  service(fixture, input);
}

void native_unknown_clock(const bool second) {
  NativeFixture fixture;
  setup(fixture.machine.bus(), second, 4, false);
  fixture.machine.bus().write({.space = DspSpace::x,
                               .address = priority_register,
                               .value = second ? secondary_priority : 1U});
  fixture.start_timeline();
  const auto error = rejects([&] { fixture.step(); });
  require(error.contains(second ? "Y:0xffff93" : "X:0xffffb3") &&
              error.contains("pc=" + hex(boot + 2, 6)) && error.contains("omr=0x000305") &&
              error.contains("ticks=") && fixture.timeline->faulted(),
          "Unknown eligible clock reports its actual status address, PC/OMR/time and quarantines "
          "the timeline");
  const auto failed = fixture.view();
  static_cast<void>(rejects([&] { fixture.step(); }));
  require(fixture.view() == failed,
          "Rejected owner cannot consume any vector, CPU state or clock period");
  NativeFixture known;
  setup(known.machine.bus(), second, 4, false);
  known.machine.bus().write({.space = DspSpace::x,
                             .address = priority_register,
                             .value = second ? secondary_priority : 1U});
  known.start_timeline();
  known.machine.bus().observe_esai_transmit(instance(second), {.clock = true, .frame_sync = false});
  known.step();
  require(known.machine.cpu().registers().pc == boot + 3 &&
              !known.machine.bus().interrupt_request(0),
          "A fresh owner with an actual clock baseline executes the unrequested foreground NOP");
}
}

void native_checks() {
  unsigned contexts = 0;
  unsigned clock_refusals = 0;
  for (const bool second : {false, true}) {
    native_unknown_clock(second);
    ++clock_refusals;
    for (const unsigned mode : {1U, 2U, 4U}) {
      for (const bool reference : {false, true}) {
        native_case({.second = second, .mode = mode, .reference = reference});
        ++contexts;
      }
    }
  }
  require(contexts == 12 && clock_refusals == 2,
          "Every independent native source/entry context completes");
  std::cout << "PASS ESAI_NATIVE_IRQ contexts=" << contexts << " clock-refusals=" << clock_refusals
            << '\n';
}
}
