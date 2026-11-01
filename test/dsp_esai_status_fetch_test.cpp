// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using blofeld::test::esai::rejects;
using blofeld::test::esai::require;
constexpr unsigned boot = 0xff0000, status_offset = 19, control_offset = 21;
constexpr unsigned primary_base = 0xffffa0, secondary_base = 0xffff80;
constexpr unsigned value = 0x123456, next_word = 0xa5c39f;
constexpr unsigned native_limit = 32, expected_status = 0x1e000, underrun = 0x4000;
constexpr unsigned word_bits = 24;
constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                   .timebase_hz = 540000000,
                                   .switch_timing = DspClockSwitch::reference_cycle_boundary};
constexpr auto timing =
    OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts;

DspStore store(const bool second, const unsigned offset, const unsigned word) {
  return {.space = second ? DspSpace::y : DspSpace::x,
          .address = (second ? secondary_base : primary_base) + offset,
          .value = word};
}

void setup(OwnDspMachine &machine, const bool second) {
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  constexpr std::uint64_t hold = 90;
  require(machine.advance_hardware_reset_ticks(hold) == hold, "Actual native reset hold");
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset_ticks(hold) == hold, "Actual native reset recovery");
  auto &bus = machine.bus();
  bus.observe_esai_transmit(second ? DspEsaiInstance::secondary : DspEsaiInstance::primary,
                            {.clock = true, .frame_sync = false});
  constexpr unsigned clock_offset = 22;
  constexpr unsigned mux = 0x818;
  constexpr unsigned control = 0xf801;
  constexpr unsigned port_control = 31;
  constexpr unsigned port_direction = 30;
  for (const auto write : std::array{store(second, clock_offset, 0),
                                     store(second, 0, value),
                                     store(second, control_offset, control),
                                     store(second, port_control, mux),
                                     store(second, port_direction, mux)}) {
    bus.write(write);
  }
  // Two genuine external24-bit words, the second without a refill, set TUE.
  for (unsigned frame = 0; frame < 2; ++frame) {
    for (unsigned bit = 0; bit < word_bits; ++bit) {
      const auto instance = second ? DspEsaiInstance::secondary : DspEsaiInstance::primary;
      bus.observe_esai_transmit(instance, {.clock = false, .frame_sync = bit == 0});
      bus.observe_esai_transmit(instance, {.clock = true, .frame_sync = bit == 0});
    }
  }
}

void enable_dma(DspBus &bus) {
  constexpr unsigned control = 0xffffec;
  constexpr unsigned request_copy = 0x880244;
  for (const auto address : {control + 3, control + 2, control + 1}) {
    bus.write({.space = DspSpace::x, .address = address});
  }
  bus.write({.space = DspSpace::x, .address = control, .value = request_copy});
  require(bus.has_active_dma(), "The negative control actually enables DMA");
}

void dma_guard(OwnDspMachine &machine, OwnDspFetch &fetch, const bool second) {
  fetch.issue();
  fetch.capture();
  const auto admission = fetch.begin();
  const auto pending = machine.cpu().pending_instruction();
  if (admission.fault.kind != dsp56300::FaultKind::none || !pending ||
      pending->effect.kind != dsp56300::EffectKind::read) {
    throw Fault("The negative control requires the genuine unexecuted SAISR read");
  }
  fetch.issue();
  const auto view = fetch.view();
  require(view.pending_address.has_value(), "The guard covers an actual concurrent F2 read");
  enable_dma(machine.bus());
  const auto registers = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto elapsed = machine.clock().elapsed_ticks();
  const auto dma_cycle = machine.bus().last_dma_cycle();
  const DspDmaInputs inputs{.cycle = 0,
                            .timing = DspDmaTiming::reference_sampled_cycles,
                            .grant = DspDmaGrant::available};
  const auto error = rejects([&] {
    static_cast<void>(
        fetch.advance_dma_capture_native(inputs, OwnDspNativeCycleModel::isolated_effect));
  });
  const auto held = machine.cpu().pending_instruction();
  require(error.contains(second ? "native=read Y:0xffff93" : "native=read X:0xffffb3") &&
              error.contains("pc=0xff0000") && machine.cpu().registers() == registers &&
              machine.cpu().progress() == progress && held && held->effect == pending->effect &&
              fetch.view() == view && machine.clock().elapsed_ticks() == elapsed &&
              machine.bus().last_dma_cycle() == dma_cycle && !machine.io().failure(),
          "Active DMA refuses before status side effects, F2 or clock/DMA progress");
}

void status_case(const bool second, const bool active_dma) {
  OwnDspMachine machine(clocks);
  constexpr unsigned primary_read = 0x0444d3;
  constexpr unsigned secondary_read = 0x044433;
  std::array<std::uint32_t, native_limit> rom{};
  rom[0] = second ? secondary_read : primary_read;
  machine.bus().install_rom({.start = boot, .words = rom});
  setup(machine, second);
  OwnDspFetch fetch(machine);
  if (active_dma) {
    dma_guard(machine, fetch, second);
    return;
  }
  bool overlap = false;
  for (unsigned cycle = 0; cycle < native_limit; ++cycle) {
    const DspDmaInputs inputs{.cycle = cycle,
                              .timing = DspDmaTiming::reference_sampled_cycles,
                              .grant = DspDmaGrant::available};
    const auto result = fetch.advance_dma_automatic(inputs, timing);
    require(!result.native || result.native->fault.kind == dsp56300::FaultKind::none,
            "Native SAISR accesses the actual known peripheral");
    overlap = overlap || (result.instruction_started && result.captured && result.native);
    if (machine.cpu().progress().instructions == 1) {
      require(overlap && machine.cpu().registers().x == expected_status &&
                  machine.cpu().registers().pc == boot + 1 &&
                  machine.cpu().progress().estimated_cycles == 1,
              "Concurrent F2/native SAISR returns the independent TFS/TDE/TEDE/TUE word: x=" +
                  hex(machine.cpu().registers().x, 12) +
                  " overlap=" + (overlap ? "true" : "false"));
      machine.bus().write(store(second, 0, next_word));
      const auto status =
          machine.bus().read({.space = second ? DspSpace::y : DspSpace::x,
                              .address = (second ? secondary_base : primary_base) + status_offset});
      require((status & underrun) == 0,
              "The real native status read arms TUE clearing on the following genuine refill");
      return;
    }
  }
  throw Fault("Native SAISR test exceeded its bounded period count");
}

void unread_control(const bool second) {
  OwnDspMachine machine(clocks);
  setup(machine, second);
  machine.bus().write(store(second, 0, next_word));
  const auto status =
      machine.bus().read({.space = second ? DspSpace::y : DspSpace::x,
                          .address = (second ? secondary_base : primary_base) + status_offset});
  require((status & underrun) != 0,
          "Refilling without a native or host status read cannot clear the actual underrun");
}
}

int main() {
  try {
    for (const bool second : {false, true}) {
      status_case(second, false);
      status_case(second, true);
      unread_control(second);
    }
    std::cout << "Native ESAI status/F2 samples, underrun side effects and DMA guards passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
