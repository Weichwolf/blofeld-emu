// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_fetch_schedule_fixture.hpp"

#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace blofeld::test::fetch_events {
namespace {
McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  // Authored HCS12 NOP stream. The real reset vector and five reset bus cycles
  // establish the MCU epoch; no injected PC, registers or fabricated periods.
  constexpr unsigned nop = 0xa7;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unprotected = 0xff;
  constexpr unsigned unsecured = 0xfe;
  constexpr unsigned entry_hi = 0x80;
  bytes.fill(nop);
  bytes[protection] = unprotected;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry_hi;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}
}

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

McuDspRuntimeInputs inputs() {
  return {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
          .maximum_cycles_per_period = 16,
          .writes = McuWriteOrder::before_mcu,
          .external_writes = McuWriteOrder::before_mcu,
          .wait_dsp_first = true,
          .reset_dsp_first = true,
          .initial_reset = OwnDspResetInputs{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3}};
}

DspDmaInputs cycle_inputs() {
  return {.known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

DspRuntimeEvents Source::events() {
  return {.identity = identity,
          .next_ticks = [this]() -> std::optional<std::uint64_t> {
            if (on_query) { on_query(); }
            return cursor < ticks.size() ? std::optional{ticks[cursor]} : std::nullopt;
          },
          .observe =
              [this](const std::uint64_t at) {
                require(cursor < ticks.size() && ticks[cursor] == at,
                        "Actual external cursor/time differs");
                if (on_observe) { on_observe(at); }
                log.push_back("E" + std::to_string(at));
                ++cursor;
              },
          .order = order,
          .maximum_events_per_period = budget};
}

Fixture::Fixture(const DspPeerOrder order,
                 std::vector<std::uint64_t> ticks,
                 const std::span<const std::uint32_t> program,
                 const bool start)
    : mcu(image(),
          BLOFELD_DEFAULT_SLA,
          McuRunContract{.clock = {.oscillator_hz = extal,
                                   .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                         .memory_timing = McuRunMemory::unstretched_internal}),
      dsp({.extal_hz = extal,
           .timebase_hz = timebase,
           .switch_timing = DspClockSwitch::reference_cycle_boundary}) {
  mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  std::array<std::uint32_t, nops> rom{};
  require(program.size() <= rom.size(), "Authored DSP fixture exceeds its ROM");
  std::ranges::copy(program, rom.begin());
  dsp.bus().install_rom({.space = DspSpace::p, .start = boot, .words = rom});
  source.order = order;
  source.ticks = std::move(ticks);
  source.on_observe = [this](std::uint64_t) {
    dsp.bus().timers().set_tio_drive(0, (source.cursor & 1U) != 0);
  };
  if (start) { connect(inputs()); }
}

void Fixture::connect(const McuDspRuntimeInputs contract) {
  runtime = std::make_unique<McuDspRuntime>(contract, mcu, dsp, source.events());
  require(runtime->end_ticks() == initial_ticks && dsp.clock().elapsed_ticks() == initial_ticks,
          "Real MCU reset did not establish five 90tick bus periods");
  runtime->release_dsp_reset();
  bind();
}

void Fixture::bind() {
  mcu.set_cycle_observer([this](const Hc12BusCycle &cycle) {
    runtime->begin_cycle(cycle, false, provide_inputs, [this](const DspRuntimeCycle &event) {
      source.log.push_back("D" + std::to_string(event.completion_ticks));
      if (on_cycle) { on_cycle(event); }
    });
  });
}

void Fixture::step() {
  require(mcu.cpu().step().instruction_executed, "Actual MCU NOP did not retire");
  runtime->end_instruction();
}

void Fixture::run(const unsigned count) {
  for (unsigned i = 0; i < count; ++i) { step(); }
}

}
