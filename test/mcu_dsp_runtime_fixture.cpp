// SPDX-License-Identifier: GPL-3.0-only
#include "mcu_dsp_runtime_fixture.hpp"

#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
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
#include <span>

namespace blofeld::test::mcu_dsp {
namespace {
McuFlashImage image(const std::span<const std::uint8_t> program) {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  require(program.size() < protection, "Authored MCU program overlaps its NVM controls");
  bytes.fill(nop);
  std::ranges::copy(program, bytes.begin());
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_inputs(const Options &options) {
  return {.clock = {.oscillator_hz = options.mcu_hz,
                    .initial_oscillator_phase_ticks = options.mcu_phase,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}
}

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

OwnDspResetInputs reset_inputs() {
  return {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2};
}

McuDspRuntimeInputs runtime_inputs() {
  return {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
          .maximum_cycles_per_period = 16,
          .writes = McuWriteOrder::before_mcu,
          .external_writes = McuWriteOrder::before_mcu,
          .wait_dsp_first = true,
          .reset_dsp_first = true,
          .initial_reset = reset_inputs()};
}

DspDmaInputs cycle_inputs() {
  return {.known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

Fixture::Fixture(const Options options)
    : mcu(image(options.mcu_program), BLOFELD_DEFAULT_SLA, mcu_inputs(options)),
      dsp({.extal_hz = dsp_hz,
           .timebase_hz = common_hz,
           .switch_timing = DspClockSwitch::reference_cycle_boundary}),
      shared_address(options.shared_address) {
  constexpr unsigned padding = 128;
  std::array<std::uint32_t, padding> rom{};
  require(options.dsp_program.size() <= rom.size(), "Authored DSP fixture exceeds its ROM");
  std::ranges::copy(options.dsp_program, rom.begin());
  dsp.bus().install_rom({.space = DspSpace::p, .start = boot, .words = rom});
  mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
  runtime = std::make_unique<McuDspRuntime>(options.runtime, mcu, dsp);
  if (options.release) { runtime->release_dsp_reset(); }
}

McuDspRuntime::Observer Fixture::observer() {
  return [this](const DspRuntimeCycle &cycle) {
    cycles.push_back(cycle);
    if (on_cycle) { on_cycle(cycle); }
  };
}

void Fixture::attach() {
  mcu.set_cycle_observer([this](const Hc12BusCycle &cycle) {
    if (before_cycle) { before_cycle(cycle); }
    const bool shared = shared_address && (cycle.kind == 'w' || cycle.kind == 'W') &&
                        cycle.address == shared_address;
    runtime->begin_cycle(cycle, shared, provide_inputs, observer());
  });
  mcu.set_wait_period_observer([this](const McuWaitPeriod period) {
    runtime->begin_wait_period(period, provide_inputs, observer());
  });
  mcu.set_reset_period_observer([this](const McuResetPeriod period) {
    runtime->begin_reset_period(period, provide_inputs, observer());
  });
  mcu.bus().set_observer([this](const BusEvent &event) {
    if (!event.write || !shared_address || event.address != *shared_address || !deliver_write) {
      return;
    }
    runtime->observe_write(
        event,
        [this, event] {
          writes.push_back(event);
          if (on_write) { on_write(event); }
        },
        provide_inputs,
        observer());
  });
}

CpuStep Fixture::step() {
  const auto result = mcu.cpu().step();
  runtime->end_instruction();
  return result;
}

void Fixture::run(const unsigned count) {
  for (unsigned i = 0; i < count; ++i) { static_cast<void>(step()); }
}

void Fixture::restore(const McuDspRuntimeCheckpoint &snapshot) {
  runtime.reset();
  runtime = std::make_unique<McuDspRuntime>(snapshot, mcu, dsp);
  cycles.clear();
  writes.clear();
  attach();
}
}
