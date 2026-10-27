// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned mcu_hz = 18000000, dsp_hz = 12000000, common_hz = 540000000;
constexpr unsigned initial_ticks = 300, mcu_period = 60, dsp_period = 45, recovery = 90;
constexpr unsigned entry = 0x8000, nop = 0xa7, byte_mask = 0xff, unsecured = 0xfe;
constexpr unsigned u_data = 0x0278, u_direction = 0x027a, native_release = 480;
constexpr unsigned partial_tail = 17;
constexpr std::uint64_t sysclk_period = common_hz / mcu_hz;
constexpr unsigned ram = 0x1200, marker = 0x5a;

McuFlashImage image(const bool wire) {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  if (wire) {
    // Own LDAB #1;STAB extended PORTU. Independent1+3 native bus periods.
    constexpr std::array<std::uint8_t, 5> program{0xc6, 1, 0x7b, 2, 0x78};
    for (unsigned i = 0; i < program.size(); ++i) { bytes.at(i) = program[i]; }
  }
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_inputs() {
  return {.clock = {.oscillator_hz = mcu_hz,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspFetchInputs inputs(const unsigned tail) {
  return {.dsp = {.extal_hz = dsp_hz,
                  .timebase_hz = common_hz,
                  .timing = DspFetchTiming::reference_cycle_order,
                  .execution = DspReferenceExecution::staged_register_boundaries,
                  .pipeline = DspFetchPipeline::automatic_memory,
                  .resets = DspFetchResetTiming::reference_cycle_boundary},
          .epoch_ticks = initial_ticks - tail,
          .shared_write_order = McuDspTieOrder::dsp_before_mcu,
          .maximum_dsp_cycles_per_period = 16,
          .wait_order = McuDspWaitOrder::dsp_before_mcu,
          .reset_order = McuDspResetOrder::dsp_before_mcu,
          .initial_reset = McuDspFetchInitialReset{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
}

DspFetchCycleInputs cycle_inputs() {
  return {.known_requests = 15, .grant = DspDmaGrant::available};
}

DspMachine &prepare(DspMachine &machine) {
  DspProgram program;
  constexpr unsigned padding = 128;
  for (unsigned i = 0; i < padding; ++i) { program.emit("nop"); }
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return machine;
}

struct Scenario {
  unsigned tail = 0;
  bool wire = false;
};

struct Fixture {
  explicit Fixture(const Scenario scenario)
      : mcu(image(scenario.wire), BLOFELD_DEFAULT_SLA, mcu_inputs()),
        schedule(inputs(scenario.tail), mcu, prepare(dsp)),
        epoch(initial_ticks - scenario.tail) {
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    mcu.bus().write(ram, marker);
    mcu.bus().write(u_data, 0);
    mcu.bus().write(u_direction, 1);
    attach();
  }

  void attach() {
    const auto observe = [this](const auto &cycle) { record(cycle); };
    mcu.set_cycle_observer([this, observe](const auto cycle) {
      schedule.begin_cycle(
          cycle, cycle.kind == 'w' && cycle.address == u_data, cycle_inputs, observe);
    });
    mcu.set_reset_period_observer([this, observe](const auto period) {
      schedule.begin_reset_period(period, cycle_inputs, observe);
    });
    mcu.bus().set_observer([this, observe](const auto &event) {
      if (!event.write || event.address != u_data) { return; }
      schedule.observe_write(
          event,
          [this] {
            const auto output = mcu.bus().gpio_outputs(McuPort::u);
            require((output.mask & output.known & output.levels & 1U) != 0,
                    "Actual native PORTU write drives the observed reset release high");
            released = schedule.end_ticks();
            schedule.release_dsp_reset();
          },
          cycle_inputs,
          observe);
    });
  }

  void record(const DspFetchCycleResult &cycle) {
    require(cycle.completed_cycles == cycles + 1 && cycle.period_ticks == dsp_period,
            "Only actual normal pipeline periods increment the DSP cycle ledger");
    ++cycles;
    if (!cycle.pipeline) { throw Fault("Shared reset fixture lost its real pipeline"); }
    const auto &phase = *cycle.pipeline;
    if (phase.retirement_published) {
      require(phase.execute2 && phase.execute2->address == boot + retired &&
                  dsp.cpu().state().pc == boot + retired + 1,
              "Native NOP publication preserves the independent sequential PC expectation");
      ++retired;
    }
  }

  void step() {
    require(mcu.cpu().step().instruction_executed, "The actual MCU instruction executes");
    schedule.end_instruction();
  }

  void check_held(const std::uint64_t expected) const {
    require(dsp.hardware_reset_active() && schedule.run().hardware_reset_active() && cycles == 0 &&
                schedule.run().completed_cycles() == 0 &&
                schedule.run().elapsed_ticks() == expected - epoch &&
                schedule.run().hardware_reset_ticks() == expected - epoch &&
                schedule.run().timer_clk2_edges() == 0 && schedule.end_ticks() == expected &&
                dsp.cpu().debug_state().native_instructions == 0,
            "Actual MCU time advances while native DSP/F1/F2/DMA/CLK2 remain held");
  }

  void check_running() const {
    const auto common =
        mcu.run_clock().elapsed_ticks() * (common_hz / mcu.run_clock().timebase_hz());
    const auto reset_ticks = released + recovery - epoch;
    const auto expected_cycles = (common - released - recovery) / dsp_period;
    require(schedule.end_ticks() == common &&
                schedule.run().hardware_reset_ticks() == reset_ticks &&
                schedule.run().elapsed_ticks() == reset_ticks + (expected_cycles * dsp_period) &&
                cycles == expected_cycles && schedule.run().completed_cycles() == expected_cycles &&
                schedule.run().timer_clk2_edges() == expected_cycles / 2 && retired > 0,
            "Shared lifetime hold/recovery/native/CLK2 time has independent integer expectations");
  }

  McuMachine mcu;
  DspMachine dsp;
  McuDspFetchSchedule schedule;
  std::uint64_t epoch, cycles = 0, retired = 0, released = 0;
};

void held_replay(const unsigned tail) {
  Fixture f({.tail = tail});
  for (unsigned i = 0; i < 3; ++i) { f.step(); }
  constexpr unsigned cut = initial_ticks + (3 * mcu_period);
  f.check_held(cut);
  const auto saved = f.schedule.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.schedule.restore(saved);
    f.cycles = 0;
    f.retired = 0;
    f.attach();
    f.check_held(cut);
    f.released = cut;
    f.schedule.release_dsp_reset();
    for (unsigned i = 0; i < 10; ++i) { f.step(); }
    f.check_running();
  }
}

void native_wire(const unsigned tail) {
  Fixture f({.tail = tail, .wire = true});
  f.step();
  f.step();
  require(f.released == native_release && f.schedule.end_ticks() == native_release + mcu_period &&
              f.schedule.run().hardware_reset_remaining_ticks() == recovery - mcu_period,
          "Native STAB write releases RESET before its genuine trailing O bus slot");
  for (unsigned i = 0; i < 10; ++i) { f.step(); }
  f.check_running();
}

void held_mcu_reset() {
  Fixture f({.tail = partial_tail});
  f.schedule.begin_system_reset(McuResetSource::external, 3);
  f.attach();
  constexpr unsigned cut = 131;
  constexpr unsigned edges = 195;
  f.mcu.advance_system_reset(cut, McuResetPad::high);
  f.check_held(initial_ticks + (cut * sysclk_period));
  const auto saved = f.schedule.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.schedule.restore(saved);
    f.attach();
    f.mcu.advance_system_reset(edges - cut, McuResetPad::high);
    f.schedule.end_system_reset();
    constexpr auto expected = initial_ticks + (edges * sysclk_period) + (5ULL * mcu_period);
    f.check_held(expected);
    require(f.mcu.cpu().state().pc == entry && f.mcu.bus().read(ram) == marker,
            "Both held owners restore while native MCU reset fetches its vector and retains SRAM");
  }
}
}

int main() {
  try {
    for (const unsigned tail : {0U, 1U, 17U, 44U}) {
      held_replay(tail);
      native_wire(tail);
    }
    held_mcu_reset();
    std::cout << "Nine shared native MCU/DSP reset cases passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
