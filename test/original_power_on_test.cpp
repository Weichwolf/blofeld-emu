// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using blofeld::test::require;
constexpr unsigned oscillator = 12000000, timebase = 540000000, common_factor = 9;
constexpr unsigned initial_edges = 195, reset_ticks = 5, common_reset_ticks = 45;
constexpr unsigned application = 0x8200, ram_begin = 0x1200, ram_end = 0x2600;
constexpr unsigned first_loopback = 0x8029, second_loopback = 0x8036, phy_poll = 0x8049;
constexpr unsigned clear_end = 0x807d, byte_mask = 0xff, phy_select = 0x80;
constexpr unsigned boot_retired = 90312, boot_cycles = 233747, initial_e = 0xfb;
constexpr unsigned common_bus_ticks = 90;

McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspFetchInputs inputs() {
  return {.dsp = {.extal_hz = oscillator,
                  .timebase_hz = timebase,
                  .timing = DspFetchTiming::reference_cycle_order,
                  .execution = DspReferenceExecution::staged_register_boundaries,
                  .pipeline = DspFetchPipeline::automatic_memory,
                  .resets = DspFetchResetTiming::reference_cycle_boundary},
          .maximum_dsp_cycles_per_period = 16,
          .reset_order = McuDspResetOrder::dsp_before_mcu,
          .initial_reset = McuDspFetchInitialReset{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
}

DspFetchCycleInputs pins() {
  return {.known_requests = 15, .grant = DspDmaGrant::available};
}

struct Observations {
  bool phy = false;
  bool cleared = false;
};

void observe(McuMachine &mcu, Observations &observed) {
  const auto pc = mcu.cpu().state().pc;
  if (pc == first_loopback || pc == second_loopback) {
    const auto output = mcu.bus().observe_pads(McuPort::e, 4);
    require(output.known == 4, "Original E2 output must exist before the explicit E3 loopback");
    const auto levels = static_cast<std::uint8_t>(3U | ((output.levels & 4U) != 0 ? 8U : 0U));
    mcu.bus().set_pin_drive(McuPort::e, {.mask = initial_e, .levels = levels});
  }
  if (pc == phy_poll && !observed.phy) {
    mcu.bus().observe_phy({.edges = 15});
    observed.phy = true;
  }
  if (pc == clear_end) {
    for (unsigned address = ram_begin; address < ram_end; ++address) {
      require(mcu.bus().read(static_cast<std::uint16_t>(address)) == 0,
              "Every SRAM byte is initialized by the original reset routine itself");
    }
    observed.cleared = true;
  }
}

void run(const McuFlashImage &image) {
  McuMachine mcu(image, BLOFELD_DEFAULT_SLA, contract(), {3});
  DspMachine dsp;
  // Explicit laboratory DSP-held profile; no chip ROM or board reset route is inferred.
  McuDspFetchSchedule schedule(inputs(), mcu, dsp);
  const auto cut = schedule.checkpoint();
  std::array<std::uint8_t, ram_end - ram_begin> expected_ram{};
  std::uint64_t expected_ticks = 0;
  for (unsigned replay = 0; replay < 2; ++replay) {
    schedule.restore(cut);
    std::uint64_t ledger = 0;
    unsigned edges = 0;
    std::string slots;
    mcu.set_reset_period_observer([&](const McuResetPeriod period) {
      require(period.duration_ticks == reset_ticks && mcu.run_clock().elapsed_ticks() == ledger,
              "Original cold start shares every literal initial SYSCLK edge");
      schedule.begin_reset_period(period, pins);
      ledger += reset_ticks;
      ++edges;
    });
    mcu.set_cycle_observer([&](const Hc12BusCycle cycle) {
      require(mcu.run_clock().elapsed_ticks() == ledger,
              "Native original cycles begin at their independent accumulated clock boundary");
      const auto divider = static_cast<unsigned>(mcu.bus().read(hc12::refdv)) + 1U;
      const auto source = (mcu.bus().read(hc12::clock_select) & phy_select) == 0 ? 5U : 1U;
      ledger += static_cast<std::uint64_t>(2U * divider) * source;
      if (mcu.system_reset_active()) { slots += cycle.kind; }
      schedule.begin_cycle(cycle, false, pins);
    });
    require(mcu.advance_system_reset(initial_edges, McuResetPad::high) == initial_edges,
            "Original initial vector is fetched after the complete documented POR sequence");
    schedule.end_system_reset();
    require(slots == "VfPPP" && edges == initial_edges &&
                schedule.run().hardware_reset_ticks() ==
                    (initial_edges * common_reset_ticks) + (5U * common_bus_ticks),
            "No initial MCU vector period is omitted from the shared DSP-held timeline");
    mcu.bus().set_pin_drive(McuPort::e, {.mask = initial_e, .levels = 3});
    mcu.bus().set_pin_drive(McuPort::q, {.floating = byte_mask});
    mcu.bus().set_pin_drive(McuPort::r, {.mask = 4, .levels = 4});
    Observations observed;
    unsigned retired = 0;
    while (mcu.cpu().state().pc != application && retired < boot_retired) {
      observe(mcu, observed);
      require(mcu.cpu().step().instruction_executed,
              "Original bootstrap retires a real instruction");
      schedule.end_instruction();
      ++retired;
    }
    require(observed.phy && observed.cleared && retired == boot_retired &&
                mcu.cpu().state().pc == application && mcu.cpu().state().sp == ram_end &&
                mcu.cpu().state().d == 0 && mcu.bus().read(hc12::modrr) == 0 &&
                mcu.run_clock().bus_cycles() == boot_cycles &&
                mcu.run_clock().reset_sysclk_edges() == initial_edges &&
                mcu.run_clock().elapsed_ticks() == ledger &&
                schedule.end_ticks() == ledger * common_factor &&
                schedule.run().hardware_reset_ticks() == ledger * common_factor &&
                schedule.run().completed_cycles() == 0 && !schedule.faulted(),
            "Unmodified bootstrap retains independent native count/state and one shared initial "
            "timeline");
    const auto final = mcu.bus().power_off_flash_image();
    require(final.bytes == image.bytes && final.known == image.known && final.fresh == image.fresh,
            "Original cold initialization preserves all NVM bytes, knowledge and erase history");
    for (unsigned i = 0; i < expected_ram.size(); ++i) {
      const auto value = mcu.bus().read(static_cast<std::uint16_t>(ram_begin + i));
      if (replay == 0) {
        expected_ram.at(i) = value;
      } else {
        require(value == expected_ram.at(i),
                "The entire initialized SRAM repeats after held-zero restore");
      }
    }
    if (replay == 0) {
      expected_ticks = ledger;
    } else {
      require(ledger == expected_ticks, "Original native bootstrap clock ledger repeats exactly");
    }
    std::cout << "original-POR suffix=" << replay << " retired=" << retired
              << " bus-cycles=" << boot_cycles << " MCU-ticks=" << ledger
              << " shared-ticks=" << schedule.end_ticks() << '\n';
  }
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 2) { throw Fault("Usage: original-power-on firmware.zip"); }
    const auto firmware = Firmware::load(std::filesystem::path(arguments[1]));
    run(McuFlashImage::from_known(firmware.mcu));
    std::cout
        << "Conditional original MCU bootstrap from POR; full instrument/DSP ROM remains open\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
