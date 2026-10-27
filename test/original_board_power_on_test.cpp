// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "firmware/firmware.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned oscillator = 12000000, timebase = 540000000;
constexpr unsigned application = 0x8200, initial_e = 0xfb, phy_tick = 14895;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned budget = 100000, reset_edges = 195, boot_retired = 90312;
constexpr unsigned boot_cycles = 233747, boot_ticks = 4221621;

reference::Config config() {
  reference::Config c;
  c.mcu = {.clock = {.oscillator_hz = oscillator,
                     .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
           .memory_timing = McuRunMemory::unstretched_internal};
  c.pipeline = reference::PipelineTiming{
      .power_on = {3},
      .reset_pad = McuResetPad::high,
      .schedule = {.dsp = {.extal_hz = oscillator,
                           .timebase_hz = timebase,
                           .timing = DspFetchTiming::reference_cycle_order,
                           .execution = DspReferenceExecution::staged_register_boundaries,
                           .pipeline = DspFetchPipeline::automatic_memory,
                           .resets = DspFetchResetTiming::reference_cycle_boundary},
                   .maximum_dsp_cycles_per_period = 16,
                   .reset_order = McuDspResetOrder::dsp_before_mcu,
                   .initial_reset = McuDspFetchInitialReset{
                       .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}}};
  c.operations = budget;
  c.release_ticks = std::numeric_limits<std::uint64_t>::max();
  c.initial = {{.port = McuPort::e, .drive = {.mask = initial_e, .levels = 3}},
               {.port = McuPort::q, .drive = {.floating = byte_mask}},
               {.port = McuPort::r, .drive = {.mask = 4, .levels = 4}}};
  c.loopback = reference::Loopback{
      .source = McuPort::e, .source_mask = 4, .destination = McuPort::e, .destination_mask = 8};
  // Same explicit reference PHY input, shifted by the newly executed 195 SYSCLK edges.
  c.events = {
      {.ticks = phy_tick, .kind = reference::InputKind::phy_window, .drive = {}, .value = 15}};
  c.dsp_input_order = DspExternalOrder::external_before_dsp;
  c.maximum_dsp_inputs_per_advance = 1;
  return c;
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 2) { throw Fault("Usage: original-board-POR firmware.zip"); }
    const auto firmware = Firmware::load(arguments[1]);
    const auto image = McuFlashImage::from_known(firmware.mcu);
    const std::vector<std::uint32_t> no_rom;
    std::ostringstream trace;
    const reference::Execution execution{.image = image,
                                         .rom = no_rom,
                                         .specification = BLOFELD_DEFAULT_SLA,
                                         .flash_out = {},
                                         .trace = &trace,
                                         .instructions = budget};
    reference::Board board(config(), execution);
    reference::Status state;
    do {
      board.advance();
      state = board.status();
    } while ((!state.mcu_state_known || state.mcu.pc != application) && !board.finished());
    std::cout << "Original Board POR pc=" << hex(state.mcu.pc) << " retired=" << state.instructions
              << " bus-cycles=" << state.mcu_cycles << " common-ticks=" << state.common_ticks
              << " operations=" << state.operations << " DSP-held=" << state.held_reset
              << " DSP-cycles=" << state.dsp_cycles << '\n';
    if (state.mcu.pc != application || state.instructions != boot_retired ||
        state.mcu_cycles != boot_cycles || state.common_ticks != boot_ticks ||
        state.operations != boot_retired + reset_edges || !state.held_reset ||
        state.dsp_cycles != 0 || state.faulted) {
      throw Fault(
          "Original MCU Board POR differs from independent qualified reset/bootstrap ledger");
    }
    std::cout << "Conditional original MCU bootstrap through actual Board POR; no full instrument "
                 "claim\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
