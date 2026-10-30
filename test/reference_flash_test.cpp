// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "mcu/mcu_flash_timing_state.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu_flash_live_fixture.hpp"
#include "storage/flash_file.hpp"
#include "storage/flash_image.hpp"
#include "storage/private_file.hpp"
#include "storage/virtual_device_state.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using live::rejected;
using live::require;
constexpr unsigned common_hz = 540000000;
constexpr unsigned maximum_operations = 10000;
constexpr unsigned completed_instructions = 200;
constexpr unsigned active_instructions = 100;

reference::Config config() {
  reference::Config result;
  result.mcu = live::contract();
  result.pipeline = reference::PipelineTiming{
      .power_on = {3},
      .reset_pad = McuResetPad::high,
      .schedule = {.dsp = {.extal_hz = live::oscillator_hz,
                           .timebase_hz = common_hz,
                           .timing = DspFetchTiming::reference_cycle_order,
                           .execution = DspReferenceExecution::staged_register_boundaries,
                           .pipeline = DspFetchPipeline::automatic_memory,
                           .resets = DspFetchResetTiming::reference_cycle_boundary},
                   .maximum_dsp_cycles_per_period = 16,
                   .reset_order = McuDspResetOrder::dsp_before_mcu,
                   .initial_reset = McuDspFetchInitialReset{
                       .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}}};
  result.release_ticks = std::numeric_limits<std::uint64_t>::max();
  result.operations = maximum_operations;
  result.select_n = 2;
  result.clock = 4;
  result.mosi = 16;
  constexpr unsigned serial_pins = 0x16;
  result.initial = {{.port = McuPort::b, .drive = {.mask = serial_pins, .levels = 2}},
                    {.port = McuPort::e, .drive = {.mask = 3, .levels = 3}}};
  result.dsp_input_order = DspExternalOrder::external_before_dsp;
  result.maximum_dsp_inputs_per_advance = 1;
  return result;
}

bool same(const McuFlashImage &a, const McuFlashImage &b) {
  return a.bytes == b.bytes && a.known == b.known && a.fresh == b.fresh;
}

struct Scenario {
  unsigned budget = completed_instructions;
  unsigned fail_publication = 0;
};

void exercise(const std::filesystem::path &path, const Scenario scenario) {
  const auto original = live::image(live::helper({}), {});
  const std::vector<std::uint32_t> no_rom;
  unsigned publications = 0;
  {
    auto owner =
        std::make_shared<VirtualDeviceState>(path, original, [] { return "EMULIVE00001"; });
    const auto input = owner->image();
    reference::Execution execution{.image = input,
                                   .rom = no_rom,
                                   .specification = BLOFELD_DEFAULT_SLA,
                                   .flash_out = {},
                                   .instructions = scenario.budget,
                                   .device_state = owner};
    auto settings = config();
    reference::Execution unbound = execution;
    unbound.device_state.reset();
    const reference::Board baseline(settings, unbound);
    const auto before = baseline.checkpoint();
    auto invalid = execution;
    invalid.flash_out = path.string() + ".forbidden";
    require(rejected([&] {
              const reference::Board rejected_board(settings, invalid);
            }).contains("excludes cold export"),
            "Invalid file ownership is rejected before device construction/publication");
    settings.checkpoint = {.operation = 0, .replays = 1};
    require(rejected([&] {
              const reference::Board rejected_board(settings, execution);
            }).contains("automatic RAM checkpoint replay"),
            "Automatic replay cannot operate a live external unit");
    settings.checkpoint.reset();
    reference::Board board(settings, execution);
    require(rejected([&] { static_cast<void>(board.checkpoint()); }).contains("RAM-only Board"),
            "Bound Board checkpoints cannot include external file history");
    require(rejected([&] { board.restore(before); }).contains("cannot be rewound"),
            "Bound restore fails before replacing native owners or transport sessions");
    owner->set_observer([&](const PrivateFilePhase phase) {
      if (phase == PrivateFilePhase::created) { ++publications; }
      if (publications == scenario.fail_publication && phase == PrivateFilePhase::published) {
        throw Fault("Own Board live-publication failure");
      }
    });
    const auto result = board.run();
    require(result == (scenario.fail_publication == 0 ? 0 : 2),
            "Board preserves the original execution result without a second failed export");
    if (scenario.fail_publication != 0) {
      require(rejected([&] { static_cast<void>(owner->image()); }).contains("reopen"),
              "Board failure leaves the actual writer quarantined until cold reopen");
    } else {
      const auto state = board.status();
      require(state.instructions == scenario.budget && state.held_reset && state.dsp_steps == 0 &&
                  state.mcu_reset && state.mcu_reset->phase == McuResetPhase::complete &&
                  same(decode_flash_image(state.cold_nvm), owner->image()),
              "Actual Board POR/guest instructions publish the current native cold cut");
    }
  }
  const bool complete = scenario.budget == completed_instructions && scenario.fail_publication != 1;
  require(publications == (complete ? 2U : 1U), "Only real transfer/completion phases publish");
  const auto disk = load_flash_image(path);
  constexpr unsigned offset = live::target - live::entry;
  require(!disk.fresh.test(offset / 2) && disk.known.test(offset) == complete &&
              (complete ? disk.bytes[offset] == live::word_data >> 8U : disk.bytes[offset] == 0),
          "Full completion or active/failed cut remains durable after Board destruction");
  const McuFlashImage unused;
  const VirtualDeviceState cold(path, unused, []() -> std::string {
    throw Fault("Existing unit data must not be factory-repaired");
  });
  require(same(disk, cold.image()), "A new file owner loads the exact Board result without repair");
  std::cout << "Board live flash budget=" << scenario.budget
            << " fault=" << scenario.fail_publication << " publications=" << publications
            << " complete=" << complete << '\n';
}

void write_text(const std::filesystem::path &path, const std::string &text) {
  write_private_file(path,
                     std::span(reinterpret_cast<const std::uint8_t *>(text.data()), text.size()));
}

void parser(const std::filesystem::path &path) {
  const std::string header = "blofeld-reference-board 1\n";
  const std::string body = "clock 12000000 0 12000000 540000000 reference-boundaries\n"
                           "ordering dsp-before-mcu dsp-before-mcu 16\nspi-b 02 04 10 reject\n"
                           "dsp-release 18446744073709551615 low\noperations 10000\n";
  const std::string choice = "mcu-flash-timing uf32-formula 1 zero-gap words\n";
  for (const bool early : {false, true}) {
    auto text = header;
    text.append(early ? choice : body);
    text.append(early ? body : choice);
    write_text(path, text);
    const auto result = reference::load_config(path).mcu.flash_timing;
    require(result.contract == FlashTimingContract::uf32_formula_candidate &&
                result.transfer_bus_cycles == 1 && result.verify_unit == FlashVerifyUnit::words,
            "Flash timing choices survive either ordering relative to clock configuration");
  }
  for (const std::string_view invalid : {"uf32-formula 0 zero-gap words",
                                         "uf32-formula 1 guess words",
                                         "uf32-formula 1 zero-gap guess",
                                         "unknown",
                                         "unqualified extra"}) {
    write_text(path, header + body + "mcu-flash-timing " + std::string(invalid) + "\n");
    static_cast<void>(rejected([&] { static_cast<void>(reference::load_config(path)); }));
  }
  write_text(path, header + body);
  require(reference::load_config(path).mcu.flash_timing.contract ==
              FlashTimingContract::unqualified,
          "Absent timing remains unqualified instead of silently selecting hardware timing");
}
}

int main(const int argc, char **argv) {
  try {
    require(argc == 2, "Expected a fresh private test directory");
    const std::filesystem::path directory(argv[1]);
    parser(directory / "timing.bfboard");
    exercise(directory / "board-complete.bfnvm", {});
    exercise(directory / "board-active.bfnvm", {.budget = active_instructions});
    exercise(directory / "board-fault-transfer.bfnvm", {.fail_publication = 1});
    exercise(directory / "board-fault-complete.bfnvm", {.fail_publication = 2});
    std::cout << "Board live flash:POR,guest publication,active cut,original I/O cause and file "
                 "guards passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
