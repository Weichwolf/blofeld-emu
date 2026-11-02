// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu_flash_live_fixture.hpp"
#include "nand_persistence_fixture.hpp"
#include "storage/flash_file.hpp"
#include "storage/flash_image.hpp"
#include "storage/nand.hpp"
#include "storage/nand_file.hpp"
#include "storage/nand_image.hpp"
#include "storage/private_file.hpp"
#include "storage/virtual_device_state.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using live::rejected;
using live::require;
constexpr unsigned common_hz = 540000000;
constexpr std::uint64_t budget = 1000;

reference::Config config(const std::filesystem::path &path) {
  reference::Config result;
  result.mcu = live::contract();
  result.dsp_clock = {.extal_hz = live::oscillator_hz,
                      .timebase_hz = common_hz,
                      .switch_timing = DspClockSwitch::reference_cycle_boundary};
  result.dsp = {.timing =
                    OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
                .maximum_cycles_per_period = 16,
                .writes = McuWriteOrder::unqualified,
                .external_writes = McuWriteOrder::unqualified,
                .reset_dsp_first = true,
                .initial_reset = OwnDspResetInputs{
                    .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  result.dma_grant = DspDmaGrant::unknown;
  result.power_on = reference::PowerOnTiming{.inputs = {3}, .reset_pad = McuResetPad::high};
  result.release_ticks = std::numeric_limits<std::uint64_t>::max();
  result.operations = budget * 2;
  result.select_n = 2;
  result.clock = 4;
  result.mosi = 16;
  constexpr unsigned serial_pins = 0x16;
  result.initial = {{.port = McuPort::b, .drive = {.mask = serial_pins, .levels = 2}},
                    {.port = McuPort::e, .drive = {.mask = 3, .levels = 3}}};
  result.dsp_input_order = DspPeerOrder::peer_first;
  result.maximum_dsp_inputs_per_advance = 4;
  result.nand = reference::NandConnection{.image = {},
                                          .continuation = NandReadContinuation::k9f1208_j,
                                          .backing = reference::NandBacking::live_store,
                                          .source_path = path};
  return result;
}

reference::NandConnection &connection(reference::Config &settings) {
  if (!settings.nand) { throw Fault("Test requires a NAND connection"); }
  return *settings.nand;
}

void binding() {
  const test::PersistenceDirectory directory;
  const auto nand_path = directory.file();
  const auto mcu_path = directory.file("unit.bfnvm");
  const auto original_nand = test::erased_nand_fixture();
  save_nand_image(nand_path, original_nand);
  const auto original = live::image(live::helper({}), {});
  const std::vector<std::uint32_t> no_rom;
  {
    auto owner =
        std::make_shared<VirtualDeviceState>(mcu_path, original, [] { return "EMUNAND00001"; });
    const auto input = owner->image();
    reference::Execution execution{.image = input,
                                   .rom = no_rom,
                                   .specification = BLOFELD_DEFAULT_SLA,
                                   .flash_out = {},
                                   .instructions = budget,
                                   .device_state = owner};
    auto settings = config(nand_path);
    auto unbound = execution;
    unbound.device_state.reset();
    auto cold = settings;
    cold.nand.reset();
    const reference::Board baseline(cold, unbound);
    const auto before = baseline.checkpoint();
    auto invalid = settings;
    connection(invalid).backing = reference::NandBacking::cold_image;
    require(rejected([&] {
              const reference::Board board(invalid, execution);
            }).contains("cold NAND connection"),
            "The old cold NAND plus live MCU guard remains effective");
    invalid = settings;
    connection(invalid).image = original_nand;
    require(rejected([&] {
              const reference::Board board(invalid, unbound);
            }).contains("second cold image"),
            "A live peer cannot silently choose a divergent in-memory image");
    invalid = settings;
    connection(invalid).source_path.reset();
    require(rejected([&] {
              const reference::Board board(invalid, unbound);
            }).contains("existing source file"),
            "Live backing requires an explicit actual file");
    auto forbidden = execution;
    forbidden.flash_out = directory.file("forbidden.bfnvm");
    require(rejected([&] {
              const reference::Board board(settings, forbidden);
            }).contains("excludes cold export"),
            "A live connection excludes disconnected cold export");
    reference::Board board(settings, execution);
    require(rejected([&] { static_cast<void>(board.checkpoint()); }).contains("RAM-only Board"),
            "Live MCU and NAND are never rewound with RAM");
    require(rejected([&] { board.restore(before); }).contains("cannot be rewound"),
            "Restore fails before a live owner can be detached");
    require(board.run() == 0, "Actual native MCU POR and flash helper complete");
    const auto state = board.status();
    const auto nand = board.nand_status();
    require(state.instructions == budget && state.held_reset && state.dsp_steps == 0 &&
                state.mcu_reset && state.mcu_reset->phase == McuResetPhase::complete && nand &&
                nand->power_ready && nand->base.ready == true,
            "The connected independent NAND power event and native MCU POR both run");
    const auto disk = load_flash_image(mcu_path);
    constexpr unsigned offset = live::target - live::entry;
    require(disk.known.test(offset) && disk.bytes.at(offset) == live::word_data >> 8U &&
                !disk.fresh.test(offset / 2),
            "Actual native MCU programming is durable while the NAND is mounted");
    require(encode_nand_image(load_nand_image(nand_path)) == encode_nand_image(original_nand),
            "Read-only NAND power events do not invent cell or history changes");
  }
  // NAND-only live ownership must also refuse RAM snapshots, independently of MCU binding.
  reference::Execution execution{.image = original,
                                 .rom = no_rom,
                                 .specification = BLOFELD_DEFAULT_SLA,
                                 .flash_out = {},
                                 .instructions = budget};
  const auto settings = config(nand_path);
  const reference::Board nand_only(settings, execution);
  require(rejected([&] { static_cast<void>(nand_only.checkpoint()); }).contains("RAM-only Board"),
          "Live NAND alone is sufficient to prohibit RAM rewind");
  require(rejected([&] { const reference::Board second(settings, execution); }).contains("lease"),
          "A second Board cannot acquire the mounted NAND writer");
}

void parser() {
  const test::PersistenceDirectory directory;
  const auto image = directory.file();
  save_nand_image(image, test::erased_nand_fixture());
  const auto path = directory.file("board.bfboard");
  const std::string header = "blofeld-reference-board 2\ndsp-execution reference-effects\n"
                             "clock 12000000 0 12000000 540000000 reference-boundaries\n"
                             "ordering dsp-before-mcu dsp-before-mcu 16\nspi-b 02 04 10 reject\n"
                             "dsp-release 18446744073709551615 low\noperations 1000\n";
  std::ostringstream row;
  row << "nand-store j " << std::quoted(image.string()) << '\n';
  const auto write = [&](const std::string &text) {
    write_private_file(path,
                       std::span(reinterpret_cast<const std::uint8_t *>(text.data()), text.size()));
  };
  write(header + row.str());
  const auto result = reference::load_config(path);
  require(result.nand && result.nand->backing == reference::NandBacking::live_store &&
              result.nand->image.pages.empty() && result.nand->source_path == image &&
              result.nand->continuation == NandReadContinuation::k9f1208_j,
          "The parser retains one live source path and the explicit part contract");
  write(header + row.str() + row.str());
  require(rejected([&] {
            static_cast<void>(reference::load_config(path));
          }).contains("Duplicate reference-board setting"),
          "A repeated key retains the existing parser's duplicate guard");
  std::ostringstream cold;
  cold << "nand-image j " << std::quoted(image.string()) << '\n';
  for (const bool live_first : {false, true}) {
    const auto selections = live_first ? row.str() + cold.str() : cold.str() + row.str();
    write(header + selections);
    require(rejected([&] {
              static_cast<void>(reference::load_config(path));
            }).contains("already configured"),
            "Different keys cannot silently replace a live or cold backing");
  }
}
}

int main() {
  try {
    parser();
    binding();
    std::cout << "Live MCU/NAND Board ownership, native POR/flash write, independent NAND power, "
                 "checkpoint guards and explicit backing parser passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
