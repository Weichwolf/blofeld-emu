// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "board/nand_peer.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp_rom_program.hpp"
#include "esai_fixture.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "nand_persistence_fixture.hpp"
#include "storage/flash_image.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_file.hpp"
#include "storage/nand_image.hpp"
#include "storage/nand_timing.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using test::esai::require;
constexpr unsigned mcu_hz = 12000000, dsp_extal = 11289600;
constexpr std::uint64_t common_hz = 635040000000;
constexpr unsigned flash_base = 0x8000, entry = 0x9000, nop = 0xa7, serial_pins = 0x16;
constexpr unsigned protection = 0x7f0d, security = 0x7f0f, full = 0xff, unsecured = 0xfe;
constexpr unsigned control = 0xffffbd, direction = 0xffffbe, function = 0xffffbf;
constexpr unsigned f_data = 0xfffff5, f_direction = 0xfffff6, f_function = 0xfffff7;
constexpr unsigned bcr = 0xfffffb, pctl = 0xfffffd;
constexpr unsigned inactive = 0x300, command = 0x700, low = 0x500;
constexpr unsigned slow_pll = 0x02e112, fast_pll = 0x02a012, bus = 0x1ffff;
constexpr unsigned operation_limit = 100000, cycle_budget = 64;
constexpr unsigned control_outputs = 0xf00, data_direction = 0x4ff;
constexpr unsigned erase_setup = 0x60, erase_confirm = 0xd0, address_phase = 0xb00;
// Authored HCS12 BRA self displacement and DSP56300 WAIT encoding.
constexpr unsigned branch_self_displacement = 0xfe, dsp_wait = 0x000086;
constexpr unsigned program_setup = 0x80, program_confirm = 0x10;
constexpr std::uint64_t power_tick = common_hz / 10000;

McuFlashImage flash() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  // HCS12 BRA self is an actual three-cycle MCU loop.
  bytes.at(entry - flash_base) = 0x20;
  bytes.at(entry + 1 - flash_base) = branch_self_displacement;
  bytes[protection] = full;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

NandImage image(const bool erase = false) {
  NandImage result;
  NandArray::Page page;
  page.cells.fill({.value = static_cast<std::uint8_t>(erase ? 0U : full), .known = full});
  page.programs = erase ? NandProgramCount{.main = 1, .spare = 1} : NandProgramCount{};
  for (unsigned row = 0; row < Nand::block_pages; ++row) { result.pages.emplace(row, page); }
  result.blocks.emplace(
      0, NandBlock{.valid = true, .program_protected = false, .erase_protected = false});
  return result;
}

struct PortWrite {
  bool y = false;
  unsigned address;
  unsigned value;
};

void store(test::DspRomProgram &rom, const PortWrite input) {
  constexpr unsigned high_base = 0xffffc0;
  constexpr unsigned low_base = 0xffff80;
  constexpr std::array prefixes{0x07f400U, 0x07b480U, 0x08f480U, 0x09f480U};
  const bool high = input.address >= high_base;
  const auto group = (high ? 2U : 0U) + static_cast<unsigned>(input.y);
  rom.word(prefixes.at(group) | (input.address - (high ? high_base : low_base)));
  rom.word(input.value);
}

struct NandLatch {
  unsigned phase;
  unsigned value;
};

void latch(test::DspRomProgram &rom, const NandLatch input) {
  store(rom, {.address = control, .value = input.phase});
  store(rom, {.y = true, .address = f_data, .value = input.value});
  store(rom, {.address = control, .value = input.phase & ~0x200U});
  rom.word(0);
  store(rom, {.address = control, .value = input.phase});
}

test::DspRomProgram program(const bool erase = false) {
  test::DspRomProgram rom;
  for (const auto input :
       {PortWrite{.address = bcr, .value = bus},
        PortWrite{.address = control, .value = inactive},
        PortWrite{.address = function, .value = full},
        PortWrite{.address = direction, .value = control_outputs},
        PortWrite{.y = true, .address = f_data, .value = erase ? erase_setup : 0U},
        PortWrite{.y = true, .address = f_function, .value = inactive},
        PortWrite{.y = true, .address = f_direction, .value = data_direction},
        PortWrite{.address = pctl, .value = slow_pll},
        PortWrite{.address = pctl, .value = fast_pll},
        PortWrite{.address = control, .value = command},
        PortWrite{.address = control, .value = low}}) {
    store(rom, input);
  }
  rom.word(0);
  store(rom, {.address = control, .value = command});
  // Small-page NAND: real 00h selects the first-half pointer before 80h.
  if (!erase) { latch(rom, {.phase = command, .value = program_setup}); }
  if (!erase) { latch(rom, {.phase = address_phase, .value = 0}); }
  for (unsigned index = 0; index < 3; ++index) { latch(rom, {.phase = address_phase, .value = 0}); }
  if (!erase) { latch(rom, {.phase = inactive, .value = 0}); }
  // K9F1208 program-confirm 10h / erase-confirm D0h, then real DSP WAIT.
  latch(rom, {.phase = command, .value = erase ? erase_confirm : program_confirm});
  rom.word(dsp_wait);
  return rom;
}

reference::Config config(const NandTiming timing) {
  reference::Config result;
  result.mcu = {.clock = {.oscillator_hz = mcu_hz,
                          .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                .memory_timing = McuRunMemory::unstretched_internal};
  result.dsp_clock = {.extal_hz = dsp_extal,
                      .timebase_hz = common_hz,
                      .switch_timing = DspClockSwitch::functional_region_boundary};
  result.dsp = {.timing = OwnDspAutomaticTiming::functional_native_regions,
                .maximum_cycles_per_period = cycle_budget,
                .writes = McuWriteOrder::before_mcu,
                .external_writes = McuWriteOrder::before_mcu,
                .wait_dsp_first = true,
                .reset_dsp_first = true,
                .initial_reset = OwnDspResetInputs{
                    .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  result.dma_grant = DspDmaGrant::available;
  result.nand = reference::NandConnection{
      .image = image(), .continuation = NandReadContinuation::k9f1208_j, .timing = timing};
  result.power_on = reference::PowerOnTiming{.inputs = {3}, .reset_pad = McuResetPad::high};
  result.release_ticks = power_tick + common_hz / (mcu_hz / 2);
  result.operations = operation_limit;
  result.select_n = 2;
  result.clock = 4;
  result.mosi = 16;
  result.initial = {{.port = McuPort::b, .drive = {.mask = serial_pins, .levels = 2}}};
  return result;
}

NandPeerDetailed status(const reference::Board &board) {
  const auto result = board.nand_status();
  if (!result) { throw Fault("Expected the connected actual NAND"); }
  return *result;
}

reference::Config write_config(const std::filesystem::path &path,
                               const bool live,
                               const bool erase,
                               const bool inhibit) {
  auto settings = config(NandTiming::functional_protocol);
  if (!settings.nand) { throw Fault("Missing NAND configuration"); }
  auto &nand = *settings.nand;
  nand.image = live ? NandImage{} : image(erase);
  nand.backing = live ? reference::NandBacking::live_store : reference::NandBacking::cold_image;
  if (live) { nand.source_path = path; }
  nand.write_protect_n = true;
  nand.cell_profile.contract = NandCellContract::reference_pulse_inhibit;
  if (inhibit) {
    nand.cell_profile.cells.emplace(
        0,
        NandCellInhibit{.program = static_cast<std::uint8_t>(erase ? 0U : 1U),
                        .erase = static_cast<std::uint8_t>(erase ? 1U : 0U)});
  }
  return settings;
}

struct Completion {
  NandImage image;
  std::uint64_t deadline = 0;
};

Completion complete(reference::Board &board, const bool erase) {
  std::optional<std::uint64_t> deadline;
  for (unsigned index = 0; index < operation_limit; ++index) {
    board.advance();
    const auto observed = status(board);
    if (observed.base.pending == (erase ? NandPending::erase : NandPending::program)) {
      if (!deadline) {
        require(observed.base.ready == false && observed.base.next.has_value(),
                "A real authored confirm creates timed Busy work");
        deadline = observed.base.next;
      }
    } else if (deadline && observed.base.ready == true) {
      require(observed.base.ticks >= *deadline && !board.runtime_status().faulted,
              "The real peripheral calendar completes computed work without CPU fault");
      const auto result = board.nand_image();
      if (!result) { throw Fault("Missing completed NAND image"); }
      return {.image = *result, .deadline = *deadline};
    }
  }
  throw Fault("Real reset-to-confirm-to-completion exceeded the authored bound");
}

void exercise(const bool live, const bool erase, const bool inhibit) {
  const test::PersistenceDirectory directory;
  const auto path = directory.file();
  if (live) { save_nand_image(path, image(erase)); }
  const auto settings = write_config(path, live, erase, inhibit);
  const auto mcu = flash();
  const auto authored = program(erase);
  const std::vector<std::uint32_t> rom(authored.words().begin(), authored.words().end());
  const reference::Execution execution{.image = mcu,
                                       .rom = rom,
                                       .specification = BLOFELD_DEFAULT_SLA,
                                       .flash_out = {},
                                       .instructions = operation_limit};
  Completion completed;
  {
    reference::Board board(settings, execution);
    completed = complete(board, erase);
  }
  unsigned expected = inhibit ? 1U : 0U;
  if (erase) { expected = full - expected; }
  const auto cell = completed.image.pages.at(0).cells.at(0);
  require(cell.value == expected && cell.known == full,
          "Actual cells match independent pulse-inhibit program/erase expectations");
  const auto neighbor = completed.image.pages.at(0).cells.at(1);
  require(neighbor.value == full,
          "Program changes only the staged byte; erase transitions every block cell");
  if (live) {
    require(encode_nand_image(load_nand_image(path)) == encode_nand_image(completed.image),
            "Regular live Board commits its actually computed cells and metadata durably");
  }
  std::cout << "ACTUAL_BOARD " << (live ? "live" : "cold") << ' ' << (erase ? "erase" : "program")
            << " inhibit=" << inhibit << " cell=" << static_cast<unsigned>(cell.value)
            << " deadline=" << completed.deadline << '\n';
}

void refusals() {
  for (const std::string reason : {"cell model",
                                   "WP high",
                                   "block condition",
                                   "block protection",
                                   "protected-block",
                                   "program history"}) {
    auto settings = config(NandTiming::functional_protocol);
    if (!settings.nand) { throw Fault("Missing NAND configuration"); }
    auto &nand = *settings.nand;
    nand.write_protect_n = true;
    nand.cell_profile.contract = NandCellContract::reference_pulse_inhibit;
    if (reason == "cell model") { nand.cell_profile = {}; }
    if (reason == "WP high") { nand.write_protect_n.reset(); }
    if (reason == "block condition") { nand.image.blocks.clear(); }
    if (reason == "block protection") { nand.image.blocks.at(0).program_protected.reset(); }
    if (reason == "protected-block") { nand.image.blocks.at(0).program_protected = true; }
    if (reason == "program history") { nand.image.pages.at(0).programs.reset(); }
    const auto mcu = flash();
    const auto authored = program();
    const std::vector<std::uint32_t> rom(authored.words().begin(), authored.words().end());
    const reference::Execution execution{.image = mcu,
                                         .rom = rom,
                                         .specification = BLOFELD_DEFAULT_SLA,
                                         .flash_out = {},
                                         .instructions = operation_limit};
    std::string refusal;
    reference::Board board(settings, execution);
    try {
      while (!board.finished()) { board.advance(); }
    } catch (const Fault &error) { refusal = error.what(); }
    std::string message = "The selected cell model must retain independent contextual guard: ";
    message += reason;
    message += ": ";
    message += refusal;
    require(refusal.contains(reason) && refusal.contains("pc=") &&
                refusal.contains("X:0xffffbd/Y:0xfffff5"),
            message);
  }
}

std::optional<bool> expected_wp(const std::string_view options) {
  if (options.contains("high")) { return true; }
  if (options.contains("low")) { return false; }
  return std::nullopt;
}

void policy(const std::filesystem::path &profile, const bool invalid, const std::string &options) {
  std::optional<reference::Config> parsed;
  std::string refusal;
  try {
    parsed = reference::load_config(profile);
  } catch (const Fault &fault) { refusal = fault.what(); }
  require(invalid ? !refusal.empty() : parsed.has_value(),
          "Explicit cell policy accepts only documented complete, unique selectors");
  if (!parsed) { return; }
  if (!parsed->nand) { throw Fault("Parser lost the actual NAND backing"); }
  const auto &nand = *parsed->nand;
  const auto expected =
      options.empty() ? NandCellContract::unqualified : NandCellContract::reference_pulse_inhibit;
  require(nand.cell_profile.contract == expected && nand.cell_profile.cells.empty(),
          "Omission keeps cells unqualified; reference chooses physics without outcome data");
  require(nand.write_protect_n == expected_wp(options),
          "Cell policy must not infer an independent write-protection input");
}

void parser() {
  const test::PersistenceDirectory directory;
  const auto path = directory.file();
  const auto profile = directory.file("cells.board");
  save_nand_image(path, image());
  const std::string prefix =
      "blofeld-reference-board 2\nclock 12000000 0 12000000 1080000000 reference-boundaries\n"
      "dsp-execution native-regions\nordering dsp-before-mcu dsp-before-mcu 32\n"
      "spi-b 02 04 10 reject\ndsp-release 108180 low\noperations 4096\n";
  for (const bool live : {false, true}) {
    for (const std::string options : {"",
                                      "cells reference",
                                      "wp high cells reference",
                                      "cells reference functional-protocol wp low",
                                      "cells",
                                      "cells unknown",
                                      "cells reference cells reference",
                                      "cells reference extra"}) {
      {
        std::ofstream output(profile);
        output.exceptions(std::ios::failbit | std::ios::badbit);
        output << prefix << (live ? "nand-store j " : "nand-image j ") << std::quoted(path.string())
               << ' ' << options << '\n';
      }
      const bool invalid = options == "cells" || options.contains("unknown") ||
                           options.contains("extra") ||
                           options == "cells reference cells reference";
      policy(profile, invalid, options);
    }
  }
}
}

int main() {
  try {
    parser();
    refusals();
    for (const bool live : {false, true}) {
      for (const bool erase : {false, true}) {
        for (const bool inhibit : {false, true}) { exercise(live, erase, inhibit); }
      }
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
