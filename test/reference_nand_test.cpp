// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "board/nand_peer.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "esai_fixture.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
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
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
constexpr unsigned oscillator = 12000000, nop = 0xa7;
constexpr unsigned entry = 0x9000, protection = 0x7f0d, security = 0x7f0f;
constexpr unsigned instruction_budget = 4096, byte_mask = 0xff, unsecured = 0xfe;
constexpr unsigned serial_pins = 0x16, power_divisor = 10000, mcu_hz = oscillator / 2;
constexpr unsigned native_boot = 0xff0000;

McuFlashImage flash() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

NandImage cells() {
  NandArray array;
  constexpr std::array<std::uint8_t, 2> values{0x42, 0xa5};
  array.observe_bytes({.page = 3, .column = 5}, values);
  return array.image();
}

reference::Config config(const unsigned hz) {
  reference::Config c;
  c.mcu = {.clock = {.oscillator_hz = oscillator,
                     .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
           .memory_timing = McuRunMemory::unstretched_internal};
  c.dsp_clock = {.extal_hz = oscillator,
                 .timebase_hz = hz,
                 .switch_timing = DspClockSwitch::reference_cycle_boundary};
  c.dsp = {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
           .maximum_cycles_per_period = 32,
           .writes = McuWriteOrder::before_mcu,
           .external_writes = McuWriteOrder::before_mcu,
           .reset_dsp_first = true,
           .initial_reset = OwnDspResetInputs{
               .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  c.dma_grant = DspDmaGrant::available;
  c.nand =
      reference::NandConnection{.image = cells(), .continuation = NandReadContinuation::k9f1208_j};
  c.release_ticks = hz / power_divisor + hz / mcu_hz;
  c.operations = instruction_budget;
  c.select_n = 2;
  c.clock = 4;
  c.mosi = 16;
  c.initial = {{.port = McuPort::b, .drive = {.mask = serial_pins, .levels = 2}}};
  return c;
}

NandPeerDetailed nand_status(const reference::Board &board) {
  const auto result = board.nand_status();
  if (!result) { throw std::runtime_error("Expected a connected NAND peer"); }
  return *result;
}

std::vector<std::uint8_t> nand_bytes(const reference::Board &board) {
  const auto image = board.nand_image();
  if (!image) { throw std::runtime_error("Expected connected NAND storage"); }
  return encode_nand_image(*image);
}

void power_and_restore(const unsigned hz, const NandTiming timing) {
  const auto image = flash();
  const std::vector<std::uint32_t> rom(instruction_budget, 0);
  auto c = config(hz);
  if (!c.nand) { throw Fault("The authored config requires its NAND connection"); }
  c.nand->timing = timing;
  const reference::Execution execution{.image = image,
                                       .rom = rom,
                                       .specification = BLOFELD_DEFAULT_SLA,
                                       .flash_out = {},
                                       .instructions = instruction_budget};
  reference::Board board(c, execution);
  const auto original = encode_nand_image(cells());
  const auto power_tick = hz / power_divisor;
  const auto mcu_period = hz / mcu_hz;
  require(!nand_status(board).base.ready && nand_status(board).timing == timing &&
              nand_bytes(board) == original,
          "The regular Board retains cold unknown Ready and the explicitly supplied sparse cells");
  while (board.status().common_ticks < power_tick - mcu_period) { board.advance(); }
  const auto before = board.status();
  const auto peer_before = nand_status(board);
  auto conflicting = c;
  conflicting.dsp_initial = {{.index = 1, .level = true}};
  require(rejects([&] {
            const reference::Board duplicate(conflicting, execution);
          }).contains("NAND Ready owns DSP timer1") &&
              board.status() == before && board.nand_status() == peer_before,
          "A pinned input cannot replace the causal NAND Ready source");
  require(before.common_ticks == power_tick - mcu_period && !peer_before.base.ready,
          "The independent MCU cycle ledger reaches the boundary immediately before 100us");
  const auto checkpoint = board.checkpoint();
  board.advance();
  const auto after = board.status();
  const auto peer_after = nand_status(board);
  require(after.common_ticks == power_tick && peer_after.base.ready == true &&
              peer_after.base.ticks == power_tick && peer_after.power_ready,
          "The NAND calendar publishes its genuine power event while native DSP reset is held");
  board.restore(checkpoint);
  require(board.status() == before && board.nand_status() == peer_before,
          "Joint Board restoration restores cold protocol, clocks and pending Ready together");
  board.advance();
  require(board.status() == after && board.nand_status() == peer_after,
          "A fresh native owner dispatches exactly the same saved deadline without stale bindings");
  for (unsigned count = 0; count < 12; ++count) { board.advance(); }
  const auto active = board.status();
  const auto peer_active = nand_status(board);
  require(active.dsp_steps > 0 && peer_active.base.ticks == active.dsp_ticks &&
              peer_active.last_pc >= native_boot &&
              peer_active.last_pc < native_boot + instruction_budget,
          "Actual native instructions use the restored NAND owner through Board period callbacks");
  const auto running = board.checkpoint();
  board.advance();
  const auto suffix = board.status();
  const auto peer_suffix = board.nand_status();
  board.restore(running);
  board.advance();
  require(board.status() == suffix && board.nand_status() == peer_suffix &&
              nand_bytes(board) == original,
          "Active native continuations retain real cells and owner-specific pad bindings");
  require(board.nand_status() == peer_suffix,
          "A cold image projection cannot remove live Ready, protocol state or pending deadlines");
}

struct ConfigurationFiles {
  std::filesystem::path profile;
  std::filesystem::path image;
  std::string prefix;
};

void timing_configuration(const ConfigurationFiles &files, const bool live) {
  for (const std::string policy :
       {"", "candidate-minima", "functional-protocol", "unknown", "functional-protocol extra"}) {
    {
      std::ofstream output(files.profile);
      output.exceptions(std::ios::failbit | std::ios::badbit);
      output << files.prefix << (live ? "nand-store j " : "nand-image j ")
             << std::quoted(files.image.string()) << ' ' << policy << '\n';
    }
    if (policy == "unknown" || policy.contains("extra")) {
      require(!rejects([&] { static_cast<void>(reference::load_config(files.profile)); }).empty(),
              "Unknown timing selectors and surplus fields cannot silently choose a policy");
    } else {
      const auto c = reference::load_config(files.profile);
      const auto expected = policy == "functional-protocol" ? NandTiming::functional_protocol
                                                            : NandTiming::candidate_minima;
      const auto backing =
          live ? reference::NandBacking::live_store : reference::NandBacking::cold_image;
      const auto expected_image = live ? NandImage{} : cells();
      require(c.nand && c.nand->timing == expected && c.nand->backing == backing &&
                  c.nand->source_path == files.image &&
                  encode_nand_image(c.nand.value().image) == encode_nand_image(expected_image),
              "Timing selection defaults to candidate minima and preserves cold/live backing");
    }
  }
}

void configuration(const std::filesystem::path &directory) {
  std::filesystem::create_directories(directory);
  const auto image = directory / "authored sparse NAND.bfnand";
  save_nand_image(image, cells());
  const auto profile = directory / "nand.board";
  const std::string prefix =
      "blofeld-reference-board 2\nclock 12000000 0 12000000 1080000000 reference-boundaries\n"
      "dsp-execution reference-effects\nordering dsp-before-mcu dsp-before-mcu 32\n"
      "spi-b 02 04 10 reject\ndsp-release 108180 low\noperations 4096\n";
  for (const std::string kind : {"j", "p", "unknown"}) {
    {
      std::ofstream output(profile);
      output.exceptions(std::ios::failbit | std::ios::badbit);
      output << prefix << "nand-image " << kind << ' ' << std::quoted(image.string()) << '\n';
    }
    if (kind == "unknown") {
      require(
          rejects([&] { static_cast<void>(reference::load_config(profile)); }).contains("j or p"),
          "Unsupported NAND continuation profiles fail before being used");
    } else {
      const auto c = reference::load_config(profile);
      const auto continuation =
          kind == "j" ? NandReadContinuation::k9f1208_j : NandReadContinuation::k9f1208_p;
      require(c.nand && c.nand->continuation == continuation &&
                  encode_nand_image(c.nand.value().image) == encode_nand_image(cells()),
              "The regular profile imports exact sparse cells through a quoted image path");
    }
  }
  for (const bool live : {false, true}) {
    timing_configuration({.profile = profile, .image = image, .prefix = prefix}, live);
  }
}
}

int main(const int count, char **args) {
  try {
    require(count == 2, "A private test artifact directory is required");
    configuration(args[1]);
    for (const unsigned hz : {540000000U, 1080000000U}) {
      for (const auto timing : {NandTiming::candidate_minima, NandTiming::functional_protocol}) {
        power_and_restore(hz, timing);
      }
    }
    std::cout << "Regular Board NAND power, native callbacks, image import and restore passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
