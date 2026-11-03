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
#include "storage/flash_image.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_image.hpp"
#include "storage/nand_timing.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::esai::require;
constexpr unsigned mcu_hz = 12000000, dsp_extal = 11289600;
constexpr std::uint64_t common_hz = 635040000000;
constexpr unsigned entry = 0x9000, nop = 0xa7, serial_pins = 0x16;
constexpr unsigned protection = 0x7f0d, security = 0x7f0f, full = 0xff, unsecured = 0xfe;
constexpr unsigned control = 0xffffbd, direction = 0xffffbe, function = 0xffffbf;
constexpr unsigned f_data = 0xfffff5, f_direction = 0xfffff6, f_function = 0xfffff7;
constexpr unsigned bcr = 0xfffffb, pctl = 0xfffffd;
constexpr unsigned inactive = 0x300, command = 0x700, low = 0x500;
constexpr unsigned slow_pll = 0x02e112, fast_pll = 0x02a012, bus = 0x1ffff;
constexpr unsigned operation_limit = 10000, cycle_budget = 64, padding_words = 64;
constexpr unsigned jump = 0x0af080, sample = 0x42;
constexpr unsigned control_outputs = 0xf00, data_direction = 0x4ff;
constexpr std::uint64_t power_tick = common_hz / 10000;

McuFlashImage flash() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  bytes[protection] = full;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

NandImage image() {
  NandArray cells;
  constexpr std::array<std::uint8_t, 1> value{sample};
  cells.observe_bytes({.page = 0, .column = 0}, value);
  return cells.image();
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

test::DspRomProgram program() {
  test::DspRomProgram rom;
  for (const auto input : {PortWrite{.address = bcr, .value = bus},
                           PortWrite{.address = control, .value = inactive},
                           PortWrite{.address = function, .value = full},
                           PortWrite{.address = direction, .value = control_outputs},
                           PortWrite{.y = true, .address = f_data, .value = full},
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
  const auto loop = rom.pc();
  rom.word(0);
  rom.word(jump);
  rom.word(loop);
  for (unsigned index = 0; index < padding_words; ++index) { rom.word(0); }
  return rom;
}

reference::Config config(const NandTiming timing) {
  reference::Config result;
  result.mcu = {.clock = {.oscillator_hz = mcu_hz,
                          .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                .memory_timing = McuRunMemory::unstretched_internal};
  result.dsp_clock = {.extal_hz = dsp_extal,
                      .timebase_hz = common_hz,
                      .switch_timing = DspClockSwitch::reference_cycle_boundary};
  result.dsp = {.timing = OwnDspAutomaticTiming::native_cycle_budget,
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
  if (!result) { throw Fault("The authored native reset test requires its connected NAND"); }
  return *result;
}

void finish(reference::Board &board) {
  for (unsigned count = 0; count < operation_limit; ++count) {
    if (status(board).base.ready == true) { return; }
    board.advance();
  }
  throw Fault("The real reset Busy deadline did not complete within the authored bound");
}

void exercise(const NandTiming timing) {
  const auto mcu = flash();
  const auto authored = program();
  const std::vector<std::uint32_t> rom(authored.words().begin(), authored.words().end());
  const auto settings = config(timing);
  const reference::Execution execution{.image = mcu,
                                       .rom = rom,
                                       .specification = BLOFELD_DEFAULT_SLA,
                                       .flash_out = {},
                                       .instructions = operation_limit};
  reference::Board board(settings, execution);
  const auto initial = board.runtime_status();
  require(initial.held_reset && initial.instructions == 0 && initial.dsp_steps == 0,
          "Both native processors begin at actual reset before any authored instruction");
  std::optional<reference::Checkpoint> pending;
  try {
    for (unsigned count = 0; count < operation_limit; ++count) {
      board.advance();
      if (status(board).base.pending == NandPending::reset) {
        pending = board.checkpoint();
        break;
      }
    }
  } catch (const Fault &error) {
    const std::string message(error.what());
    require(timing == NandTiming::candidate_minima && message.contains("tWP") &&
                message.contains("X:0xffffbd/Y:0xfffff5") && message.contains("pc="),
            "The same native pulse retains its strict contextual candidate-minimum refusal");
    std::cout << "Native POR strict candidate short-pulse refusal preserved\n";
    return;
  }
  require(timing == NandTiming::functional_protocol && pending.has_value(),
          "Only the selected functional timing admits the original-shaped real FF reset pulse");
  if (!pending) { throw Fault("Expected a complete pending reset checkpoint"); }
  const auto before = board.runtime_status();
  const auto nand_before = status(board);
  require(nand_before.timing == timing && nand_before.base.ready == false &&
              nand_before.base.next && nand_before.base.cell.value == sample,
          "Actual native command latching starts Busy while preserving the real sparse cell");
  finish(board);
  const auto expected = board.runtime_status();
  const auto nand_expected = status(board);
  for (unsigned replay = 0; replay < 2; ++replay) {
    board.restore(pending.value());
    require(board.runtime_status() == before && status(board) == nand_before,
            "A fresh native owner restores the selected timing and full pending NAND state");
    finish(board);
    require(board.runtime_status() == expected && status(board) == nand_expected,
            "Original scheduled reset work reproduces the complete native state and exact clocks");
  }
  require(nand_expected.base.pending == NandPending::none && nand_expected.base.ready == true &&
              nand_expected.base.cell.value == sample && !nand_expected.trace_access &&
              !expected.faulted && expected.dsp_steps > 0,
          "Real native reset completion preserves data and creates no fake array read or PCM");
  std::cout << "Native POR functional reset/Busy/cell/policy and two full owner restores pass\n";
}
}

int main() {
  try {
    exercise(NandTiming::candidate_minima);
    exercise(NandTiming::functional_protocol);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
