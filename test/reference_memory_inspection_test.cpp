// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_memory.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned common_hz = 180000000;
constexpr unsigned entry = 0x9000;
constexpr unsigned flash_base = 0x8000;
constexpr unsigned flash_alias = 0x5000;
constexpr unsigned nop = 0xa7;
constexpr unsigned load_a_extended = 0xb6;
constexpr unsigned protection_offset = 0x7f0d;
constexpr unsigned security_offset = 0x7f0f;
constexpr unsigned unsecured = 0xfe;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned ram_base = 0x1200;
constexpr unsigned q_data = 0x0260;
constexpr unsigned reset_serial_mask = 0x16;
constexpr unsigned other_u_pins = 0x3e;
constexpr unsigned other_u_levels = 0x2a;
constexpr unsigned rom_words = 1024;
constexpr unsigned budget = 32;
constexpr unsigned dsp_boot_rom = 0xff0000;
constexpr unsigned dsp_last_address = 0xffffff;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected a visible inspection or unobserved-input refusal");
}

McuFlashImage authored_image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  constexpr auto offset = entry - flash_base;
  // Independently authored LDAA $0260: the input is deliberately not driven.
  bytes[offset] = load_a_extended;
  bytes[offset + 1] = q_data >> 8U;
  bytes[offset + 2] = q_data & byte_mask;
  bytes[protection_offset] = byte_mask;
  bytes[security_offset] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

reference::Config config() {
  reference::Config result;
  result.mcu = {.clock = {.oscillator_hz = oscillator_hz,
                          .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                .memory_timing = McuRunMemory::unstretched_internal};
  result.dsp_clock = {.extal_hz = oscillator_hz,
                      .timebase_hz = common_hz,
                      .switch_timing = DspClockSwitch::reference_cycle_boundary};
  result.dsp = {.timing =
                    OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
                .maximum_cycles_per_period = 16,
                .writes = McuWriteOrder::before_mcu,
                .external_writes = McuWriteOrder::before_mcu,
                .reset_dsp_first = true,
                .initial_reset = OwnDspResetInputs{
                    .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  result.dma_grant = DspDmaGrant::available;
  result.mcu_u0_release = true;
  result.operations = budget;
  result.select_n = 2;
  result.clock = 4;
  result.mosi = 16;
  result.initial = {
      {.port = McuPort::b, .drive = {.mask = reset_serial_mask, .levels = 2}},
      {.port = McuPort::u, .drive = {.mask = other_u_pins, .levels = other_u_levels}}};
  return result;
}

void inspect_dsp_storage() {
  DspBus bus;
  constexpr unsigned address = 3;
  constexpr unsigned value = 0x123456;
  for (const auto space : {DspSpace::p, DspSpace::x, DspSpace::y}) {
    require(!bus.inspect_memory({.space = space, .address = address}),
            "Uninitialized DSP storage remains unknown in every space");
    bus.write({.space = space, .address = address, .value = value});
    require(bus.inspect_memory({.space = space, .address = address}) == value,
            "DSP inspection returns the actual stored24-bit word");
  }
  constexpr std::array rom{value, 0U};
  bus.install_rom({.start = dsp_boot_rom, .words = rom});
  const auto revision = bus.program_revision();
  require(bus.inspect_memory({.address = dsp_boot_rom}) == value &&
              bus.inspect_memory({.address = dsp_boot_rom + 1}) == 0 &&
              !bus.inspect_memory({.address = dsp_boot_rom + 2}) &&
              bus.program_revision() == revision,
          "ROM zero, nonzero and absent words stay distinct without code invalidation");
  constexpr unsigned shi_receive = 0xffff94;
  const auto failure = rejects([&] {
    static_cast<void>(bus.inspect_memory({.space = DspSpace::x, .address = shi_receive}));
  });
  require(failure.contains("storage inspection refuses I/O") && failure.contains("ffff94"),
          "Host inspection refuses a consuming FIFO register before invoking its guest read");
  constexpr unsigned vacant = 0x3000;
  constexpr unsigned vacant_word = 0xabcdef;
  bus.set_unselected_data_read_word(vacant_word);
  static_cast<void>(rejects(
      [&] { static_cast<void>(bus.inspect_memory({.space = DspSpace::x, .address = vacant})); }));
  require(bus.unselected_reads() == 0,
          "Vacant storage is not replaced with the configured guest unselected-read word");
  constexpr unsigned unselected_io = 0xffff80;
  bus.set_unselected_io_read_word(vacant_word);
  require(rejects([&] {
            static_cast<void>(bus.inspect_memory({.space = DspSpace::y, .address = unselected_io}));
          }).contains("storage inspection refuses I/O") &&
              bus.unselected_reads() == 0,
          "Unselected I/O cannot turn inspection into a synthetic guest response");
  constexpr unsigned overwide = 0x1000000;
  require(rejects([&] {
            static_cast<void>(bus.inspect_memory({.space = DspSpace::x, .address = overwide}));
          }).contains("exceeds 24 bits"),
          "Overwide addresses fail visibly");
  constexpr unsigned switched_program = 0x1000;
  constexpr unsigned switched_x = 0x10000;
  constexpr unsigned memory_switch = 0x80;
  constexpr unsigned switch_delay = 3;
  bus.write({.space = DspSpace::x, .address = switched_x, .value = value});
  bus.observe_omr(memory_switch);
  require(rejects([&] {
            static_cast<void>(bus.inspect_memory({.address = switched_program}));
          }).contains("elapsed switch cycles"),
          "Host inspection cannot finish a pending DSP memory switch");
  bus.advance_instruction_cycles(switch_delay);
  require(bus.inspect_memory({.address = switched_program}) == value,
          "Completed P-to-X alias uses the actual physical word");
}

void inspect_board() {
  const auto image = authored_image();
  const std::vector<std::uint32_t> rom(rom_words, 0);
  const reference::Execution execution{.image = image,
                                       .rom = rom,
                                       .specification = BLOFELD_DEFAULT_SLA,
                                       .flash_out = {},
                                       .instructions = budget};
  reference::Board board(config(), execution);
  const auto before = board.runtime_status();
  const auto unknown = board.inspect_mcu_memory(ram_base, 2);
  const auto code = board.inspect_mcu_memory(entry, 1);
  const auto alias = board.inspect_mcu_memory(flash_alias, 1);
  const auto io = board.inspect_mcu_memory(q_data, 1);
  const auto dsp_unknown = board.inspect_dsp_memory({.space = DspSpace::x}, 2);
  const auto dsp_rom = board.inspect_dsp_memory({.address = dsp_boot_rom}, 2);
  require(dsp_unknown.size() == 2 && !dsp_unknown[0] && !dsp_unknown[1] && dsp_rom.size() == 2 &&
              dsp_rom[0] == 0 && dsp_rom[1] == 0,
          "Board DSP copies retain unknown RAM and actual installed ROM");
  static_cast<void>(rejects(
      [&] { static_cast<void>(board.inspect_dsp_memory({.address = dsp_last_address}, 2)); }));
  require(board.inspect_dsp_memory({.address = dsp_last_address}, 0).empty(),
          "An empty DSP range consumes no address");
  require(
      unknown.size() == 2 && !unknown[0].value && !unknown[1].value &&
          code.front() == alias.front() && code.front().value == load_a_extended &&
          io.front().space == McuSpace::registers && !io.front().value,
      "Board inspection preserves physical storage identity, unknown RAM and peripheral selection");
  constexpr unsigned last_address = 0xffff;
  static_cast<void>(rejects([&] { static_cast<void>(board.inspect_mcu_memory(last_address, 2)); }));
  require(board.inspect_mcu_memory(last_address, 0).empty(), "An empty range consumes no address");
  const auto after = board.runtime_status();
  require(before.operations == after.operations && before.common_ticks == after.common_ticks &&
              before.dsp_steps == after.dsp_steps && before.mcu.pc == after.mcu.pc &&
              !after.faulted,
          "Cold and refused range inspections change neither native state nor clocks");
  const auto failure = rejects([&] { board.advance(); });
  require(failure.contains("0x0260") && failure.contains("GPIO input"),
          "Inspection never acknowledges or initializes the unobserved GPIO input");
  const auto failed = board.runtime_status();
  require(failed.faulted && board.inspect_mcu_memory(entry, 1).front() == code.front() &&
              !board.inspect_mcu_memory(ram_base, 1).front().value &&
              board.inspect_dsp_memory({.address = dsp_boot_rom}, 2) == dsp_rom,
          "A failed inactive boundary remains inspectable without inventing RAM contents");
  const auto retained = board.runtime_status();
  require(retained.faulted && retained.operations == failed.operations &&
              retained.common_ticks == failed.common_ticks && retained.mcu.pc == failed.mcu.pc,
          "Failure inspection cannot clear the fault or resume execution");
}
}

int main() {
  try {
    inspect_dsp_storage();
    inspect_board();
    std::cout << "MCU/DSP inspection: cold/range/alias/I-O/failed-boundary purity checks passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
