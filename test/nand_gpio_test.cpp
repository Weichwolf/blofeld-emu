// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/nand.hpp"
#include "dsp_boot_reference.hpp"
#include "dsp_norm_fixture.hpp"
#include "nand_gpio_fixture.hpp"
#include "nand_program_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <ostream>
#include <span>
#include <sstream>
#include <string>

namespace blofeld::test {
namespace {
constexpr unsigned sample_row = 0x12345;
constexpr unsigned instruction_limit = 10000;
constexpr unsigned missing_column = 37;

unsigned own_reader(DspProgram &program, const unsigned row) {
  setup_gpio(program);
  program.immediate("r0", 0);
  program.immediate("m0", word_mask);
  latch(program, {.value = 0, .mode = nand_cle});
  for (const unsigned byte : {0U, row & byte_mask, (row >> 8U) & byte_mask, row >> 16U}) {
    latch(program, {.value = byte, .mode = nand_ale});
  }
  movep(program, idle_c, port_c_data, 'x');
  // Release outputs first; then select input mode, before asserting RE.
  movep(program, input_f, port_f_direction, 'y');
  movep(program, all_inputs_f, port_f_control, 'y');
  const auto poll = program.pc();
  program.emit("brclr #12,x:<<$ffff8b,$0");
  for (unsigned column = 0; column < Nand::page_bytes; ++column) {
    program.emit("bclr #8,x:<<" + number(port_c_data));
    program.emit("movep y:<<" + number(port_f_data) + ",a");
    program.emit("move a1,x:(r0)+");
    program.emit("bset #8,x:<<" + number(port_c_data));
  }
  return poll;
}

std::array<std::uint8_t, Nand::page_bytes> sample_page() {
  std::array<std::uint8_t, Nand::page_bytes> bytes{};
  for (unsigned index = 0; index < bytes.size(); ++index) {
    constexpr unsigned multiplier = 29;
    bytes[index] = static_cast<std::uint8_t>((index * multiplier + (index >> 8U)) & byte_mask);
  }
  return bytes;
}

void step(DspCore &cpu, NandLink &link, std::ostream &trace) {
  const auto before = cpu.state();
  trace << "pc=" << hex(before.pc, 6) << " sr=" << hex(before.sr, 6)
        << " omr=" << hex(before.omr, 6) << " r0=" << hex(before.r[0], 6)
        << " r4=" << hex(before.r[4], 6) << " r5=" << hex(before.r[5], 6) << '\n';
  cpu.step();
  link.after_step(before);
}

void test_own_reader(const unsigned row, const bool missing) {
  DspProgram program;
  const auto poll = own_reader(program, row);
  DspBus bus;
  program.install(bus);
  Nand nand;
  const auto bytes = sample_page();
  if (missing) {
    nand.observe_bytes(row, 0, std::span(bytes).first(missing_column));
    nand.observe_bytes(row, missing_column + 1, std::span(bytes).subspan(missing_column + 1));
  } else {
    nand.observe_bytes(row, 0, bytes);
  }
  nand.observe_power_ready();
  std::ostringstream trace;
  NandLink link(bus, nand, trace, {.poll_pcs = {poll}, .selected = true, .outcomes = {}});
  DspCore cpu(bus);
  bool rejected = false;
  for (unsigned count = 0; cpu.state().pc != program.pc(); ++count) {
    require(count < instruction_limit, "Own native NAND reader instruction bound exceeded");
    try {
      step(cpu, link, trace);
    } catch (const Fault &fault) {
      const std::string message = fault.what();
      require(missing && message.contains("unknown contents") && message.contains("dsp-pc=") &&
                  message.contains("NAND-page=") && message.contains("column="),
              "Unexpected native NAND failure: " + message);
      rejected = true;
      break;
    }
  }
  const unsigned expected = missing ? missing_column : Nand::page_bytes;
  require(rejected == missing && link.reads() == expected && nand.column() == expected &&
              nand.page() == row && link.commands() == 1 && link.addresses() == 4 &&
              link.polls() == 3,
          "Native DSP must emit exact command/address/read counts and really wait for readiness");
  require(cpu.state().r[0] == expected, "Only native data stores may advance the destination");
  for (unsigned index = 0; index < expected; ++index) {
    require(bus.read({.space = DspSpace::x, .address = index}) == bytes[index],
            "Native GPIO MOVEP/store must preserve every main/spare byte, without host RAM writes");
  }
}

void test_link_preconditions(const std::optional<bool> selected) {
  DspProgram program;
  setup_gpio(program);
  // Both strobes change together; without a measured edge order this must reject.
  movep(program, 0, port_c_data, 'x');
  DspBus bus;
  program.install(bus);
  Nand nand;
  nand.observe_power_ready();
  std::ostringstream trace;
  NandLink link(bus, nand, trace, {.poll_pcs = {}, .selected = selected, .outcomes = {}});
  DspCore cpu(bus);
  std::string reason;
  for (unsigned count = 0; reason.empty(); ++count) {
    require(count < instruction_limit, "NAND link precondition diagnostic exceeded its bound");
    try {
      step(cpu, link, trace);
    } catch (const Fault &fault) { reason = fault.what(); }
  }
  std::string expected = "Unknown NAND chip-select";
  if (selected.has_value()) {
    expected = *selected ? "Unordered NAND RE/WE" : "NAND device deselected";
  }
  require(reason.contains(expected) && reason.contains("dsp-pc=") &&
              reason.contains("NAND-page=") && link.commands() == 0 && link.addresses() == 0 &&
              link.reads() == 0 && nand.column() == 0,
          "Missing select or unordered strobes must reject before any NAND operation: " + reason);
}

void install_original_stage(DspBus &bus, const Firmware &firmware) {
  constexpr std::size_t mcu_frame_offset = 0x7875;
  constexpr unsigned stage_base = 0x6a;
  constexpr std::size_t stage_words = 94;
  const auto frame =
      std::span<const std::uint8_t>(firmware.mcu).subspan(mcu_frame_offset, (stage_words + 2) * 3);
  require(dsp_boot_word(frame.first<3>()) == stage_words &&
              dsp_boot_word(frame.subspan<3, 3>()) == stage_base,
          "Pinned unmodified MCU image must retain the identified DSP frame");
  for (std::size_t index = 0; index < stage_words; ++index) {
    bus.write({.address = stage_base + static_cast<unsigned>(index),
               .value = dsp_boot_word(
                   std::span<const std::uint8_t, 3>(frame.subspan((index + 2) * 3, 3)))});
  }
}

void test_original_address(const Firmware &firmware, std::ostream &trace, const unsigned row) {
  // Isolated subroutine call with explicit own GPIO/register preconditions.
  // No original reset/clear/init sequence is claimed or bypassed in the boot test.
  constexpr unsigned original_reader = 0x9c;
  constexpr unsigned original_poll = 0xbb;
  constexpr unsigned original_turnaround = 0x93;
  constexpr unsigned original_row_output = 0xb0;
  constexpr unsigned reload_boundary = 0xc0b0;
  constexpr unsigned buffer = 0xc000;
  constexpr unsigned absolute_jsr = 0x0bf080;
  DspProgram program;
  setup_gpio(program);
  program.immediate("r4", reload_boundary);
  constexpr unsigned original_omr = 0x600389;
  program.immediate("omr", original_omr);
  program.immediate("r5", row);
  program.immediate("r2", buffer);
  program.immediate("m5", word_mask);
  program.word(absolute_jsr);
  program.word(original_reader);
  DspBus bus;
  install_original_stage(bus, firmware);
  program.install(bus);
  Nand nand;
  nand.observe_bytes(row, 0, sample_page());
  nand.observe_power_ready();
  NandLink link(bus, nand, trace, {.poll_pcs = {original_poll}, .selected = true, .outcomes = {}});
  DspCore cpu(bus);
  std::string reason;
  for (unsigned count = 0; reason.empty(); ++count) {
    require(count < instruction_limit, "Original NAND subroutine instruction bound exceeded");
    try {
      step(cpu, link, trace);
    } catch (const Fault &fault) { reason = fault.what(); }
  }
  const bool high_row = row == sample_row;
  if (high_row) {
    require(reason.contains("reserved-bit write at Y:0xfffff5") &&
                reason.contains("pc=" + hex(original_row_output, 6)) &&
                reason.contains("value=" + hex(row, 6)),
            "Original high row must expose its unqualified upper Port F write: " + reason);
  } else {
    require(reason.contains("read turnaround") &&
                reason.contains("dsp-pc=" + hex(original_turnaround, 6)) &&
                reason.contains("NAND-page=" + hex(row, 6)),
            "Original NAND route must expose the unqualified first read turnaround: " + reason);
  }
  trace << "expected-frontier=" << reason << '\n';
  require(link.commands() == 1 && link.addresses() == (high_row ? 1U : 4U) &&
              link.polls() == (high_row ? 0U : 3U) && link.reads() == 0 &&
              nand.page() == (high_row ? 0U : row) && nand.column() == 0 &&
              cpu.state().r[5] == row + 1,
          "Original instructions must preserve exact NAND addressing/frontier side effects");
  std::cout << "Original NAND subroutine row " << hex(row, 6)
            << ": expected unqualified frontier retained: " << reason << '\n';
}
}
}

int main(const int count, char **arguments) {
  try {
    if (count == 1) {
      blofeld::test::test_own_reader(0, false);
      blofeld::test::test_own_reader(blofeld::test::sample_row, false);
      blofeld::test::test_own_reader(blofeld::test::sample_row, true);
      blofeld::test::test_link_preconditions(std::nullopt);
      blofeld::test::test_link_preconditions(false);
      blofeld::test::test_link_preconditions(true);
      std::cout << "Own native GPIO reader transferred two full NAND pages including spare bytes; "
                   "missing contents fail with CPU and device context.\n";
    } else if (count == 3) {
      const auto firmware = blofeld::Firmware::load(std::filesystem::path(arguments[1]));
      std::ofstream trace(arguments[2]);
      blofeld::test::require(trace.is_open(), "Cannot open private original NAND trace");
      trace.exceptions(std::ios::failbit | std::ios::badbit);
      constexpr unsigned low_row = 0x123;
      blofeld::test::test_original_address(firmware, trace, low_row);
      blofeld::test::test_original_address(firmware, trace, blofeld::test::sample_row);
    } else {
      throw blofeld::Fault("Usage: nand-gpio-test [private-firmware.zip private.trace]");
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
