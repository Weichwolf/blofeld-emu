// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>

namespace {
void write_rom(const std::filesystem::path &path, const blofeld::test::DspProgram &program) {
  blofeld::DspBus bus;
  program.install(bus);
  std::ofstream stream(path, std::ios::binary);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  for (unsigned address = blofeld::test::boot; address < program.pc(); ++address) {
    const auto word = bus.peek_program(address);
    constexpr unsigned high_shift = 16;
    const std::array bytes{static_cast<char>(word >> high_shift),
                           static_cast<char>(word >> 8U),
                           static_cast<char>(word)};
    stream.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  }
  stream.close();
}

void echo_loop(blofeld::test::DspProgram &result) {
  // Own polling diagnostic: actual HRX -> RAM -> HTX, never host word injection.
  result.immediate("r4", 0x100);
  const auto loop = result.pc();
  result.emit("jclr #17,x:<<$ffff91," + blofeld::test::number(loop));
  result.emit("movep x:<<$ffff94,x0");
  result.emit("move x0,x:(r4)+");
  result.emit("movep x0,x:<<$ffff93");
  constexpr unsigned absolute_jump = 0x0af080;
  result.word(absolute_jump);
  result.word(loop);
}

blofeld::test::DspProgram capture_program() {
  constexpr unsigned status = 0xc00000;
  constexpr unsigned vector_offset = 0x54;
  constexpr unsigned absolute_jump = 0x0af080;
  blofeld::test::DspProgram result;
  result.immediate("vba", blofeld::test::boot);
  result.immediate("sr", status);
  result.emit("movep #>0,x:<<$ffff90");
  result.emit("movep #>$200,x:<<$fffffe");
  result.emit("movep #>0,x:<<$ffff8e");
  result.emit("movep #>1,x:<<$ffff8d");
  result.emit("movep #>$165,x:<<$ffff8f");
  const auto loop = result.pc();
  result.emit("nop");
  result.word(absolute_jump);
  result.word(loop);
  while (result.pc() < blofeld::test::boot + vector_offset) { result.emit("nop"); }
  blofeld::test::require(result.pc() == blofeld::test::boot + vector_offset,
                         "Own capture vector has its declared VBA-relative address");
  // Actual two-word fast vector enables a real SHI request output.
  result.emit("movep #>$89,x:<<$ffff91");
  return result;
}

blofeld::test::DspProgram esai_program(const std::string_view mode) {
  const bool second = mode.starts_with("esai1");
  const std::string space = second ? "y:<<" : "x:<<";
  constexpr unsigned primary = 0xffffa0;
  constexpr unsigned secondary = 0xffff80;
  constexpr unsigned legal_mux = 0x838;
  constexpr unsigned missing_hckt_mux = 0x818;
  constexpr unsigned port_control = 31;
  constexpr unsigned port_direction = 30;
  constexpr unsigned transmit_control = 21;
  constexpr unsigned clock_control = 22;
  const unsigned base = second ? secondary : primary;
  blofeld::test::DspProgram result;
  const auto put = [&](const unsigned offset, const unsigned value) {
    result.emit("movep #>" + blofeld::test::number(value) + ',' + space +
                blofeld::test::number(base + offset));
  };
  const auto mux = mode.ends_with("missing-hckt") ? missing_hckt_mux : legal_mux;
  constexpr unsigned word32 = 0x7d00;
  constexpr unsigned bit24 = 0xf900;
  constexpr unsigned external_clock = 0x200;
  const auto format = mode.ends_with("external") ? bit24 : word32;
  constexpr unsigned hckt_clock = 0x640301;
  constexpr unsigned first = 0xa5c39f;
  put(transmit_control, format);
  if (mode.ends_with("active")) {
    put(port_control, mux);
    put(port_direction, mux);
  }
  const bool external = mode.ends_with("external") || mode.ends_with("external-word");
  constexpr unsigned external_sckt_generated_frame = 0x400200;
  auto selected_clock = external ? external_clock : hckt_clock;
  if (mode.ends_with("external-sckt")) { selected_clock = external_sckt_generated_frame; }
  put(clock_control, selected_clock);
  if (!mode.ends_with("active")) {
    put(port_control, mux);
    put(port_direction, mux);
  }
  put(0, first);
  put(transmit_control, format | 1U);
  if (mode.ends_with("refill") || mode.ends_with("drop-sckt")) {
    constexpr unsigned ordinary_delay = 64;
    constexpr unsigned tied_delay = 154;
    const auto refill_delay = mode.ends_with("tied-refill") ? tied_delay : ordinary_delay;
    for (unsigned index = 0; index < refill_delay; ++index) { result.emit("nop"); }
    constexpr unsigned next = 0x123456;
    if (mode.ends_with("drop-sckt")) {
      constexpr unsigned sckt_mux = 8;
      put(port_control, mux & ~sckt_mux);
    } else {
      put(0, next);
    }
  }
  const auto loop = result.pc();
  result.emit("nop");
  constexpr unsigned absolute_jump = 0x0af080;
  result.word(absolute_jump);
  result.word(loop);
  return result;
}

blofeld::test::DspProgram program(const std::string_view mode) {
  if (mode == "esai0" || mode == "esai1" || mode == "esai0-refill" || mode == "esai1-refill" ||
      mode == "esai1-external" || mode == "esai1-tied-refill" || mode == "esai1-missing-hckt" ||
      mode == "esai1-active" || mode == "esai1-external-word" || mode == "esai1-external-sckt" ||
      mode == "esai1-drop-sckt") {
    return esai_program(mode);
  }
  if (mode == "capture-irq") { return capture_program(); }
  if (mode != "transmit" && mode != "unknown-transmit" && mode != "delayed-request" &&
      mode != "short-request" && mode != "echo") {
    throw blofeld::Fault("Unknown own SHI ROM scenario");
  }
  blofeld::test::DspProgram result;
  if (mode == "delayed-request" || mode == "short-request") {
    constexpr unsigned ordinary_delay = 64;
    constexpr unsigned short_delay = 257;
    const unsigned delay = mode == "short-request" ? short_delay : ordinary_delay;
    for (unsigned i = 0; i < delay; ++i) { result.emit("nop"); }
  }
  // Own mode-0, 24-bit single-word receive request. DSP56371UM chapter 7.
  result.emit("movep #>0,x:<<$ffff90");
  result.emit(mode == "echo" ? "movep #>$189,x:<<$ffff91" : "movep #>$89,x:<<$ffff91");
  if (mode == "transmit" || mode == "echo") { result.emit("movep #>$a15c37,x:<<$ffff93"); }
  if (mode == "echo") {
    echo_loop(result);
    return result;
  }
  if (mode == "delayed-request" || mode == "short-request") {
    const unsigned pulse = mode == "delayed-request" ? 16 : 0;
    for (unsigned i = 0; i < pulse; ++i) { result.emit("nop"); }
    result.emit("movep #>$88,x:<<$ffff91");
  }
  const auto loop = result.pc();
  result.emit("nop");
  constexpr unsigned absolute_jump = 0x0af080;
  result.word(absolute_jump);
  result.word(loop);
  return result;
}
}

int main(const int argc, char **argv) {
  try {
    if (argc != 3) { throw blofeld::Fault("Usage: reference-rom scenario output.bin"); }
    write_rom(argv[2], program(argv[1]));
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
