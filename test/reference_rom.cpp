// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
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

blofeld::test::DspProgram program(const std::string_view mode) {
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
