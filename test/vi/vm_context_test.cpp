// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/execute.hpp"
#include "execution.hpp"
#include "isa.hpp"
#include "mcu/execute.hpp"
#include "storage/flash_image.hpp"
#include "support/isa.hpp"
#include "vm_context.hpp"

#include <algorithm>
#include <array>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::unique_ptr<blofeld::vi::FlashImage> image() {
  namespace isa = cpu12::isa;
  constexpr std::uint16_t stack = 0x1fff;
  constexpr std::uint16_t start = 0x8000;
  constexpr cpu12::Byte clear_stop_disable = 0x7f;
  constexpr auto program = isa::program(
      isa::opcode<isa::Opcode::lds_immediate>(),
      std::array{static_cast<cpu12::Byte>(stack >> 8U), static_cast<cpu12::Byte>(stack)},
      isa::opcode<isa::Opcode::andcc_immediate>(),
      std::array{clear_stop_disable},
      isa::stop);
  auto flash = std::make_unique<blofeld::vi::FlashImage>();
  flash->known.fill(1);
  std::ranges::copy(program, flash->bytes.begin());
  flash->bytes[blofeld::vi::FlashImage::byte_count - 2] = static_cast<cpu12::Byte>(start >> 8U);
  flash->bytes.back() = static_cast<cpu12::Byte>(start);
  return flash;
}

void debug_output() {
  namespace isa = dsp56300::isa;
  constexpr dsp56300::Word literal = 0x5a;
  constexpr dsp56300::Word x0_register = 4;
  constexpr dsp56300::Word write = 0x8000;
  constexpr dsp56300::Word once_port = 0x3c;
  constexpr dsp56300::Word once_address = 0xfffffc;
  constexpr auto load = isa::opcode<isa::Opcode::move_immediate_x0>()[0] | (literal << 8U);
  constexpr auto output =
      isa::opcode<isa::Opcode::movep_register_high>()[0] | (x0_register << 8U) | write | once_port;
  constexpr auto input = output & ~write;
  constexpr auto program = isa::program(std::array{load, output}, isa::stop);
  const auto result = blofeld::vi::dsp::execute(program, {});
  require(!result.failure && result.instructions == 3, "The DSP can write its OnCE output latch");
  constexpr auto reading = isa::program(std::array{input}, isa::stop);
  const auto refused = blofeld::vi::dsp::execute(reading, {});
  require(refused.failure && refused.failure->operation == blofeld::vi::Access::read &&
              refused.failure->space == 'X' && refused.failure->address == once_address,
          "OnCE data is readable only through JTAG, never a fabricated DSP readback");
}
}

int main() try {
  debug_output();
  auto flash = image();
  constexpr auto words = dsp56300::isa::program(dsp56300::isa::nop, dsp56300::isa::stop);
  const std::vector<dsp56300::Word> boot(words.begin(), words.end());
  const auto mcu = blofeld::vi::mcu::execute(*flash, {});
  const auto dsp = blofeld::vi::dsp::execute(boot, {});
  require(!mcu.failure && mcu.instructions == 3, "MCU owner performs real reset and native STOP");
  require(!dsp.failure && dsp.instructions == 2, "DSP owner executes its native ROM instructions");
  for (unsigned iteration = 0; iteration < 8; ++iteration) {
    blofeld::vi::VmContext context(image(), boot);
    const auto reports = context.run();
    require(!reports[0].exception && !reports[1].exception && !reports[0].failure &&
                !reports[1].failure,
            "Independent execution owners publish results and join cleanly");
    require(reports[0].instructions + reports[1].instructions > 0,
            "Native execution precedes coordinated shutdown");
    bool rejected = false;
    try {
      static_cast<void>(context.run());
    } catch (const std::logic_error &) { rejected = true; }
    require(rejected, "A VM run cannot be silently reused after shutdown");
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
