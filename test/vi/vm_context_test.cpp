// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/execute.hpp"
#include "execution.hpp"
#include "io/shi.hpp"
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
#include <ios>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void executed(const blofeld::vi::ExecutionReport &report, const char *message) {
  if (report.failure) {
    const auto &failure = *report.failure;
    std::cerr << "pc=0x" << std::hex << failure.pc << " address=0x" << failure.address
              << " value=0x" << failure.value << std::dec << " fault=" << failure.code << '\n';
  }
  require(!report.failure, message);
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
  blofeld::vi::Shi shi;
  const auto result = blofeld::vi::dsp::execute(program, shi, {});
  require(!result.failure && result.instructions == 3, "The DSP can write its OnCE output latch");
  constexpr auto reading = isa::program(std::array{input}, isa::stop);
  const auto refused = blofeld::vi::dsp::execute(reading, shi, {});
  require(refused.failure && refused.failure->operation == blofeld::vi::Access::read &&
              refused.failure->space == 'X' && refused.failure->address == once_address,
          "OnCE data is readable only through JTAG, never a fabricated DSP readback");
}

void boot_transfer() {
  namespace mcu = cpu12::isa;
  namespace dsp = dsp56300::isa;
  constexpr std::uint32_t control_address = 0xffff91;
  constexpr std::uint32_t fifo24 = 0x29;
  constexpr std::uint32_t absolute_address = 0x30;
  constexpr std::uint32_t receive_port = 0x14;
  constexpr auto move =
      dsp::opcode<dsp::Opcode::movep_program_low>()[0] | (absolute_address << 8U) | receive_port;
  constexpr auto reader =
      dsp::program(std::array{move, 0U, move, 1U}, dsp::opcode<dsp::Opcode::jmp_absolute_short>());
  constexpr auto payload = dsp::program(dsp::nop, dsp::stop);
  std::vector<cpu12::Byte> program;
  const auto store = [&program](const cpu12::Byte value, const cpu12::Byte address) {
    const std::array bytes{mcu::opcode<mcu::Opcode::ldab_immediate>()[0],
                           value,
                           mcu::opcode<mcu::Opcode::stab_direct>()[0],
                           address};
    program.insert(program.end(), bytes.begin(), bytes.end());
  };
  constexpr cpu12::Byte all_outputs = 0xff;
  constexpr cpu12::Byte stop_enabled = 0x7f;
  constexpr std::uint16_t stack = 0x1fff;
  constexpr auto enable_stop = mcu::program(
      mcu::opcode<mcu::Opcode::lds_immediate>(),
      std::array{static_cast<cpu12::Byte>(stack >> 8U), static_cast<cpu12::Byte>(stack)},
      mcu::opcode<mcu::Opcode::andcc_immediate>(),
      std::array{stop_enabled});
  program.insert(program.end(), enable_stop.begin(), enable_stop.end());
  store(all_outputs, 3);
  store(3, 1);
  constexpr unsigned word_bits = 24;
  for (const auto word : payload) {
    store(1, 1);
    for (unsigned bit = word_bits; bit != 0; --bit) {
      constexpr cpu12::Byte data_bit = 0x10;
      const auto level = static_cast<cpu12::Byte>(1 | (((word >> (bit - 1)) & 1) * data_bit));
      store(level, 1);
      store(static_cast<cpu12::Byte>(level | 4), 1);
      store(level, 1);
    }
    store(3, 1);
  }
  program.insert(program.end(), mcu::stop.begin(), mcu::stop.end());
  auto flash = image();
  std::ranges::copy(program, flash->bytes.begin());
  blofeld::vi::Shi shi;
  require(shi.write({.address = control_address, .value = fifo24}),
          "Configure the real receive FIFO");
  const auto sent = blofeld::vi::mcu::execute(*flash, shi, {});
  executed(sent, "Native MCU stores deliver complete boot words");
  const auto loaded = blofeld::vi::dsp::execute(reader, shi, {});
  executed(loaded, "Native DSP loads the delivered code");
  require(!loaded.failure && loaded.instructions == 5,
          "Native DSP receives, writes PRAM, jumps and executes the transmitted program");
  constexpr auto jump = dsp::opcode<dsp::Opcode::jmp_absolute_short>();
  const auto missing = blofeld::vi::dsp::execute(jump, shi, {});
  require(missing.failure && missing.failure->space == 'P' && missing.failure->address == 0,
          "A new DSP cannot execute uninitialized program RAM as zero-filled NOPs");
}

void masked_port_read() {
  namespace isa = cpu12::isa;
  constexpr std::uint16_t stack = 0x1fff;
  constexpr std::uint16_t pull_q = 0x264;
  constexpr std::uint16_t port_q = 0x260;
  constexpr cpu12::Byte clear_stop_disable = 0x7f;
  constexpr auto program = isa::program(
      isa::opcode<isa::Opcode::lds_immediate>(),
      std::array{static_cast<cpu12::Byte>(stack >> 8U), static_cast<cpu12::Byte>(stack)},
      isa::opcode<isa::Opcode::andcc_immediate>(),
      std::array{clear_stop_disable},
      isa::opcode<isa::Opcode::ldab_immediate>(),
      std::array<cpu12::Byte, 1>{1},
      isa::opcode<isa::Opcode::stab_extended>(),
      std::array{static_cast<cpu12::Byte>(pull_q >> 8U), static_cast<cpu12::Byte>(pull_q)},
      isa::opcode<isa::Opcode::brset_extended>(),
      std::array{static_cast<cpu12::Byte>(port_q >> 8U),
                 static_cast<cpu12::Byte>(port_q),
                 cpu12::Byte{1},
                 cpu12::Byte{3}},
      isa::opcode<isa::Opcode::ldaa_extended>(),
      std::array<cpu12::Byte, 2>{0x30, 0},
      isa::stop);
  auto flash = image();
  std::ranges::copy(program, flash->bytes.begin());
  blofeld::vi::Shi shi;
  const auto result = blofeld::vi::mcu::execute(*flash, shi, {});
  executed(result, "Native BRSET reads only its requested GPIO bit");
  require(result.instructions == 6, "The known bit branches around the unknown-memory access");
}
}

int main() try {
  debug_output();
  boot_transfer();
  masked_port_read();
  auto flash = image();
  constexpr auto words = dsp56300::isa::program(dsp56300::isa::nop, dsp56300::isa::stop);
  const std::vector<dsp56300::Word> boot(words.begin(), words.end());
  blofeld::vi::Shi shi;
  const auto mcu = blofeld::vi::mcu::execute(*flash, shi, {});
  const auto dsp = blofeld::vi::dsp::execute(boot, shi, {});
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
