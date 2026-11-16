// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "dsp/program.hpp"
#include "execution.hpp"
#include "io/midi_port.hpp"
#include "isa.hpp"
#include "mcu/program.hpp"
#include "storage/flash_image.hpp"
#include "support/isa.hpp"
#include "support/mcu.hpp"
#include "vm_context.hpp"

#include <array>
#include <chrono>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <exception>
#include <future>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {
namespace vi = blofeld::vi;
namespace aot = blofeld::app::aot;
constexpr dsp56300::Word word = 0xa591e7;
constexpr cpu12::Address entry = 0x8000, stack = 0x1fff, reset_output = 0x278;
constexpr cpu12::Address reset_direction = reset_output + 2;
constexpr cpu12::Byte byte_mask = 0xff, spi_pins = 0x16;
constexpr unsigned word_bits = 24;
constexpr dsp56300::Word sink_address = 0xc000, spi_control = 0xffff91, fifo24 = 0x29;
constexpr dsp56300::Word status_port = 0x11, receive_port = 0x14, receive_ready = 17;

void put(std::vector<cpu12::Byte> &code, cpu12::Address address, cpu12::Byte value) {
  using cpu12::isa::Opcode;
  constexpr auto load = cpu12::isa::opcode<Opcode::ldaa_immediate>();
  constexpr auto store = cpu12::isa::opcode<Opcode::staa_extended>();
  code.insert(code.end(),
              {load[0],
               value,
               store[0],
               static_cast<cpu12::Byte>(address >> 8U),
               static_cast<cpu12::Byte>(address & byte_mask)});
}

std::shared_ptr<const vi::mcu::Program> mcu(blofeld::test::McuImage &image) {
  using cpu12::isa::Opcode;
  auto seed =
      cpu12::isa::program(cpu12::isa::opcode<Opcode::lds_immediate>(),
                          std::array<cpu12::Byte, 2>{static_cast<cpu12::Byte>(stack >> 8U),
                                                     static_cast<cpu12::Byte>(stack & byte_mask)});
  std::vector<cpu12::Byte> code(seed.begin(), seed.end());
  put(code, reset_output, 0);
  put(code, reset_direction, 1);
  code.insert(code.end(), 64, cpu12::isa::nop[0]);
  put(code, reset_output, 1);
  code.insert(code.end(), 1024, cpu12::isa::nop[0]);
  put(code, 1, 3);
  put(code, 3, spi_pins);
  put(code, 1, 1);
  for (unsigned bit = word_bits; bit != 0; --bit) {
    const auto data = static_cast<cpu12::Byte>(1 | (((word >> (bit - 1)) & 1) * 0x10));
    put(code, 1, data);
    put(code, 1, static_cast<cpu12::Byte>(data | 4));
    put(code, 1, data);
  }
  put(code, 1, 3);
  code.insert(code.end(), cpu12::isa::wai.begin(), cpu12::isa::wai.end());
  image.load(entry, code);
  image->bytes[vi::FlashImage::byte_count - 2] = static_cast<cpu12::Byte>(entry >> 8U);
  image->bytes[vi::FlashImage::byte_count - 1] = static_cast<cpu12::Byte>(entry & byte_mask);
  return image.program();
}

std::vector<dsp56300::Word> dsp_code() {
  namespace isa = dsp56300::isa;
  using dsp56300::Word;
  constexpr Word immediate = 0x34;
  constexpr Word absolute = 0x30;
  constexpr Word load = 0x8000;
  constexpr Word x0 = 4;
  const auto literal = isa::opcode<isa::Opcode::move_x_effective>()[0] | (immediate << 8U) | load;
  const auto store = isa::opcode<isa::Opcode::move_x_effective>()[0] | (absolute << 8U);
  std::vector<Word> code{literal, fifo24, store, spi_control};
  const auto polling = 0xff0000 + static_cast<Word>(code.size());
  code.insert(code.end(),
              {isa::opcode<isa::Opcode::jclr_port_low>()[0] | (status_port << 8U) | receive_ready,
               polling,
               isa::opcode<isa::Opcode::movep_register_low>()[0] | (x0 << 8U) | receive_port,
               isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0,
               sink_address});
  return code;
}
}

int main() try {
  const auto code = dsp_code();
  const std::array spans{dsp56300::CodeSpan{.address = 0xff0000, .words = code}};
  constexpr std::array entries{dsp56300::Word{0xff0000}};
  const std::array variants{
      dsp56300::CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array choices{vi::dsp::ProgramChoice{.variant = 0, .resumes = entries, .words = {}}};
  const auto dsp =
      std::make_shared<vi::dsp::Program>(aot::prepare_dsp_program({.variants = variants}, choices));
  blofeld::test::McuImage source;
  const auto cpu = mcu(source);
  for (const std::uint64_t window : {std::uint64_t{19200}, std::uint64_t{38400}}) {
    auto image = std::make_unique<vi::FlashImage>(*source);
    vi::VmContext vm(
        std::move(image), code, cpu, {}, {}, vi::MidiPort::InputMode::stream, dsp, window);
    if (!vm.midi().finish_input()) { throw std::runtime_error("Input EOF refused"); }
    auto result = std::async(std::launch::async, [&] { return vm.run(); });
    if (result.wait_for(std::chrono::seconds{3}) != std::future_status::ready) {
      vm.request_stop();
      static_cast<void>(result.get());
      throw std::runtime_error("Authored whole-owner SPI/reset deadline expired");
    }
    const auto reports = result.get();
    for (const auto &report : reports) {
      if (report.exception) { std::rethrow_exception(report.exception); }
    }
    const auto failure = reports[1].failure;
    if (!failure) { throw std::runtime_error("Actual DSP has no completion marker"); }
    if (failure->address != sink_address || failure->value != word) {
      throw std::runtime_error("Prepared actual DSP did not receive the exact authored MCU word");
    }
    std::cout << "WINDOW=" << window << " WORD=" << failure->value << '\n';
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
