// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "dsp/ports.hpp"
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
constexpr cpu12::Address irq_handler = 0xa000, marker = 0x1800, led = 0x240;
constexpr cpu12::Address xirq_vector = 0x7ff4;
constexpr cpu12::Byte clear_xirq = 0xbf;
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

std::shared_ptr<const vi::mcu::Program> mcu(blofeld::test::McuImage &image, const bool dense) {
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
  if (dense) {
    put(code, marker, 0);
    const auto enable = cpu12::isa::program(cpu12::isa::opcode<Opcode::andcc_immediate>(),
                                            std::array<cpu12::Byte, 1>{clear_xirq});
    code.insert(code.end(), enable.begin(), enable.end());
    put(code, led, 1);
    const auto wait = cpu12::isa::program(
        cpu12::isa::opcode<Opcode::ldaa_extended>(),
        std::array<cpu12::Byte, 2>{static_cast<cpu12::Byte>(marker >> 8U),
                                   static_cast<cpu12::Byte>(marker & byte_mask)},
        cpu12::isa::opcode<Opcode::beq_relative>(),
        std::array<cpu12::Byte, 1>{0xf6});
    code.insert(code.end(), wait.begin(), wait.end());
    std::vector<cpu12::Byte> handler;
    put(handler, marker, 1);
    handler.insert(handler.end(), cpu12::isa::rti.begin(), cpu12::isa::rti.end());
    image.load(irq_handler, handler);
    image->bytes[xirq_vector] = static_cast<cpu12::Byte>(irq_handler >> 8U);
    image->bytes[xirq_vector + 1] = static_cast<cpu12::Byte>(irq_handler & byte_mask);
  } else {
    code.insert(code.end(), 1024, cpu12::isa::nop[0]);
  }
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

std::vector<dsp56300::Word> dsp_code(const bool dense) {
  namespace isa = dsp56300::isa;
  using dsp56300::Word;
  constexpr Word immediate = 0x34;
  constexpr Word absolute = 0x30;
  constexpr Word load = 0x8000;
  constexpr Word x0 = 4;
  const auto literal = isa::opcode<isa::Opcode::move_x_effective>()[0] | (immediate << 8U) | load;
  const auto store = isa::opcode<isa::Opcode::move_x_effective>()[0] | (absolute << 8U);
  std::vector<Word> code{literal, fifo24, store, spi_control};
  if (dense) {
    constexpr Word data = 0xffffbd;
    constexpr Word direction = 0xffffbe;
    constexpr Word function = 0xffffbf;
    constexpr Word ready_data = 0xffff9d;
    constexpr Word ready_direction = 0xffff9e;
    constexpr Word ready_pin = 0x200;
    const auto put = [&](const vi::dsp::Ports::Write access) {
      const auto operation = access.space == dsp56300::Space::x ? isa::Opcode::move_x_effective
                                                                : isa::Opcode::move_y_effective;
      code.insert(
          code.end(),
          {literal, access.value, static_cast<Word>(operation) | (absolute << 8U), access.address});
    };
    put({.address = data, .value = 0});
    put({.address = direction, .value = byte_mask});
    put({.address = function, .value = byte_mask});
    put({.space = dsp56300::Space::y, .address = ready_data, .value = ready_pin});
    put({.space = dsp56300::Space::y, .address = ready_direction, .value = ready_pin});
    constexpr Word count = 128;
    constexpr Word port_mask = 0x3f;
    const auto repeat = isa::opcode<isa::Opcode::rep_immediate>()[0] | (count << 8U);
    const auto change = isa::opcode<isa::Opcode::bchg_port_low>()[0] | ((data & port_mask) << 8U);
    for (unsigned block = 0; block < 32; ++block) { code.insert(code.end(), {repeat, change}); }
    put({.address = data, .value = 0x80});
    put({.space = dsp56300::Space::y, .address = ready_data, .value = 0});
    put({.space = dsp56300::Space::y, .address = ready_data, .value = ready_pin});
  }
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

namespace {
void exercise(const bool dense) {
  const auto code = dsp_code(dense);
  const std::array spans{dsp56300::CodeSpan{.address = 0xff0000, .words = code}};
  constexpr std::array entries{dsp56300::Word{0xff0000}};
  const std::array variants{
      dsp56300::CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array choices{vi::dsp::ProgramChoice{.variant = 0, .resumes = entries, .words = {}}};
  const auto dsp =
      std::make_shared<vi::dsp::Program>(aot::prepare_dsp_program({.variants = variants}, choices));
  blofeld::test::McuImage source;
  const auto cpu = mcu(source, dense);
  for (const std::uint64_t window :
       {std::uint64_t{38400}, dense ? std::uint64_t{128000} : std::uint64_t{19200}}) {
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
    std::cout << "DENSE=" << dense << " WINDOW=" << window << " WORD=" << failure->value << '\n';
  }
}
}

int main() try {
  for (const bool dense : {false, true}) { exercise(dense); }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
