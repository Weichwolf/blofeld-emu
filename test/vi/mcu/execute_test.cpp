// SPDX-License-Identifier: GPL-3.0-only
#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/shi.hpp"
#include "isa.hpp"
#include "mcu/execute.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <vector>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void native_boot_word() {
  namespace isa = cpu12::isa;
  constexpr std::uint32_t control = 0xffff91;
  constexpr std::uint32_t receive = 0xffff94;
  constexpr std::uint32_t fifo24 = 0x29;
  constexpr std::uint32_t datum = 0xa591e7;
  constexpr std::uint8_t serial_outputs = 0x17;
  constexpr std::uint8_t data_bit = 0x10;
  constexpr cpu12::Address marker = 0x3000;
  std::vector<cpu12::Byte> program;
  const auto append = [&program](const auto &bytes) {
    program.insert(program.end(), bytes.begin(), bytes.end());
  };
  const auto store = [&](const cpu12::Byte value, const cpu12::Byte address) {
    append(std::array{isa::opcode<isa::Opcode::ldab_immediate>()[0],
                      value,
                      isa::opcode<isa::Opcode::stab_direct>()[0],
                      address});
  };
  store(3, 1);
  store(serial_outputs, 3);
  store(1, 1);
  constexpr unsigned bits = 24;
  for (unsigned bit = bits; bit != 0; --bit) {
    store(static_cast<cpu12::Byte>(1 | (((datum >> (bit - 1)) & 1) * data_bit)), 1);
    append(isa::program(isa::opcode<isa::Opcode::bset_direct>(),
                        std::array<cpu12::Byte, 2>{1, 4},
                        isa::opcode<isa::Opcode::bclr_direct>(),
                        std::array<cpu12::Byte, 2>{1, 4}));
  }
  store(3, 1);
  constexpr cpu12::Byte application_outputs = 0xe1;
  store(application_outputs, 3);
  constexpr auto finish = isa::program(isa::opcode<isa::Opcode::ldaa_direct>(),
                                       std::array<cpu12::Byte, 1>{1},
                                       isa::opcode<isa::Opcode::staa_extended>(),
                                       std::array<cpu12::Byte, 2>{0x30, 0});
  append(finish);
  auto image = std::make_unique<blofeld::vi::FlashImage>();
  image->known.fill(1);
  std::ranges::copy(program, image->bytes.begin());
  image->bytes[blofeld::vi::FlashImage::byte_count - 2] = 0x80;
  image->bytes.back() = 0;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  require(shi.write({.address = control, .value = fifo24}), "Configure actual SPI receive");
  const auto report = blofeld::vi::mcu::execute(*image, shi, reset, {});
  constexpr cpu12::Byte released_readback = 0x17;
  require(report.failure && report.failure->address == marker &&
              report.failure->value == released_readback,
          "Native transfers preserve full-byte RMW; released SPI inputs read their real pull-ups");
  std::uint32_t word = 0;
  require(shi.read(receive, word) && word == datum && !shi.read(receive, word),
          "Full-byte RMW preserves data/select bits and delivers exactly one original word");
}

void firmware_controls_reset() {
  namespace isa = cpu12::isa;
  constexpr cpu12::Address marker = 0x3000;
  constexpr auto program = isa::program(isa::opcode<isa::Opcode::clr_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x78},
                                        isa::opcode<isa::Opcode::ldaa_immediate>(),
                                        std::array<cpu12::Byte, 1>{1},
                                        isa::opcode<isa::Opcode::staa_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x7a},
                                        isa::opcode<isa::Opcode::brclr_extended>(),
                                        std::array<cpu12::Byte, 4>{2, 0x78, 1, 3},
                                        isa::opcode<isa::Opcode::ldaa_extended>(),
                                        std::array<cpu12::Byte, 2>{0x30, 2},
                                        isa::opcode<isa::Opcode::clr_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x7a},
                                        isa::opcode<isa::Opcode::brset_extended>(),
                                        std::array<cpu12::Byte, 4>{2, 0x78, 1, 3},
                                        isa::opcode<isa::Opcode::ldaa_extended>(),
                                        std::array<cpu12::Byte, 2>{0x30, 1},
                                        isa::opcode<isa::Opcode::ldaa_extended>(),
                                        std::array<cpu12::Byte, 2>{0x30, 0});
  auto image = std::make_unique<blofeld::vi::FlashImage>();
  image->known.fill(1);
  std::ranges::copy(program, image->bytes.begin());
  image->bytes[blofeld::vi::FlashImage::byte_count - 2] = 0x80;
  image->bytes.back() = 0;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::ExecutionReport report;
  std::atomic<bool> done{false};
  std::jthread worker([&](const std::stop_token &stop) {
    try {
      report = blofeld::vi::mcu::execute(*image, shi, reset, stop);
    } catch (...) { report.exception = std::current_exception(); }
    done.store(true, std::memory_order_release);
  });
  while (!reset.state().held) {
    require(!done.load(std::memory_order_acquire), "The MCU must assert reset through U0");
    std::this_thread::yield();
  }
  require(reset.state().epoch == 1 && reset.acknowledge(1), "The DSP owner applies the reset");
  worker.join();
  require(!report.exception && report.failure && report.failure->address == marker &&
              report.failure->operation == blofeld::vi::Access::read && !reset.state().held,
          "Native GPIO reads see driven low, then the released RESET pull-up");
}

void native_display_status() {
  namespace isa = cpu12::isa;
  constexpr cpu12::Byte outputs = 0xe1;
  constexpr cpu12::Byte all_bits = 0xff;
  constexpr cpu12::Byte display_on = 0xaf;
  constexpr cpu12::Byte read_mode = 0x60;
  constexpr cpu12::Byte read_enable = 0x14;
  std::vector<cpu12::Byte> program;
  const auto store = [&](const cpu12::Byte value, const cpu12::Byte address) {
    const auto bytes = std::array{isa::opcode<isa::Opcode::ldab_immediate>()[0],
                                  value,
                                  isa::opcode<isa::Opcode::stab_direct>()[0],
                                  address};
    program.insert(program.end(), bytes.begin(), bytes.end());
  };
  store(outputs, 3);
  store(0x40, 1);
  store(all_bits, 2);
  store(display_on, 0);
  store(0x10, 8);
  store(0, 8);
  store(0, 2);
  store(read_mode, 1);
  store(read_enable, 8);
  constexpr auto finish = isa::program(isa::opcode<isa::Opcode::ldaa_direct>(),
                                       std::array<cpu12::Byte, 1>{0},
                                       isa::opcode<isa::Opcode::staa_extended>(),
                                       std::array<cpu12::Byte, 2>{0x30, 0});
  program.insert(program.end(), finish.begin(), finish.end());
  auto image = std::make_unique<blofeld::vi::FlashImage>();
  image->known.fill(1);
  std::ranges::copy(program, image->bytes.begin());
  image->bytes[blofeld::vi::FlashImage::byte_count - 2] = 0x80;
  image->bytes.back() = 0;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::mcu::execute(*image, shi, reset, {});
  constexpr cpu12::Address marker = 0x3000;
  require(report.failure && report.failure->address == marker && report.failure->value == 0x40,
          "Native MCU stores and reads observe the LCD's actual command-derived status");
}
}

int main() try {
  native_boot_word();
  firmware_controls_reset();
  native_display_status();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
