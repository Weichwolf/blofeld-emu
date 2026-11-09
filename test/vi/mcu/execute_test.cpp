// SPDX-License-Identifier: GPL-3.0-only
#include "display/st7565.hpp"
#include "display/state.hpp"
#include "execution.hpp"
#include "io/controls.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "isa.hpp"
#include "mcu/execute.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <future>
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
  blofeld::vi::DisplayState display;
  const auto report = blofeld::vi::mcu::execute(*image, shi, reset, {}, nullptr, &display);
  constexpr cpu12::Address marker = 0x3000;
  require(report.failure && report.failure->address == marker && report.failure->value == 0x40,
          "Native MCU stores and reads observe the LCD's actual command-derived status");
  const auto snapshot = display.take();
  require(snapshot && snapshot->state().display_on && snapshot->ram(0, 0).known == 0,
          "The final owner snapshot retains the actual LCD command and unknown RAM");
}

void native_package_and_peer() {
  namespace isa = cpu12::isa;
  constexpr cpu12::Byte datum = 0xa5;
  constexpr cpu12::Address marker = 0x3000;
  constexpr cpu12::Byte peer_byte_mask = 0xff;
  constexpr std::array<cpu12::Byte, 2> input_address{2, 0x60};
  constexpr std::array<cpu12::Byte, 2> marker_address{static_cast<cpu12::Byte>(marker >> 8U),
                                                      static_cast<cpu12::Byte>(marker)};
  std::vector<cpu12::Byte> program;
  const auto append = [&](const auto &bytes) {
    program.insert(program.end(), bytes.begin(), bytes.end());
  };
  const auto store = [&](const cpu12::Byte value, const cpu12::Byte address) {
    append(std::array{isa::opcode<isa::Opcode::ldab_immediate>()[0],
                      value,
                      isa::opcode<isa::Opcode::stab_direct>()[0],
                      address});
  };
  store(4, 9);
  store(4, 8);
  append(
      isa::program(isa::opcode<isa::Opcode::brclr_direct>(), std::array<cpu12::Byte, 3>{8, 8, 0}));
  const auto first = program.size() - 1;
  store(0, 8);
  append(
      isa::program(isa::opcode<isa::Opcode::brset_direct>(), std::array<cpu12::Byte, 3>{8, 8, 0}));
  const auto second = program.size() - 1;
  constexpr auto select_64 = isa::program(isa::opcode<isa::Opcode::ldab_immediate>(),
                                          std::array<cpu12::Byte, 1>{3},
                                          isa::opcode<isa::Opcode::stab_extended>(),
                                          std::array<cpu12::Byte, 2>{2, 0x57});
  append(isa::program(isa::opcode<isa::Opcode::bra_relative>(),
                      std::array{static_cast<cpu12::Byte>(select_64.size())}));
  program[first] = static_cast<cpu12::Byte>(program.size() - first - 1);
  program[second] = static_cast<cpu12::Byte>(program.size() - second - 1);
  append(select_64);
  append(isa::program(isa::opcode<isa::Opcode::ldab_extended>(),
                      input_address,
                      isa::opcode<isa::Opcode::stab_extended>(),
                      marker_address));
  auto image = std::make_unique<blofeld::vi::FlashImage>();
  image->known.fill(1);
  std::ranges::copy(program, image->bytes.begin());
  image->bytes[blofeld::vi::FlashImage::byte_count - 2] = 0x80;
  image->bytes.back() = 0;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::PeerPort peer;
  peer.dsp_data({.mask = peer_byte_mask, .known = peer_byte_mask, .value = datum});
  const auto report = blofeld::vi::mcu::execute(*image, shi, reset, {}, nullptr, nullptr, &peer);
  require(report.failure && report.failure->address == marker && report.failure->value == datum,
          "Actual package-sense branches select 100-pin routing and PTQ samples the DSP driver");
}

void native_peer_xirq() {
  namespace isa = cpu12::isa;
  constexpr cpu12::Byte datum = 0xb2;
  constexpr cpu12::Byte peer_byte_mask = 0xff;
  constexpr cpu12::Address marker = 0x3000;
  constexpr std::array<cpu12::Byte, 2> input_address{2, 0x60};
  constexpr std::array<cpu12::Byte, 2> marker_address{static_cast<cpu12::Byte>(marker >> 8U),
                                                      static_cast<cpu12::Byte>(marker)};
  constexpr std::uint16_t stack = 0x1fff;
  constexpr auto setup =
      isa::program(isa::opcode<isa::Opcode::lds_immediate>(),
                   std::array<cpu12::Byte, 2>{static_cast<cpu12::Byte>(stack >> 8U),
                                              static_cast<cpu12::Byte>(stack)},
                   isa::opcode<isa::Opcode::andcc_immediate>(),
                   std::array<cpu12::Byte, 1>{0xbf},
                   isa::wai);
  constexpr auto handler = isa::program(isa::opcode<isa::Opcode::ldab_extended>(),
                                        input_address,
                                        isa::opcode<isa::Opcode::stab_extended>(),
                                        marker_address);
  auto image = std::make_unique<blofeld::vi::FlashImage>();
  image->known.fill(1);
  std::ranges::copy(setup, image->bytes.begin());
  std::ranges::copy(handler, image->bytes.begin() + static_cast<std::ptrdiff_t>(setup.size()));
  constexpr auto entry = static_cast<std::uint16_t>(0x8000 + setup.size());
  image->bytes[blofeld::vi::FlashImage::byte_count - 12] = static_cast<cpu12::Byte>(entry >> 8U);
  image->bytes[blofeld::vi::FlashImage::byte_count - 11] = static_cast<cpu12::Byte>(entry);
  image->bytes[blofeld::vi::FlashImage::byte_count - 2] = 0x80;
  image->bytes.back() = 0;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::PeerPort peer;
  peer.dsp_data({.mask = peer_byte_mask, .known = peer_byte_mask, .value = datum});
  require(peer.dsp_ready(true) && peer.dsp_ready(false), "Post an actual external request pulse");
  const auto ended = blofeld::vi::mcu::execute(*image, shi, reset, {}, nullptr, nullptr, &peer);
  require(!ended.failure && ended.instructions == 3 && !peer.mcu_interrupt(),
          "An XIRQ pulse that ended before unmasking cannot enter its vector");
  require(peer.dsp_ready(true), "Keep an actual XIRQ source asserted through MCU admission");
  const auto report = blofeld::vi::mcu::execute(*image, shi, reset, {}, nullptr, nullptr, &peer);
  require(report.failure && report.failure->address == marker && report.failure->value == datum &&
              peer.mcu_interrupt() == 2 && peer.dsp_ready(false) && !peer.mcu_interrupt(),
          "Unmasking X wakes through the real XIRQ vector despite I remaining masked");
}

void native_pedal_input() {
  namespace isa = cpu12::isa;
  constexpr cpu12::Byte other_outputs = 0x37;
  constexpr auto program = isa::program(isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array{other_outputs},
                                        isa::opcode<isa::Opcode::stab_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x4a},
                                        isa::opcode<isa::Opcode::ldab_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x48},
                                        isa::opcode<isa::Opcode::stab_extended>(),
                                        std::array<cpu12::Byte, 2>{0x30, 0});
  auto image = std::make_unique<blofeld::vi::FlashImage>();
  image->known.fill(1);
  std::ranges::copy(program, image->bytes.begin());
  image->bytes[blofeld::vi::FlashImage::byte_count - 2] = 0x80;
  image->bytes.back() = 0;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::Controls controls;
  require(!controls.pedal(), "The virtual pedal starts inactive");
  for (const bool level : {false, true}) {
    controls.pedal(level);
    const auto report = blofeld::vi::mcu::execute(*image, shi, reset, {}, &controls);
    constexpr cpu12::Address marker = 0x3000;
    require(report.failure && report.failure->address == marker &&
                report.failure->value == (level ? 8U : 0U),
            "Native MCU reads sample the external pedal while retaining raw 0/8 levels");
  }
}

void native_local_port_access() {
  namespace isa = cpu12::isa;
  constexpr cpu12::Byte datum = 0x90;
  constexpr cpu12::Address marker = 0x3000;
  constexpr auto program = isa::program(isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array<cpu12::Byte, 1>{1},
                                        isa::opcode<isa::Opcode::stab_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x50},
                                        isa::opcode<isa::Opcode::clr_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x62},
                                        isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array{datum},
                                        isa::opcode<isa::Opcode::stab_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x60},
                                        isa::opcode<isa::Opcode::stab_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x61},
                                        isa::opcode<isa::Opcode::ldab_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x52},
                                        isa::opcode<isa::Opcode::ldab_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x62},
                                        isa::opcode<isa::Opcode::stab_extended>(),
                                        std::array<cpu12::Byte, 2>{0x30, 0});
  auto image = std::make_unique<blofeld::vi::FlashImage>();
  image->known.fill(1);
  std::ranges::copy(program, image->bytes.begin());
  image->bytes[blofeld::vi::FlashImage::byte_count - 2] = 0x80;
  image->bytes.back() = 0;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::PeerPort peer;
  blofeld::vi::ExecutionTime time;
  std::stop_source stop;
  auto worker = std::async(std::launch::async, [&] {
    return blofeld::vi::mcu::execute(
        *image, shi, reset, stop.get_token(), nullptr, nullptr, &peer, &time);
  });
  const auto completed = worker.wait_for(std::chrono::seconds(1));
  if (completed != std::future_status::ready) { stop.request_stop(); }
  const auto report = worker.get();
  require(completed == std::future_status::ready && report.failure &&
              report.failure->address == marker && report.failure->value == 0 &&
              peer.mcu_data().mask == 0 && peer.mcu_data().value == datum &&
              !peer.dsp_interrupt(true),
          "Local register reads, ignored writes and undriven latches need no DSP owner");
}
}

int main() try {
  native_boot_word();
  firmware_controls_reset();
  native_display_status();
  native_package_and_peer();
  native_peer_xirq();
  native_pedal_input();
  native_local_port_access();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
