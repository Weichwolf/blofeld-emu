// SPDX-License-Identifier: GPL-3.0-only
#include "display/st7565.hpp"
#include "display/state.hpp"
#include "execution.hpp"
#include "io/controls.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
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
#include <limits>
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
  blofeld::vi::ExecutionTime lifecycle;
  const auto ended =
      blofeld::vi::mcu::execute(*image, shi, reset, {}, nullptr, nullptr, &peer, &lifecycle);
  require(lifecycle.mcu_wait_time().has_value(),
          "Only the actual native WAI publishes the initial MCU wait observation");
  require(!ended.failure && ended.instructions == 3 && !peer.mcu_interrupt(),
          "An XIRQ pulse that ended before unmasking cannot enter its vector");
  require(peer.dsp_ready(true), "Keep an actual XIRQ source asserted through MCU admission");
  const auto report = blofeld::vi::mcu::execute(*image, shi, reset, {}, nullptr, nullptr, &peer);
  require(report.failure && report.failure->address == marker && report.failure->value == datum &&
              peer.mcu_interrupt() == 2 && peer.dsp_ready(false) && !peer.mcu_interrupt(),
          "Unmasking X wakes through the real XIRQ vector despite I remaining masked");
  blofeld::vi::PeerPort idle;
  require(idle.dsp_ready(true), "A real notification can accompany a released data bus");
  const auto released = blofeld::vi::mcu::execute(*image, shi, reset, {}, nullptr, nullptr, &idle);
  require(released.failure && released.failure->address == marker && released.failure->value == 0,
          "Native MCU input reads the attached DSP's documented released-bus bias");
  blofeld::vi::PeerPort unknown;
  unknown.dsp_data({.mask = peer_byte_mask});
  require(unknown.dsp_ready(true), "An active unknown driver accompanies a genuine notification");
  const auto rejected =
      blofeld::vi::mcu::execute(*image, shi, reset, {}, nullptr, nullptr, &unknown);
  require(rejected.failure && rejected.failure->operation == blofeld::vi::Access::read &&
              rejected.failure->address == ((unsigned{input_address[0]} << 8U) | input_address[1]),
          "The native binding cannot convert actively unknown peer bytes into idle zeros");
  blofeld::vi::PeerPort announced;
  constexpr cpu12::Byte command = 0x0d;
  announced.dsp_data({.mask = peer_byte_mask, .known = peer_byte_mask, .value = command});
  require(announced.dsp_ready(true) && announced.dsp_ready(false),
          "Publish a real report command before the MCU owner starts");
  const auto pending =
      blofeld::vi::mcu::execute(*image, shi, reset, {}, nullptr, nullptr, &announced);
  require(pending.failure && pending.failure->address == marker &&
              pending.failure->value == command && announced.mcu_interrupt() == 1,
          "A still-driven report enters the original XIRQ vector after MCU unmasking");
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
                                        isa::opcode<isa::Opcode::stab_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x52},
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
  // libc++ mutates the source on request_stop(); libstdc++ accepts a const source.
#if defined(_LIBCPP_VERSION)
  std::stop_source stop;
#else
  const std::stop_source stop;
#endif
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

void native_queued_data_without_dsp() {
  namespace isa = cpu12::isa;
  constexpr cpu12::Byte first = 0x12;
  constexpr cpu12::Byte last = 0x34;
  constexpr cpu12::Byte outputs = 0xff;
  constexpr cpu12::Address marker = 0x3000;
  constexpr auto program = isa::program(isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array{outputs},
                                        isa::opcode<isa::Opcode::stab_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x62},
                                        isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array{first},
                                        isa::opcode<isa::Opcode::stab_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x60},
                                        isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array{last},
                                        isa::opcode<isa::Opcode::stab_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x60},
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
  std::promise<blofeld::vi::ExecutionReport> result;
  auto completed = result.get_future();
  std::jthread mcu([&](const std::stop_token &stop) {
    result.set_value(
        blofeld::vi::mcu::execute(*image, shi, reset, stop, nullptr, nullptr, &peer, &time));
  });
  const auto ready = completed.wait_for(std::chrono::seconds(1));
  mcu.request_stop();
  mcu.join();
  const auto report = completed.get();
  require(ready == std::future_status::ready && report.failure &&
              report.failure->address == marker && report.failure->value == last &&
              peer.mcu_data().value == last && peer.sample_mcu_data(0).mask == 0 &&
              peer.sample_mcu_data(std::numeric_limits<std::uint64_t>::max()) ==
                  blofeld::vi::PeerPort::Sample{.mask = outputs, .known = outputs, .value = last},
          "Native MCU data and direction writes queue real timestamped values without a DSP wait");
}

void native_polling_progress(const bool pulse) {
  namespace isa = cpu12::isa;
  constexpr cpu12::Address marker = 0x3000;
  constexpr cpu12::Byte datum = 0x5a;
  constexpr std::uint64_t peer_time = 1000;
  constexpr auto setup = isa::program(isa::opcode<isa::Opcode::lds_immediate>(),
                                      std::array<cpu12::Byte, 2>{0x1f, 0xff},
                                      isa::opcode<isa::Opcode::andcc_immediate>(),
                                      std::array<cpu12::Byte, 1>{0xbf},
                                      isa::opcode<isa::Opcode::ldab_immediate>(),
                                      std::array<cpu12::Byte, 1>{0x80},
                                      isa::opcode<isa::Opcode::stab_extended>(),
                                      std::array<cpu12::Byte, 2>{0, 0x46});
  constexpr cpu12::Byte repeat_poll = 0xfb;
  constexpr auto loop = isa::program(isa::opcode<isa::Opcode::ldab_extended>(),
                                     std::array<cpu12::Byte, 2>{0, 0x44},
                                     isa::opcode<isa::Opcode::bra_relative>(),
                                     std::array{repeat_poll});
  constexpr auto handler = isa::program(isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array{datum},
                                        isa::opcode<isa::Opcode::stab_extended>(),
                                        std::array<cpu12::Byte, 2>{0x30, 0});
  auto image = std::make_unique<blofeld::vi::FlashImage>();
  image->known.fill(1);
  std::ranges::copy(setup, image->bytes.begin());
  std::ranges::copy(loop, image->bytes.begin() + static_cast<std::ptrdiff_t>(setup.size()));
  const auto offset = setup.size() + loop.size();
  std::ranges::copy(handler, image->bytes.begin() + static_cast<std::ptrdiff_t>(offset));
  constexpr auto entry = static_cast<cpu12::Address>(0x8000 + offset);
  image->bytes[blofeld::vi::FlashImage::byte_count - 12] = static_cast<cpu12::Byte>(entry >> 8U);
  image->bytes[blofeld::vi::FlashImage::byte_count - 11] = static_cast<cpu12::Byte>(entry);
  image->bytes[blofeld::vi::FlashImage::byte_count - 2] = 0x80;
  image->bytes.back() = 0;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::PeerPort peer;
  blofeld::vi::ExecutionTime time;
  std::promise<blofeld::vi::ExecutionReport> result;
  auto completed = result.get_future();
  std::jthread mcu([&](const std::stop_token &stop) {
    try {
      result.set_value(
          blofeld::vi::mcu::execute(*image, shi, reset, stop, nullptr, nullptr, &peer, &time));
    } catch (...) { result.set_exception(std::current_exception()); }
  });
  bool posted = false;
  std::jthread dsp([&](const std::stop_token &stop) {
    std::uint64_t ticks = 0;
    while (const auto limit = time.dsp_limit(ticks, stop)) {
      ticks = *limit;
      if (ticks >= peer_time) {
        posted = peer.dsp_ready(true) && (!pulse || peer.dsp_ready(false));
        return;
      }
    }
  });
  const auto ready = completed.wait_for(std::chrono::seconds(1));
  mcu.request_stop();
  dsp.request_stop();
  mcu.join();
  dsp.join();
  const auto report = completed.get();
  require(ready == std::future_status::ready && posted && report.failure &&
              report.failure->address == marker && report.failure->value == datum,
          "A native timer-poll loop publishes device time so its peer can deliver the real XIRQ");
}

void native_panel_sampling(const bool timed_boundary) {
  namespace isa = cpu12::isa;
  constexpr cpu12::Byte phase_marker = 0x42;
  constexpr cpu12::Byte response = 0x5a;
  constexpr cpu12::Address marker = 0x3000;
  constexpr cpu12::Byte all_bits = 0xff;
  constexpr cpu12::Byte bank_outputs = 0xc0;
  constexpr cpu12::Byte bank_a = 0x40;

  enum Offset : cpu12::Byte {
    direction_s = 0x72,
    data_s = 0x70,
    data_j = 0x50,
    direction_j = 0x52,
    pull_q = 0x64,
    polarity_q = 0x65,
    data_q = 0x60,
    timer_count = 0x44
  };

  std::vector<cpu12::Byte> program;
  const auto append = [&program](const auto &bytes) {
    program.insert(program.end(), bytes.begin(), bytes.end());
  };
  const auto store = [&](const cpu12::Byte value, const cpu12::Byte offset) {
    append(std::array{isa::opcode<isa::Opcode::ldab_immediate>()[0],
                      value,
                      isa::opcode<isa::Opcode::stab_extended>()[0],
                      cpu12::Byte{2},
                      offset});
  };
  store(bank_outputs, direction_s);
  store(bank_a, data_s);
  store(1, data_j); // J0 is an inactive DSP request.
  store(1, direction_j);
  store(all_bits, pull_q);
  store(all_bits, polarity_q); // Actual MCU pull-downs define inactive panel inputs.
  store(phase_marker, data_q); // Q remains an input; this is not a DSP request.
  constexpr unsigned samples = 128;
  for (unsigned index = 0; index < samples; ++index) {
    append(isa::program(isa::opcode<isa::Opcode::ldab_extended>(),
                        std::array<cpu12::Byte, 2>{2, data_q}));
  }
  if (timed_boundary) {
    append(isa::program(isa::opcode<isa::Opcode::ldab_direct>(),
                        std::array<cpu12::Byte, 1>{timer_count},
                        isa::opcode<isa::Opcode::ldab_extended>(),
                        std::array<cpu12::Byte, 2>{2, data_q}));
  }
  append(isa::program(isa::opcode<isa::Opcode::stab_extended>(),
                      std::array<cpu12::Byte, 2>{static_cast<cpu12::Byte>(marker >> 8U), 0}));
  auto image = std::make_unique<blofeld::vi::FlashImage>();
  image->known.fill(1);
  std::ranges::copy(program, image->bytes.begin());
  image->bytes[blofeld::vi::FlashImage::byte_count - 2] = 0x80;
  image->bytes.back() = 0;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::PeerPort peer;
  blofeld::vi::ExecutionTime time;
  std::promise<blofeld::vi::ExecutionReport> result;
  auto completed = result.get_future();
  std::jthread mcu([&](const std::stop_token &stop) {
    try {
      result.set_value(
          blofeld::vi::mcu::execute(*image, shi, reset, stop, nullptr, nullptr, &peer, &time));
    } catch (...) { result.set_exception(std::current_exception()); }
  });
  bool sampled = false;
  std::jthread dsp([&](const std::stop_token &stop) {
    std::uint64_t ticks = 0;
    while (const auto limit = time.dsp_limit(ticks, stop)) {
      ticks = *limit;
      const auto input = peer.sample_mcu_data(ticks);
      if (input.mask != 0 || input.value != phase_marker) { continue; }
      sampled = true;
      if (timed_boundary) {
        peer.dsp_data({.mask = all_bits, .known = all_bits, .value = response});
      }
    }
  });
  const auto ready = completed.wait_for(std::chrono::seconds(1));
  mcu.request_stop();
  dsp.request_stop();
  mcu.join();
  dsp.join();
  const auto report = completed.get();
  const auto expected = timed_boundary ? response : cpu12::Byte{0};
  require(
      ready == std::future_status::ready && sampled && report.failure &&
          report.failure->address == marker && report.failure->value == expected,
      "Repeated input-bank reads preserve inactive inputs and actual data across a timer access");
}

void native_clock_input_frontier() {
  namespace isa = cpu12::isa;
  constexpr cpu12::Byte divider = 3;
  constexpr cpu12::Byte divider_register = 0x35;
  constexpr cpu12::Address marker = 0x3000;
  constexpr std::uint64_t initial_credit = 1000;
  constexpr std::uint64_t next_credit = 10000;
  constexpr std::uint64_t paused_at = 980;
  constexpr std::size_t nop_count = 40;
  constexpr auto padding = [] {
    std::array<cpu12::Byte, nop_count> bytes{};
    bytes.fill(isa::nop.front());
    return bytes;
  }();
  constexpr auto program =
      isa::program(isa::opcode<isa::Opcode::ldaa_immediate>(),
                   std::array{divider},
                   isa::opcode<isa::Opcode::staa_direct>(),
                   std::array{divider_register},
                   padding,
                   isa::opcode<isa::Opcode::staa_extended>(),
                   std::array<cpu12::Byte, 2>{static_cast<cpu12::Byte>(marker >> 8U), 0});
  auto image = std::make_unique<blofeld::vi::FlashImage>();
  image->known.fill(1);
  std::ranges::copy(program, image->bytes.begin());
  image->bytes[blofeld::vi::FlashImage::byte_count - 2] = 0x80;
  image->bytes.back() = 0;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::MidiPort midi(blofeld::vi::MidiPort::InputMode::stream);
  require(midi.advance_input(initial_credit), "Publish actual empty-input execution credit");
  std::promise<blofeld::vi::ExecutionReport> completion;
  auto completed = completion.get_future();
  std::jthread mcu([&](const std::stop_token &stop) {
    try {
      completion.set_value(blofeld::vi::mcu::execute(
          *image, shi, reset, stop, nullptr, nullptr, nullptr, nullptr, &midi));
    } catch (...) { completion.set_exception(std::current_exception()); }
  });
  const auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (midi.time() < paused_at &&
         completed.wait_for(std::chrono::seconds(0)) != std::future_status::ready &&
         std::chrono::steady_clock::now() < timeout) {
    std::this_thread::yield();
  }
  const bool paused = midi.time() == paused_at &&
                      completed.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
  if (!paused) {
    mcu.request_stop();
    mcu.join();
    static_cast<void>(completed.get());
    throw std::runtime_error(
        "A bus-divider change must reproject the exclusive input frontier before later execution");
  }
  require(midi.advance_input(next_credit), "Release new input credit without fabricating a byte");
  const auto ready = completed.wait_for(std::chrono::seconds(2));
  mcu.request_stop();
  mcu.join();
  const auto report = completed.get();
  require(ready == std::future_status::ready && report.failure &&
              report.failure->address == marker && report.failure->value == divider,
          "The same original continuation reaches its marker only after real producer credit");
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
  native_queued_data_without_dsp();
  native_polling_progress(false);
  native_polling_progress(true);
  native_panel_sampling(false);
  native_panel_sampling(true);
  native_clock_input_frontier();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
