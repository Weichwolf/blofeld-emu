// SPDX-License-Identifier: GPL-3.0-only
#include "io/shi.hpp"
#include "mcu/boot_port.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
constexpr std::uint32_t control = 0xffff91, clock_register = 0xffff90, receive = 0xffff94;
constexpr std::uint32_t fifo24 = 0x29, datum = 0xa591e7;
constexpr std::uint8_t all_outputs = 0xff, data_bit = 0x10;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

bool drive(blofeld::vi::mcu::BootPort &port, const std::uint8_t value) {
  return port.write({.direction = all_outputs, .known = all_outputs, .value = value});
}

void reset_partial_word() {
  blofeld::vi::Shi shi;
  blofeld::vi::mcu::BootPort port(shi);
  require(shi.write({.address = control, .value = fifo24}), "Configure reset-phase SPI");
  require(drive(port, 3) && drive(port, 1) && drive(port, 5) && drive(port, 1),
          "Start a partial word before reset");
  port.reset();
  shi.reset();
  require(shi.write({.address = control, .value = fifo24}) && drive(port, 3) && drive(port, 1),
          "Start a new word after resetting both owners");
  constexpr unsigned bits = 24;
  for (unsigned bit = bits; bit != 0; --bit) {
    const auto data = static_cast<std::uint8_t>(1 | (((datum >> (bit - 1)) & 1) * data_bit));
    require(drive(port, data) && drive(port, static_cast<std::uint8_t>(data | 4)) &&
                drive(port, data),
            "Capture the new word on trailing edges");
  }
  require(port.pending() && port.complete({}) && !port.pending(),
          "Submit the complete word outside native callbacks");
  std::uint32_t word = 0;
  require(shi.read(receive, word) && word == datum && !shi.read(receive, word),
          "A discarded partial word cannot contaminate the next boot transfer");
}

void delayed_receive_mode() {
  blofeld::vi::Shi shi;
  blofeld::vi::mcu::BootPort port(shi);
  require(drive(port, 3) && drive(port, 1) && port.pending(),
          "A selected transaction awaits the DSP's actual receive configuration");
  bool completed = false;
  std::jthread mcu([&](const std::stop_token &stop) { completed = port.complete(stop); });
  require(shi.write({.address = control, .value = fifo24}), "The DSP enables reception itself");
  mcu.join();
  require(completed && !port.pending(), "Configuration wakes the blocked transaction owner");
  constexpr unsigned bits = 24;
  for (unsigned bit = bits; bit != 0; --bit) {
    const auto data = static_cast<std::uint8_t>(1 | (((datum >> (bit - 1)) & 1) * data_bit));
    require(drive(port, data) && drive(port, static_cast<std::uint8_t>(data | 4)) &&
                drive(port, data),
            "The original first word is captured after configuration");
  }
  require(port.pending() && port.complete({}) && !port.pending(),
          "Submit the complete word outside native callbacks");
  std::uint32_t word = 0;
  require(shi.read(receive, word) && word == datum && !shi.read(receive, word),
          "A slower DSP host thread cannot silently drop the first word");
}

void cancel_receive_mode() {
  blofeld::vi::Shi shi;
  blofeld::vi::mcu::BootPort port(shi);
  require(drive(port, 3) && drive(port, 1) && port.pending(), "Await a disabled receiver");
  bool completed = true;
  std::jthread mcu([&](const std::stop_token &stop) { completed = port.complete(stop); });
  mcu.request_stop();
  mcu.join();
  require(!completed && port.pending(), "Shutdown cancels readiness without inventing a mode");
}

void bounded_word_admission() {
  blofeld::vi::Shi shi;
  blofeld::vi::mcu::BootPort port(shi);
  require(shi.write({.address = control, .value = fifo24}), "Configure the bounded receiver");
  static_cast<void>(shi.select(true));
  constexpr unsigned capacity = 10;
  for (unsigned index = 0; index < capacity; ++index) {
    require(shi.transfer(index).accepted, "Fill every actual FIFO slot");
  }
  require(drive(port, 3) && drive(port, 1), "Select the next bootstrap word");
  constexpr unsigned bits = 24;
  for (unsigned bit = bits; bit != 0; --bit) {
    const auto data = static_cast<std::uint8_t>(1 | (((datum >> (bit - 1)) & 1) * data_bit));
    require(drive(port, data) && drive(port, static_cast<std::uint8_t>(data | 4)) &&
                drive(port, data),
            "Collect one complete word without modifying a full shared FIFO");
  }
  // libc++ mutates the source on request_stop(); libstdc++ accepts a const source.
#if defined(_LIBCPP_VERSION)
  std::stop_source stopped;
#else
  const std::stop_source stopped;
#endif
  stopped.request_stop();
  require(port.pending() && !port.complete(stopped.get_token()) && port.pending(),
          "A canceled submission retains pending ownership and never reports completion");
  std::uint32_t word = 0;
  require(shi.read(receive, word) && word == 0 && port.complete({}) && !port.pending(),
          "A real DSP read releases capacity for the pending word");
  for (unsigned index = 1; index < capacity; ++index) {
    require(shi.read(receive, word) && word == index, "Existing FIFO words keep their order");
  }
  require(shi.read(receive, word) && word == datum && !shi.read(receive, word),
          "The pending word arrives exactly once after existing data");
  constexpr std::uint32_t overrun = 0x100000;
  require(shi.read(control, word) && (word & overrun) == 0,
          "VM backpressure does not masquerade as a device FIFO overrun");
}

void modes(const unsigned mode) {
  blofeld::vi::Shi shi;
  blofeld::vi::mcu::BootPort port(shi);
  require(shi.write({.address = control, .value = fifo24}) &&
              shi.write({.address = clock_register, .value = mode}),
          "Configure SPI mode");
  const std::uint8_t idle = (mode & 2) != 0 ? 4 : 0;
  const auto inactive = static_cast<std::uint8_t>(idle | 3);
  require(drive(port, inactive), "Establish deselected output");
  require(drive(port, static_cast<std::uint8_t>(idle | 1)), "Select a word");
  constexpr unsigned bits = 24;
  for (unsigned bit = bits; bit != 0; --bit) {
    const auto data = static_cast<std::uint8_t>(1 | (((datum >> (bit - 1)) & 1) * data_bit));
    require(drive(port, static_cast<std::uint8_t>(data | idle)), "Place data before capture");
    require(drive(port, static_cast<std::uint8_t>(data | (idle ^ 4))), "Clock leading edge");
    if (port.pending()) {
      require(port.complete({}), "Admit a word captured on its final leading edge");
    }
    require(drive(port, static_cast<std::uint8_t>(data | idle)), "Clock trailing edge");
  }
  require(port.complete({}) && !port.pending(),
          "Complete final trailing-edge admission outside native callbacks");
  std::uint32_t word = 0;
  require(shi.read(receive, word) && word == datum, "All four modes receive MSB-first data");
  require(!shi.read(receive, word), "Only one complete word was published");
  require(drive(port, inactive), "Deselect the completed word");
  require(drive(port, static_cast<std::uint8_t>(idle | 1)), "Start another word");
  require(drive(port, static_cast<std::uint8_t>((idle ^ 4) | 1)), "Supply one partial clock");
  require(drive(port, inactive) && !shi.read(receive, word),
          "Abort never publishes a partial word");
}
}

int main() try {
  for (unsigned mode = 0; mode < 4; ++mode) { modes(mode); }
  reset_partial_word();
  delayed_receive_mode();
  cancel_receive_mode();
  bounded_word_admission();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
