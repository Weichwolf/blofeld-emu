// SPDX-License-Identifier: GPL-3.0-only
#include "display/port.hpp"
#include "io/shi.hpp"
#include "mcu/ports.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

constexpr std::uint8_t all_bits = 0xff, outputs = 0xe1;
constexpr std::uint8_t write_control = 0x40, read_control = 0x60, enable = 0x10;
constexpr std::uint8_t page_two = 0xb2, pattern = 0xa5, display_on = 0xaf, adc_reverse = 0xa1;

struct Bus {
  blofeld::vi::Shi shi;
  blofeld::vi::mcu::Ports ports{shi};
  blofeld::vi::DisplayPort display;

  void put(const blofeld::vi::mcu::Ports::Write access) {
    require(ports.write(access) && display.write(ports, access), "Decode an actual port store");
  }

  void send(const std::uint8_t value, const bool data = false) {
    put({.offset = 1, .value = static_cast<std::uint8_t>(write_control | (data ? 1U : 0U))});
    put({.offset = 2, .value = all_bits});
    put({.offset = 0, .value = value});
    put({.offset = 8, .value = enable});
    put({.offset = 8});
  }

  void begin_read(const bool data) {
    put({.offset = 2});
    put({.offset = 1, .value = static_cast<std::uint8_t>(read_control | (data ? 1U : 0U))});
    put({.offset = 8, .value = enable});
  }

  [[nodiscard]] std::uint8_t sample() const {
    std::uint8_t value = 0;
    require(display.read(value, all_bits), "Read a known controller byte");
    return value;
  }
};

void status_and_data() {
  Bus bus;
  bus.put({.offset = 3, .value = outputs});
  bus.begin_read(false);
  require(bus.sample() == read_control, "Reset status: normal ADC, display off, ready");
  bus.put({.offset = 8});
  bus.send(display_on);
  bus.send(adc_reverse);
  bus.begin_read(false);
  require(bus.sample() == 0 && bus.sample() == 0,
          "Status reflects real display/ADC commands without advancing RAM");
  bus.put({.offset = 8});
  bus.send(page_two);
  bus.send(3);
  bus.send(pattern, true);
  bus.send(3);
  bus.begin_read(true);
  require(bus.sample() == 3 && bus.sample() == 3, "Dummy byte is the retained command holder");
  bus.put({.offset = 8});
  bus.begin_read(true);
  require(bus.sample() == pattern && bus.sample() == pattern,
          "Repeated Port A reads retain the same real RAM byte");
  bus.put({.offset = 8});
  require(bus.display.controller().state().column == 4,
          "Only completing the transaction advances the RAM read cursor once");
  bus.put({.offset = 8});
  require(bus.display.controller().state().column == 4, "Duplicate inactive writes do not advance");
  bus.begin_read(true);
  std::uint8_t value = 0;
  require(bus.display.reading() && !bus.display.read(value, all_bits),
          "Unwritten display RAM owns the read but remains unknown");
}

void rejected_write() {
  Bus bus;
  bus.put({.offset = 3, .value = outputs});
  bus.put({.offset = 1, .value = write_control});
  bus.put({.offset = 8, .value = enable});
  require(bus.ports.write({.offset = 8}) && !bus.display.write(bus.ports, {.offset = 8}) &&
              bus.display.error() != nullptr,
          "No MCU data output means no invented display command");
}
}

int main() try {
  status_and_data();
  rejected_write();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
