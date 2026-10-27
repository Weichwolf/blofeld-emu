// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_sci.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
namespace reg = blofeld::hc12;
constexpr unsigned divisor = 100, receive_enable = 4, full_flag = 0x20, error_flags = 15;
constexpr unsigned normal_period = 1600, mildly_fast = 1585, fast_period = 1550;
constexpr unsigned read_delay = 700, tail_padding = 1000;

void clocks(blofeld::McuSci &sci, const unsigned count, const bool high) {
  sci.advance({.bus_cycles = count, .receive_level = high});
}

void data(blofeld::McuSci &sci, const std::uint8_t word, const unsigned period) {
  for (unsigned bit = 0; bit < 8; ++bit) { clocks(sci, period, ((word >> bit) & 1U) != 0); }
}

bool read_word(blofeld::McuSci &sci, const std::uint8_t expected) {
  const auto status = sci.read(reg::sci_status1);
  if ((status & full_flag) == 0) { return false; }
  const auto actual = sci.read(reg::sci_data_low);
  if ((status & error_flags) != 0 || actual != expected) {
    throw blofeld::Fault("Independent clean wire byte was corrupted");
  }
  return true;
}

bool pair(const unsigned period, const std::uint8_t word) {
  blofeld::McuSci sci;
  sci.write({.address = reg::sci_baud_low, .value = divisor});
  sci.write({.address = reg::sci_control2, .value = receive_enable});
  clocks(sci, 3 * divisor, true);
  clocks(sci, period, false);
  data(sci, word, period);
  clocks(sci, period, true);
  // Begin the next start bit before polling, then read only after the first
  // receiver's complete RT16 boundary. No receive-transfer hypothesis is set.
  clocks(sci, read_delay, false);
  const auto first = read_word(sci, word);
  clocks(sci, period - read_delay, false);
  data(sci, word, period);
  clocks(sci, period + tail_padding, true);
  const auto second = read_word(sci, word);
  std::cout << "period=" << period << " word=" << static_cast<unsigned>(word) << " first=" << first
            << " second=" << second << '\n';
  return first && second;
}
}

int main() {
  try {
    bool complete = true;
    constexpr std::array<std::uint8_t, 2> words{0, 0xff};
    for (const auto period : {normal_period, mildly_fast, fast_period}) {
      for (const auto word : words) { complete = pair(period, word) && complete; }
    }
    return complete ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
