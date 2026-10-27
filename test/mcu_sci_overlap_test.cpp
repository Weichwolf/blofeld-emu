// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_sci.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
namespace reg = blofeld::hc12;
constexpr unsigned divisor = 100, receive_enable = 4, full_flag = 0x20, error_flags = 15;
constexpr unsigned noise_flag = 4, first_word = 0, byte_mask = 0xff, ninth_mask = 0x80;
enum class Width : std::uint8_t { eight = 8, nine = 9 };

void require(const bool valid, const char *message) {
  if (!valid) { throw blofeld::Fault(message); }
}

void tick(blofeld::McuSci &sci, const bool level, const unsigned samples = 1) {
  sci.advance({.bus_cycles = samples * divisor, .receive_level = level});
}

void data(blofeld::McuSci &sci, const unsigned word, const Width width = Width::eight) {
  for (unsigned bit = 0; bit < static_cast<unsigned>(width); ++bit) {
    tick(sci, ((word >> bit) & 1U) != 0, 16);
  }
}

struct ExpectedWord {
  unsigned value = 0;
  unsigned errors = 0;
};

void word(blofeld::McuSci &sci, const ExpectedWord expected) {
  const auto status = sci.read(reg::sci_status1);
  require((status & (full_flag | error_flags)) == (full_flag | expected.errors),
          "Literal start/stop sample pattern must produce exactly its expected flags");
  require(sci.read(reg::sci_data_high) == ((expected.value >> 1U) & ninth_mask),
          "The independently supplied ninth bit reaches R8");
  require(sci.read(reg::sci_data_low) == (expected.value & byte_mask),
          "Literal external data byte must be retained");
}

void unresolved(blofeld::McuSci &sci) {
  try {
    static_cast<void>(sci.read(reg::sci_status1));
  } catch (const blofeld::Fault &error) {
    require(std::string(error.what()).contains("Unverified SCI stop-to-data"),
            "The original flag uncertainty must remain explicit");
    return;
  }
  throw blofeld::Fault("Capturing a following start must not invent the earlier flag edge");
}

struct Pattern {
  unsigned stop_samples = 0;
  bool noisy_start = false;
  unsigned value = 0;
  Width width = Width::eight;
};

void overlap(const Pattern pattern) {
  const auto [stop_samples, noisy_start, value, width] = pattern;
  blofeld::McuSci sci;
  sci.write({.address = reg::sci_control1,
             .value = static_cast<std::uint8_t>(width == Width::nine ? 16U : 0U)});
  sci.write({.address = reg::sci_baud_low, .value = divisor});
  sci.write({.address = reg::sci_control2, .value = receive_enable});
  tick(sci, true, 3);
  tick(sci, false, 16);
  data(sci, first_word, width);
  tick(sci, true, stop_samples);
  const auto prefix = 16 - stop_samples;
  std::optional<blofeld::McuSci> copy;
  if (prefix == 1) {
    copy = sci;
    unresolved(*copy);
  }
  for (unsigned sample = 1; sample <= 16; ++sample) {
    const bool level = noisy_start && sample == 3;
    tick(sci, level);
    if (copy) { tick(*copy, level); }
    if (sample == prefix - 1) {
      copy = sci;
      unresolved(*copy);
    }
    if (sample == prefix) {
      const auto old_errors = stop_samples == 9 ? noise_flag : 0;
      word(sci, {.value = first_word, .errors = old_errors});
      if (!copy) { throw blofeld::Fault("A real partial start checkpoint must exist"); }
      word(*copy, {.value = first_word, .errors = old_errors});
    }
  }
  if (!copy) { throw blofeld::Fault("Missing complete partial-start checkpoint"); }
  data(sci, value, width);
  data(*copy, value, width);
  tick(sci, true, 16);
  tick(*copy, true, 16);
  const auto errors = noisy_start ? noise_flag : 0;
  word(sci, {.value = value, .errors = errors});
  word(*copy, {.value = value, .errors = errors});
}

blofeld::McuSci partial_start() {
  blofeld::McuSci sci;
  sci.write({.address = reg::sci_baud_low, .value = divisor});
  sci.write({.address = reg::sci_control2, .value = receive_enable});
  tick(sci, true, 3);
  tick(sci, false, 16);
  data(sci, first_word);
  tick(sci, true, 10);
  tick(sci, false, 5);
  unresolved(sci);
  return sci;
}

void cancel_partial_start() {
  for (const bool reset : {false, true}) {
    auto sci = partial_start();
    if (reset) {
      sci.reset();
    } else {
      sci.write({.address = reg::sci_control2, .value = 0});
    }
    sci.write({.address = reg::sci_baud_low, .value = divisor});
    sci.write({.address = reg::sci_control2, .value = receive_enable});
    tick(sci, true, 3);
    tick(sci, false, 16);
    data(sci, byte_mask);
    tick(sci, true, 16);
    word(sci, {.value = byte_mask});
    require((sci.read(reg::sci_status1) & full_flag) == 0,
            "Reset and RE disable must discard the interrupted frame and next start");
  }
}
}

int main() {
  try {
    cancel_partial_start();
    unsigned cases = 0;
    for (const auto width : {Width::eight, Width::nine}) {
      const auto words = 1U << static_cast<unsigned>(width);
      for (unsigned stop = 9; stop <= 15; ++stop) {
        for (const bool noisy : {false, true}) {
          for (unsigned value = 0; value < words; ++value) {
            overlap({.stop_samples = stop, .noisy_start = noisy, .value = value, .width = width});
            ++cases;
          }
        }
      }
    }
    std::cout << "Literal eight/nine-bit stop/start overlap cases=" << cases
              << " partial-copy=retained unknown-transfer=refused\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
