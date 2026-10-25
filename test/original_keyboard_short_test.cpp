// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "keyboard_short_fixture.hpp"

#include <array>
#include <exception>
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>

namespace {
using namespace blofeld::test::keyboard_short;
using blofeld::test::require;

std::array<unsigned, 3> input(const std::string &path) {
  std::ifstream stream(path, std::ios::binary);
  std::array<unsigned, 3> result{};
  for (auto &value : result) {
    const auto byte = stream.get();
    require(byte != std::char_traits<char>::eof(), "Missing original short-message byte");
    value = static_cast<unsigned>(byte);
  }
  require(result == std::array{raw_start, raw_data, raw_end} &&
              stream.get() == std::char_traits<char>::eof(),
          "Pinned original Keyboard short-message framing differs");
  return result;
}

void matrix(const blofeld::Firmware &firmware,
            const std::array<unsigned, 3> message,
            std::ostream &trace) {
  for (unsigned route = 0; route < rings.size(); ++route) {
    Fixture fixture(firmware, route);
    for (const auto u : values) {
      for (const auto v : values) {
        for (unsigned copy = 0; copy < 2; ++copy) {
          fixture.values(u, v);
          fixture.enqueue(message);
          fixture.invoke(rings.at(route).entry);
          fixture.verify();
          trace << "CASE " << route << ' ' << copy << ' ' << u << ' ' << v << ' '
                << fixture.result() << '\n';
        }
      }
    }
    fixture.intact(firmware);
  }
}

void boundaries(const blofeld::Firmware &firmware,
                const std::array<unsigned, 3> message,
                std::ostream &trace) {
  for (unsigned route = 0; route < rings.size(); ++route) {
    for (unsigned copy = 0; copy < 2; ++copy) {
      Fixture fixture(firmware, route);
      fixture.values(values.back(), values.back());
      fixture.enqueue(message);
      fixture.blocked();
      if (route == 1) { fixture.resume_blocked(); }
      fixture.intact(firmware);
      trace << "BULK " << route << ' ' << copy << " blocked=" << route
            << " replays=" << (route == 1 ? 2 : 0) << '\n';
    }
    for (unsigned missing = 0; missing < stale_pc.size(); ++missing) {
      Fixture fixture(firmware, route, missing);
      fixture.values(values.back(), values.front());
      fixture.enqueue(message);
      fixture.recover_missing();
      fixture.intact(firmware);
      trace << "CUT " << route << ' ' << missing << " replays=2\n";
    }
  }
}
} // namespace

int main(const int parameter_count, char **arguments) {
  try {
    require(parameter_count == 4,
            "Usage: original-keyboard-short firmware.zip private.message private.trace");
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    const auto message = input(arguments[2]);
    std::ofstream trace(arguments[3]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    matrix(firmware, message, trace);
    boundaries(firmware, message, trace);
    trace << "RESULT 144 4 6 12\n";
    std::cout
        << "Original Keyboard short-message rejection, stale reads and deferred F7 verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
