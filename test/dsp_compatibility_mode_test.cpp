// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_compatibility_mode.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>

namespace {
using namespace blofeld;

void require(const bool condition) {
  if (!condition) { throw Fault("SC retirement/settling contract failed"); }
}

void settling() {
  DspCompatibilityMode mode;
  require(mode.sample(0, false) == std::optional(false));
  require(!mode.sample(0, true));
  require(mode.retire(10, true));
  require(!mode.sample(10, true));
  require(!mode.sample(11, true));
  require(!mode.sample(12, true));
  require(mode.sample(13, true) == std::optional(true));
  require(!mode.sample(13, false));
  // Rewriting the same SC value is not a mode change.
  require(mode.retire(11, true));
  require(mode.sample(13, true) == std::optional(true));
  constexpr unsigned disabled_ready = 17;
  require(mode.retire(14, false));
  require(!mode.sample(14, false));
  require(!mode.sample(16, false));
  require(mode.sample(disabled_ready, false) == std::optional(false));
}

void histories() {
  DspCompatibilityMode mode;
  require(mode.retire(10, true));
  const auto pending = mode;
  require(mode.retire(11, false));
  require(!mode.sample(13, false));
  require(mode.sample(14, false) == std::optional(false));
  mode = pending;
  require(!mode.sample(12, true));
  require(mode.sample(13, true) == std::optional(true));
  require(!mode.retire(9, false));
  require(mode == pending);
  require(!mode.sample(9, true));
  constexpr auto last = std::numeric_limits<std::uint64_t>::max();
  require(mode.retire(last - 2, false));
  require(!mode.sample(last, false));
  require(!mode.sample(0, false));
  mode = {};
  require(mode.sample(0, false) == std::optional(false));
}
}

int main() {
  try {
    settling();
    histories();
    std::cout << "PASS SC_MODE settling histories clock-boundaries\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
