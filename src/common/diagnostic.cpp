// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"

#include <cstdint>
#include <iomanip>
#include <ios>
#include <sstream>
#include <string>

namespace blofeld {
std::string hex(const std::uint64_t value, const unsigned width) {
  std::ostringstream output;
  output << "0x" << std::hex << std::setfill('0') << std::setw(static_cast<int>(width)) << value;
  return output.str();
}

}
