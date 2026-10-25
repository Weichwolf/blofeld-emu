// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "display/st7565.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, const std::size_t size) {
  constexpr std::size_t stride = 3;
  constexpr std::size_t limit = 2048;
  blofeld::St7565 lcd;
  const auto count = std::min(size / stride, limit);
  for (std::size_t index = 0; index < count; ++index) {
    const auto record = std::span(data + (index * stride), stride);
    try {
      switch (record[0] % 6) {
        case 0: lcd.hardware_reset(); break;
        case 1: lcd.command(record[1]); break;
        case 2: lcd.write_data(record[1]); break;
        case 3: static_cast<void>(lcd.read_data()); break;
        case 4: static_cast<void>(lcd.ram(record[1], record[2])); break;
        case 5: static_cast<void>(lcd.state()); break;
        default: throw blofeld::Fault("Invalid LCD fuzzer operation");
      }
    } catch (const blofeld::Fault &) {
      // Qualification failures are expected; keep exploring the surviving state.
      continue;
    }
  }
  return 0;
}
