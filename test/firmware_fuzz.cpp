// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, const std::size_t size) {
  try {
    const std::vector<std::uint8_t> bytes(data, data + size);
    static_cast<void>(blofeld::Firmware::parse(bytes));
  } catch (const blofeld::Fault &) {
    // Malformed files are expected rejections. Other failures remain visible to libFuzzer.
    return 0;
  }
  return 0;
}
