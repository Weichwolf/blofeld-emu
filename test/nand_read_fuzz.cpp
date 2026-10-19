// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/nand_read.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, const std::size_t size) {
  constexpr std::size_t stride = 8;
  constexpr std::size_t operation_limit = 512;
  constexpr unsigned actions = 8;
  constexpr unsigned reset_command = 0xff;
  blofeld::NandRead device;
  const auto count = size / stride < operation_limit ? size / stride : operation_limit;
  for (std::size_t index = 0; index < count; ++index) {
    const auto *record = data + (index * stride);
    try {
      switch (record[0] % actions) {
        case 0: device.command(record[1]); break;
        case 1: device.address(record[1]); break;
        case 2: device.observe_completion(); break;
        case 3: device.observe_power_ready(); break;
        case 4: static_cast<void>(device.read()); break;
        case 5: {
          const auto page = (static_cast<unsigned>(record[1]) << 16U) |
                            (static_cast<unsigned>(record[2]) << 8U) | record[3];
          const auto column = (static_cast<unsigned>(record[4]) << 8U) | record[5];
          const std::array<std::uint8_t, 2> bytes{record[6], record[7]};
          device.observe_bytes(page, column, bytes);
          break;
        }
        case 6: device.command(reset_command); break;
        case 7:
          static_cast<void>(device.ready());
          static_cast<void>(device.page());
          static_cast<void>(device.column());
          break;
        default: throw blofeld::Fault("Invalid NAND fuzzer action");
      }
    } catch (const blofeld::Fault &) {
      // Invalid/unknown protocol states must reject; continue to exercise reset/retry.
      continue;
    }
  }
  return 0;
}
