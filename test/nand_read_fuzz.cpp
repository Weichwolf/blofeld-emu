// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/nand.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace {
std::optional<bool> level(const std::uint8_t value) {
  if (value % 3U == 0) { return std::nullopt; }
  return value % 3U == 2;
}
}

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, const std::size_t size) {
  constexpr std::size_t stride = 8;
  constexpr std::size_t operation_limit = 1024;
  constexpr unsigned actions = 16;
  constexpr unsigned reset_command = 0xff;
  blofeld::Nand device;
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
        case 8: device.observe_wp(level(record[1])); break;
        case 9:
          device.observe_block((static_cast<unsigned>(record[1]) << 8U) | record[2],
                               {.valid = level(record[3]),
                                .program_protected = level(record[4]),
                                .erase_protected = level(record[5])});
          break;
        case 10: device.write(record[1]); break;
        case 11:
          device.observe_completion(static_cast<blofeld::NandOutcome>(record[1] % 3U));
          break;
        case 12: device.power_loss(); break;
        case 13: static_cast<void>(device.read(record[1])); break;
        case 14:
          device.observe_cell({.page = (static_cast<unsigned>(record[1]) << 16U) |
                                       (static_cast<unsigned>(record[2]) << 8U) | record[3],
                               .column = (static_cast<unsigned>(record[4]) << 8U) | record[5]},
                              {.value = record[6], .known = record[7]});
          break;
        case 15:
          static_cast<void>(device.pending());
          static_cast<void>(device.status());
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
