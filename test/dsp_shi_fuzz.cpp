// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_shi.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, const std::size_t size) {
  constexpr std::size_t stride = 6;
  constexpr std::size_t operation_limit = 256;
  constexpr unsigned action_count = 7;
  constexpr unsigned first_register = 0xffff90;
  constexpr unsigned register_cases = 6; // Includes one unsupported register.
  blofeld::DspShi shi;
  const auto operations = size / stride < operation_limit ? size / stride : operation_limit;
  for (std::size_t index = 0; index < operations; ++index) {
    const auto *record = data + (index * stride);
    const auto address = first_register + (record[1] % register_cases);
    const auto value = (static_cast<std::uint32_t>(record[2]) << 16U) |
                       (static_cast<std::uint32_t>(record[3]) << 8U) | record[4];
    try {
      switch (record[0] % action_count) {
        case 0: shi.write({.address = address, .value = value}); break;
        case 1: static_cast<void>(shi.read(address)); break;
        case 2: {
          std::optional<bool> mosi;
          if ((record[2] & 1U) != 0) { mosi = (record[3] & 1U) != 0; }
          shi.observe_spi(
              {.selected = (record[1] & 1U) != 0, .clock = (record[1] & 2U) != 0, .mosi = mosi});
          break;
        }
        case 3: shi.advance_instruction_cycles(value); break;
        case 4: static_cast<void>(shi.outputs()); break;
        case 5: static_cast<void>(shi.interrupt_vectors()); break;
        case 6: shi.reset(); break;
        default: throw blofeld::Fault("Invalid SHI model-fuzzer action");
      }
    } catch (const blofeld::Fault &) {
      // Undefined reads, unsupported modes and unverified edge ordering remain
      // visible expected rejections, never traffic to physical hardware.
      continue;
    }
  }
  return 0;
}
