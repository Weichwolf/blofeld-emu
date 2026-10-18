// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_gpio.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, const std::size_t size) {
  constexpr std::size_t stride = 9;
  constexpr std::size_t operation_limit = 256;
  constexpr unsigned action_count = 15;
  constexpr unsigned word_mask = 0xffffff;
  blofeld::DspBus bus;
  const auto count = size / stride < operation_limit ? size / stride : operation_limit;
  for (std::size_t index = 0; index < count; ++index) {
    const auto *record = data + (index * stride);
    const auto area = static_cast<blofeld::DspSpace>(record[1] % 4U);
    const auto address = (static_cast<std::uint32_t>(record[2]) << 16U) |
                         (static_cast<std::uint32_t>(record[3]) << 8U) | record[4];
    const auto value = (static_cast<std::uint32_t>(record[5]) << 24U) |
                       (static_cast<std::uint32_t>(record[6]) << 16U) |
                       (static_cast<std::uint32_t>(record[7]) << 8U) | record[8];
    try {
      switch (record[0] % action_count) {
        case 0: static_cast<void>(bus.read({.space = area, .address = address})); break;
        case 1: bus.write({.space = area, .address = address, .value = value}); break;
        case 2: static_cast<void>(bus.peek_program(address)); break;
        case 3: bus.observe_omr(value); break;
        case 4: bus.advance_instruction_cycles(value); break;
        case 5: bus.reset(); break;
        case 6: {
          const std::array<std::uint32_t, 2> words{value, value ^ word_mask};
          bus.install_rom({.space = area, .start = address, .words = words});
          break;
        }
        case 7: static_cast<void>(bus.take_code_changes()); break;
        case 8:
          bus.shi().observe_spi({.selected = (record[6] & 1U) != 0,
                                 .clock = (record[7] & 1U) != 0,
                                 .mosi = (record[8] & 1U) != 0});
          break;
        case 9:
          bus.pll().observe_pinit(
              (record[8] & 2U) != 0 ? std::nullopt : std::optional<bool>((record[8] & 1U) != 0));
          break;
        case 10: static_cast<void>(bus.pll().target_ratio()); break;
        case 11:
          bus.gpio(static_cast<blofeld::DspPort>(record[1] % 4U))
              .set_drive({.mask = address, .levels = value});
          break;
        case 12:
          static_cast<void>(
              bus.gpio(static_cast<blofeld::DspPort>(record[1] % 4U)).observe_pads(value));
          break;
        case 13: bus.reset_peripherals(); break;
        case 14:
          static_cast<void>(
              bus.gpio(static_cast<blofeld::DspPort>(record[1] % 4U))
                  .read(static_cast<blofeld::DspGpioRegister>(record[2] % 4U), value));
          break;
        default: throw blofeld::Fault("Invalid DSP model-fuzzer action");
      }
    } catch (const blofeld::Fault &) {
      // Invalid modes/addresses, missing ROM/RAM and undefined peripheral
      // operations are expected explicit rejections; no physical device is used.
      continue;
    }
  }
  return 0;
}
