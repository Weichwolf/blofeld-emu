// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_timers.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace {
constexpr unsigned timer_clock_action = 17;
constexpr unsigned timer_drive_action = 18;
constexpr unsigned timer_read_action = 19;
constexpr unsigned timer_vectors_action = 20;
constexpr unsigned timer_ack_action = 21;
constexpr unsigned action_count = 22;

void timer_action(blofeld::DspBus &bus,
                  const std::uint8_t *record,
                  const std::uint32_t address,
                  const std::uint32_t value) {
  switch ((record[0] % action_count) - timer_clock_action) {
    case 0: {
      const auto upper = (static_cast<std::uint64_t>(record[1]) << 24U) |
                         (static_cast<std::uint64_t>(record[2]) << 16U) |
                         (static_cast<std::uint64_t>(record[3]) << 8U) | record[4];
      bus.timers().advance_clk2((upper << 32U) | value);
      break;
    }
    case 1:
      bus.timers().set_tio_drive(
          record[1] % 4U,
          (record[8] & 2U) != 0 ? std::nullopt : std::optional<bool>((record[8] & 1U) != 0));
      break;
    case 2: static_cast<void>(bus.timers().read(address, value)); break;
    case 3: static_cast<void>(bus.timers().interrupt_vectors()); break;
    case 4: bus.timers().acknowledge_interrupt(value); break;
    default: throw blofeld::Fault("Invalid DSP timer fuzzer action");
  }
}
}

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, const std::size_t size) {
  constexpr std::size_t stride = 9;
  constexpr std::size_t operation_limit = 256;
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
        case 15:
          static_cast<void>(bus.access_peripheral_bit(
              {.space = area,
               .address = address,
               .bit = record[5],
               .operation = static_cast<blofeld::DspBitOperation>(record[6] % 5U)}));
          break;
        case 16:
          bus.gpio(static_cast<blofeld::DspPort>(record[1] % 4U))
              .write({.reg = static_cast<blofeld::DspGpioRegister>(record[2] % 4U),
                      .value = value,
                      .mask = address});
          break;
        case timer_clock_action:
        case timer_drive_action:
        case timer_read_action:
        case timer_vectors_action:
        case timer_ack_action: timer_action(bus, record, address, value); break;
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
