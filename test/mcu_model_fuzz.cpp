// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, const std::size_t size) {
  constexpr std::size_t stride = 6;
  constexpr std::size_t operation_limit = 256;
  constexpr unsigned clock_slice_mask = 0x0fff;
  const std::array<std::uint8_t, 32768> flash{}; // Own synthetic bytes, no firmware required.
  auto bus = std::make_unique<blofeld::McuBus>(flash);
  const auto operations = size / stride < operation_limit ? size / stride : operation_limit;
  for (std::size_t index = 0; index < operations; ++index) {
    const auto *record = data + (index * stride);
    const auto address =
        static_cast<std::uint16_t>((static_cast<unsigned>(record[1]) << 8U) | record[2]);
    try {
      switch (record[0] % 10U) {
        case 0: static_cast<void>(bus->read(address)); break;
        case 1: bus->write(address, record[3]); break;
        case 2: bus->retire_instruction(); break;
        case 3:
          bus->set_pin_drive(static_cast<blofeld::McuPort>(record[1]),
                             {.mask = record[3], .levels = record[4], .floating = record[5]});
          break;
        case 4: bus->reset(); break;
        case 5: bus->check_execute({.address = address, .length = (record[3] & 7U) + 1U}); break;
        case 6: static_cast<void>(bus->peek(address)); break;
        case 7: bus = std::make_unique<blofeld::McuBus>(flash); break;
        case 8: bus->observe_phy({.edges = address}); break;
        case 9: bus->advance_sci(address & clock_slice_mask); break;
        default: throw blofeld::Fault("Invalid model-fuzzer action");
      }
    } catch (const blofeld::Fault &) {
      // Unsupported accesses, unknown RAM/pins and invalid configurations are
      // explicit expected rejections. Continue exercising state after rejection.
      continue;
    }
  }
  return 0;
}
