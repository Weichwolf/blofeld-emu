// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_flash.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_usb.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>

namespace {
constexpr unsigned action_count = 31;
enum class UsbAction : std::uint8_t { decoder = 18, sof, setup, out, input, finish, configuration };
enum class FlashAction : std::uint8_t { clock = 27, transfer, complete, power_cycle };

void flash_action(blofeld::McuBus &bus, const std::span<const std::uint8_t, 6> record) {
  constexpr unsigned bus_hz_scale = 1000;
  switch (static_cast<FlashAction>(record[0] % action_count)) {
    case FlashAction::clock: {
      const auto oscillator = ((static_cast<std::uint32_t>(record[1]) << 16U) |
                               (static_cast<std::uint32_t>(record[2]) << 8U) | record[3]) *
                              4;
      const auto clock = ((static_cast<std::uint32_t>(record[4]) << 8U) | record[5]) * bus_hz_scale;
      bus.observe_flash_clock({.oscillator_hz = oscillator, .bus_hz = clock});
      return;
    }
    case FlashAction::transfer: bus.advance_flash(blofeld::FlashMilestone::transfer_buffer); return;
    case FlashAction::complete:
      bus.advance_flash(blofeld::FlashMilestone::complete_command);
      return;
    case FlashAction::power_cycle: bus.power_cycle(); return;
    default: throw blofeld::Fault("Invalid flash model-fuzzer action");
  }
}

void usb_action(blofeld::McuBus &bus, const std::span<const std::uint8_t, 6> record) {
  const auto address =
      static_cast<std::uint16_t>((static_cast<unsigned>(record[1]) << 8U) | record[2]);
  switch (static_cast<UsbAction>(record[0] % action_count)) {
    case UsbAction::decoder:
      bus.observe_usb_decoder({.reset = (record[3] & 1U) != 0,
                               .suspended = (record[3] & 2U) != 0,
                               .enumerated = (record[3] & 4U) != 0,
                               .full_speed = (record[3] & 8U) != 0});
      return;
    case UsbAction::sof: bus.observe_usb_sof(address, (record[3] & 1U) != 0); return;
    case UsbAction::setup: {
      const std::array<std::uint8_t, 8> packet{
          record[1], record[2], record[3], record[4], record[5], 0, 0, 0};
      bus.receive_usb_setup(packet);
      return;
    }
    case UsbAction::out:
      bus.receive_usb_out({.buffer = static_cast<blofeld::UsbBuffer>(record[1]),
                           .data = record.subspan(3),
                           .data1 = (record[2] & 1U) != 0,
                           .crc_valid = (record[2] & 2U) != 0});
      return;
    case UsbAction::input:
      static_cast<void>(bus.request_usb_in(static_cast<blofeld::UsbBuffer>(record[1])));
      return;
    case UsbAction::finish: bus.finish_usb_in((record[3] & 1U) != 0); return;
    case UsbAction::configuration: {
      std::optional<std::uint8_t> interface;
      if ((record[5] & 1U) != 0) { interface = record[4]; }
      bus.observe_usb_configuration(
          {.configuration = record[3], .interface = interface, .alternate = record[2]});
      return;
    }
    default: throw blofeld::Fault("Invalid USB model-fuzzer action");
  }
}
}

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, const std::size_t size) {
  constexpr std::size_t stride = 6;
  constexpr std::size_t operation_limit = 256;
  constexpr unsigned clock_slice_mask = 0x0fff;
  constexpr unsigned memory_read_action = 17;
  constexpr unsigned code_observation_action = 25;
  constexpr unsigned flash_options_action = 26;
  constexpr unsigned flash_protection_offset = 0x7f0d;
  constexpr unsigned flash_security_offset = 0x7f0f;
  const std::array<std::uint8_t, 32768> flash{}; // Own synthetic bytes, no firmware required.
  auto bus = std::make_unique<blofeld::McuBus>(flash);
  const auto operations = size / stride < operation_limit ? size / stride : operation_limit;
  for (std::size_t index = 0; index < operations; ++index) {
    const auto *record = data + (index * stride);
    const auto address =
        static_cast<std::uint16_t>((static_cast<unsigned>(record[1]) << 8U) | record[2]);
    try {
      if (record[0] % action_count >= static_cast<unsigned>(FlashAction::clock)) {
        flash_action(*bus, std::span<const std::uint8_t, stride>{record, stride});
        continue;
      }
      if (record[0] % action_count == flash_options_action) {
        auto configured_flash = flash;
        configured_flash[flash_protection_offset] = record[3];
        configured_flash[flash_security_offset] = record[4];
        bus = std::make_unique<blofeld::McuBus>(configured_flash);
        continue;
      }
      if (record[0] % action_count == code_observation_action) {
        bus->note_code_byte(address);
        static_cast<void>(bus->code_revision());
        continue;
      }
      if (record[0] % action_count >= static_cast<unsigned>(UsbAction::decoder)) {
        usb_action(*bus, std::span<const std::uint8_t, stride>{record, stride});
        continue;
      }
      switch (record[0] % action_count) {
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
        case 10: bus->advance_timer(address & clock_slice_mask); break;
        case 11: bus->sample_timer_inputs(record[3]); break;
        case 12: bus->begin_instruction({.pc = address, .ccr = record[3]}); break;
        case 13: static_cast<void>(bus->interrupt_request(record[3])); break;
        case 14: bus->acknowledge_interrupt(address); break;
        case 15:
          bus->advance_oscillator(address);
          static_cast<void>(bus->oscillator_cycles_until_rti());
          break;
        case 16:
          bus->write_memory({.address = address,
                             .size = (record[3] & 7U) + 1U,
                             .value = (static_cast<unsigned>(record[4]) << 8U) | record[5]});
          break;
        case memory_read_action:
          static_cast<void>(bus->read_memory({.address = address, .size = (record[3] & 7U) + 1U}));
          break;
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
