// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "storage/device_identity.hpp"
#include "storage/flash_image.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned serial_first = 0x81e0, serial_offset = 0x1e0;
constexpr unsigned application = 0x8200, phy_poll = 0x8049, native_boot_instructions = 90314;
constexpr unsigned source_serial_check = 0x84f2, source_after_store = 0x84f9;
constexpr unsigned own_frame = 0x25e0, frame_bytes = 9, serial_slot = 6, dsp_slot = 8;
constexpr unsigned canary = 0xa5, instruction_limit = 200000;
constexpr unsigned own_d = 0xa55a, own_x = 0x1234, own_y = 0x5678, retained_a = 0xa500;
constexpr unsigned fixture_inputs = 0xfb, ram_end = 0x2600;
constexpr unsigned input_ccr = 0xd0, invalid_ccr = 0xd8, available_ccr = 0xd4;

void require(const bool condition, const std::string_view text) {
  if (!condition) { throw Fault(std::string(text)); }
}

void native_serial_check(McuBus &bus, Hc12 &cpu, const bool provisioned) {
  for (unsigned n = 0; n < frame_bytes; ++n) {
    bus.write(static_cast<std::uint16_t>(own_frame + n), canary);
  }
  // Declared diagnostic ABI at the actual unchanged resident serial-load block.
  // Other startup status fields remain canaries; no DSP reply or healthy boot is supplied.
  cpu.set_state({.pc = source_serial_check,
                 .d = own_d,
                 .x = own_x,
                 .y = own_y,
                 .sp = own_frame,
                 .ccr = input_ccr});
  for (unsigned n = 0; n < 3; ++n) { cpu.step(); }
  const auto expected = provisioned ? 0U : 0x80U;
  require(cpu.state().pc == source_after_store && cpu.state().sp == own_frame &&
              cpu.state().d == (retained_a | expected) &&
              cpu.state().ccr == (provisioned ? available_ccr : invalid_ccr),
          "Original serial decision pc=" + hex(cpu.state().pc) + " sp=" + hex(cpu.state().sp) +
              " d=" + hex(cpu.state().d) + " ccr=" + hex(cpu.state().ccr));
  for (unsigned n = 0; n < frame_bytes; ++n) {
    require(
        bus.read(static_cast<std::uint16_t>(own_frame + n)) ==
            (n == serial_slot ? expected : canary),
        "Only the genuine native serial-status slot changes; other startup statuses stay intact");
  }
  require(bus.read(own_frame + dsp_slot) == canary,
          "A virtual serial must not fabricate a successful DSP startup status");
  std::cout << "SERIAL provisioned=" << provisioned
            << " first=" << static_cast<unsigned>(bus.read(serial_first)) << " status=" << expected
            << " pc=" << cpu.state().pc << " ccr=" << static_cast<unsigned>(cpu.state().ccr)
            << " dsp=" << canary << '\n';
}

void original_boot(const McuFlashImage &image, const bool provisioned) {
  McuBus bus(image);
  bus.set_pin_drive(McuPort::e,
                    {.mask = fixture_inputs}); // Existing explicit original-boot input fixture.
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  bool observed_phy = false;
  unsigned instructions = 0;
  while (cpu.state().pc != application && instructions < instruction_limit) {
    if (cpu.state().pc == phy_poll && !observed_phy) {
      bus.observe_phy({.edges = 15});
      observed_phy = true;
    }
    cpu.step();
    ++instructions;
  }
  require(instructions == native_boot_instructions && cpu.state().pc == application &&
              cpu.state().sp == ram_end && cpu.state().d == 0 && observed_phy,
          "The actual unchanged bootloader validates the original application checksum with either "
          "unit image");
  std::cout << "BOOT provisioned=" << provisioned << " instructions=" << instructions
            << " pc=" << cpu.state().pc << " sp=" << cpu.state().sp << " d=" << cpu.state().d
            << '\n';
  native_serial_check(bus, cpu, provisioned);
  const auto retained = bus.power_off_flash_image();
  require(
      retained.bytes == image.bytes && retained.known == image.known &&
          retained.fresh == image.fresh,
      "Boot and native serial consumption preserve every program/factory byte and NVM history bit");
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Expected hash-pinned local original1.25 archive");
    const auto firmware = Firmware::load(arguments[1]);
    const auto original = McuFlashImage::from_known(firmware.mcu);
    const auto factory = provision_virtual_identity(original, "EMU-NATIVE01");
    for (unsigned n = 0; n < McuFlashImage::byte_count; ++n) {
      require((n >= serial_offset && n < serial_offset + device_identity::serial_bytes) ||
                  original.bytes[n] == factory.bytes[n],
              "Every original executable, vector, checksum and option byte remains unchanged");
    }
    original_boot(original, false);
    const auto cold = decode_flash_image(encode_flash_image(factory));
    original_boot(cold, true);
    std::cout << "Original boot and serial warning use persistent factory data; full DSP boot "
                 "remains open.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
