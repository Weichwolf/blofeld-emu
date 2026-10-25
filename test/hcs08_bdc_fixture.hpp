// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace blofeld::test::hcs08_bdc {
// Explicit candidate target mapping. CPU writes still use the original fixture
// footprint; BDC upload writes use the actual RAM map and separate privileges.
class Memory final : public Hcs08BdcMemory {
public:
  explicit Memory(hcs08_programmer::Memory &bus) : bus_(bus) {}

  [[nodiscard]] bool secured(const ExecutionContext context) const override {
    bus_.flash.context(context, Hcs08FlashAgent::background);
    return bus_.flash.security_engaged();
  }

  std::uint8_t read(const std::uint16_t address, const ExecutionContext context) override {
    bus_.flash.context(context, Hcs08FlashAgent::background);
    if (address >= hcs08_flash::first) { return bus_.flash.read_array(address); }
    if (address >= hcs08_flash::divider && address <= hcs08_flash::command) {
      return bus_.flash.read(address);
    }
    if (address < hcs08_programmer::ram_first || address >= hcs08_programmer::ram_end) {
      throw Fault("Unqualified BDC target read address=" + hex(address) + " pc=" + hex(context.pc));
    }
    if (secured(context)) { return 0; }
    bus_.context = context;
    return bus_.hcs08::Memory::read(address);
  }

  void write(const ByteWrite access, const ExecutionContext context) override {
    bus_.flash.context(context, Hcs08FlashAgent::background);
    if (access.address >= hcs08_flash::first) {
      bus_.flash.stage(access);
    } else if (access.address >= hcs08_flash::divider && access.address <= hcs08_flash::command) {
      bus_.flash.write(access);
    } else {
      if (access.address < hcs08_programmer::ram_first ||
          access.address >= hcs08_programmer::ram_end) {
        throw Fault("Unqualified BDC target write address=" + hex(access.address) +
                    " pc=" + hex(context.pc));
      }
      if (secured(context)) { return; }
      bus_.context = context;
      bus_.hcs08::Memory::write(access.address, access.value);
    }
  }

private:
  hcs08_programmer::Memory &bus_;
};

inline void command(Hcs08Bdc &bdc, const std::span<const std::uint8_t> frame) {
  for (const auto byte : frame) { bdc.receive(byte); }
  bdc.service();
}

inline void write(Hcs08Bdc &bdc, const ByteWrite access) {
  constexpr unsigned byte_mask = 255;
  constexpr unsigned high_shift = 8;
  constexpr std::uint8_t opcode = 0xc0;
  const std::array<std::uint8_t, 4> frame{opcode,
                                          static_cast<std::uint8_t>(access.address >> high_shift),
                                          static_cast<std::uint8_t>(access.address & byte_mask),
                                          access.value};
  command(bdc, frame);
}

inline std::uint8_t read(Hcs08Bdc &bdc, const std::uint16_t address) {
  constexpr unsigned byte_mask = 255;
  constexpr unsigned high_shift = 8;
  constexpr std::uint8_t opcode = 0xe0;
  const std::array<std::uint8_t, 3> frame{opcode,
                                          static_cast<std::uint8_t>(address >> high_shift),
                                          static_cast<std::uint8_t>(address & byte_mask)};
  command(bdc, frame);
  return bdc.read_reply();
}
} // namespace blofeld::test::hcs08_bdc
