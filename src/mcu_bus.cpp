// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_bus.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <span>
#include <string_view>
#include <utility>

namespace blofeld {

McuBus::McuBus(const std::span<const std::uint8_t> flash) {
  if (flash.size() != 32768) { throw Fault("The MCU requires a 32768-byte flash image"); }
  std::ranges::copy(flash, bytes_.begin() + 0x8000);
}

bool McuBus::ram_mapped(const std::uint16_t address) {
  // Full SMRAM configuration/remapping remains to be modeled. No speculative
  // RAM range is accepted before that implementation is verified.
  static_cast<void>(address);
  return false;
}

std::string_view McuBus::latch_name(const std::uint16_t address) {
  switch (address) {
    case hc12::ddra: return "DDRA";
    case hc12::ddrb: return "DDRB";
    case hc12::porte: return "PORTE output latch";
    case hc12::ddre: return "DDRE";
    case hc12::pear: return "PEAR";
    case hc12::initee: return "INITEE";
    case hc12::tscr1: return "TSCR1";
    case hc12::ptt: return "PTT output latch";
    case hc12::ddrt: return "DDRT";
    case hc12::ptm: return "PTM output latch";
    case hc12::ddrm: return "DDRM";
    case hc12::ptj: return "PTJ output latch";
    case hc12::ddrj: return "DDRJ";
    case hc12::modrr: return "MODRR";
    case hc12::ptq: return "PTQ output latch";
    case hc12::ddrq: return "DDRQ";
    case hc12::perq: return "PERQ";
    case hc12::ppsq: return "PPSQ";
    default: return {};
  }
}

std::uint8_t McuBus::read(const std::uint16_t address) {
  if (address >= 0x8000 || ram_mapped(address)) { return bytes_[address]; }
  if (address < 0x400) { throw Fault("Unimplemented MCU peripheral/pin read at " + hex(address)); }
  throw Fault("Unmapped MCU read at " + hex(address));
}

void McuBus::write(const std::uint16_t address, const std::uint8_t value) {
  if (ram_mapped(address)) {
    ++revision_;
    bytes_[address] = value;
    return;
  }
  if (address >= 0x8000) {
    throw Fault("MCU flash write requires the flash-controller model at " + hex(address));
  }
  const auto name = latch_name(address);
  if (name.empty()) { throw Fault("Unimplemented MCU peripheral write at " + hex(address)); }
  ++revision_;
  bytes_[address] = value;
  if (observer_) { observer_({.write = true, .address = address, .value = value, .name = name}); }
}

std::uint8_t McuBus::peek(const std::uint16_t address) const {
  return bytes_[address];
}

void McuBus::check_execute(const InstructionFetch fetch) const {
  for (unsigned index = 0; index < fetch.length; ++index) {
    const auto target = static_cast<std::uint32_t>(fetch.address) + index;
    if (target > hc12::address_mask ||
        (target < 0x8000 && !ram_mapped(static_cast<std::uint16_t>(target)))) {
      throw Fault("Unmapped MCU instruction fetch at " + hex(target));
    }
  }
}

void McuBus::set_observer(std::function<void(const BusEvent &)> observer) {
  observer_ = std::move(observer);
}

} // namespace blofeld
