// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_bus.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_memory.hpp"

#include <cstdint>
#include <functional>
#include <span>
#include <string_view>
#include <utility>

namespace blofeld {
namespace {
constexpr std::uint16_t reserved_sci_end = 0x00d0;
constexpr std::uint16_t reserved_flash_begin = 0x0100;
}

McuBus::McuBus(const std::span<const std::uint8_t> flash) : memory_(flash) {}

void McuBus::reset() {
  memory_.reset();
  gpio_.reset();
  timer_control_ = 0;
  ++revision_;
}

bool McuBus::reserved_register(const std::uint16_t offset) {
  // Explicitly documented read-zero/write-ignored range, not an unknown-IO fallback.
  return offset >= reserved_sci_end && offset < reserved_flash_begin;
}

std::uint8_t McuBus::read_register(const std::uint16_t offset) const {
  if (!McuMemory::register_name(offset).empty()) { return memory_.read_register(offset); }
  if (!McuGpio::register_name(offset).empty()) { return gpio_.read(offset); }
  if (offset == hc12::smramcfg || offset == hc12::smramstat) { return 0; }
  if (reserved_register(offset)) { return 0; }
  if (offset == hc12::tscr1) { return timer_control_; }
  throw Fault("Unimplemented MCU peripheral read at " + hex(offset));
}

std::string_view McuBus::write_register(const ByteWrite write) {
  const auto offset = write.address;
  const auto value = write.value;
  const auto memory_name = McuMemory::register_name(offset);
  if (!memory_name.empty()) {
    memory_.write_register(write);
    return memory_name;
  }
  const auto gpio_name = McuGpio::register_name(offset);
  if (!gpio_name.empty()) {
    gpio_.write(write);
    return gpio_name;
  }
  if (reserved_register(offset)) { return "reserved/write ignored"; }
  if (offset == hc12::tscr1) {
    // Control latch only: counter, compare and interrupt accesses still fail.
    timer_control_ = value;
    return "TSCR1";
  }
  if (offset == hc12::smramcfg && (value & 1U) == 0) { return "SMRAMCFG normal mode"; }
  throw Fault("Unimplemented MCU peripheral write at " + hex(offset));
}

std::uint8_t McuBus::read(const std::uint16_t address) {
  const auto location = memory_.locate(address);
  if (location.space == McuSpace::registers) {
    try {
      return read_register(location.offset);
    } catch (const Fault &fault) {
      throw Fault("MCU read at " + hex(address) + " (register offset " + hex(location.offset) +
                  "): " + fault.what());
    }
  }
  return memory_.read(address);
}

void McuBus::write(const std::uint16_t address, const std::uint8_t value) {
  const auto location = memory_.locate(address);
  if (location.space == McuSpace::registers) {
    try {
      const auto name = write_register({.address = location.offset, .value = value});
      if (observer_) {
        observer_({.write = true, .address = address, .value = value, .name = name});
      }
    } catch (const Fault &fault) {
      throw Fault("MCU write at " + hex(address) + " (register offset " + hex(location.offset) +
                  "): " + fault.what());
    }
  } else {
    memory_.write({.address = address, .value = value});
  }
  ++revision_;
}

std::uint8_t McuBus::peek(const std::uint16_t address) const {
  return memory_.peek(address);
}

void McuBus::check_execute(const InstructionFetch fetch) const {
  constexpr unsigned address_space_size = 65536;
  if (fetch.length == 0 || fetch.length > address_space_size - fetch.address) {
    throw Fault("Invalid MCU instruction fetch range at " + hex(fetch.address));
  }
  for (unsigned index = 0; index < fetch.length; ++index) {
    const auto target = static_cast<std::uint32_t>(fetch.address) + index;
    if (target > hc12::address_mask) { throw Fault("MCU instruction fetch exceeds address space"); }
    memory_.check_execute(static_cast<std::uint16_t>(target));
  }
}

void McuBus::retire_instruction() {
  if (memory_.retire_instruction()) { ++revision_; }
}

void McuBus::set_pin_drive(const McuPort port, const PinDrive drive) {
  gpio_.set_drive(port, drive);
}

void McuBus::set_observer(std::function<void(const BusEvent &)> observer) {
  observer_ = std::move(observer);
}

}
