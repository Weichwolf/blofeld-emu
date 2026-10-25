// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_flash.hpp"
#include "mcu/mcu_memory.hpp"
#include "storage/flash_image.hpp"

#include <cstdint>
#include <string>

namespace blofeld {

McuBus::McuBus(const McuFlashImage &image) : memory_(image), flash_(memory_) {}

McuFlashImage McuBus::power_off_flash_image() const {
  return flash_.power_off_image();
}

void McuBus::stage_flash(const MemoryWrite access) {
  constexpr unsigned address_space = 65536;
  if (access.size == 0 || access.size > sizeof(std::uint64_t) ||
      access.size > address_space - access.address) {
    throw Fault("Unsupported MCU flash operand range at " + hex(access.address));
  }
  const auto location = memory_.locate(access.address);
  if (access.size == 2 && (access.address & 1U) == 0) {
    const auto next = memory_.locate(static_cast<std::uint16_t>(access.address + 1));
    if (next.space != McuSpace::flash || next.offset != location.offset + 1) {
      throw Fault("Unqualified MCU flash operand crosses a mapping boundary at " +
                  hex(access.address));
    }
  }
  try {
    flash_.stage({.address = location.offset, .size = access.size, .value = access.value});
  } catch (const Fault &fault) {
    throw Fault("MCU flash operand write at " + hex(access.address) +
                " size=" + std::to_string(access.size) + " pc=" + hex(context_.pc) +
                " ccr=" + hex(context_.ccr, 2) + ": " + fault.what());
  }
  ++revision_;
}

void McuBus::observe_flash_clock(const FlashClockRates rates) {
  flash_.observe_clock(rates);
}

void McuBus::observe_flash_stop(const FlashStopTransition transition,
                                const FlashStopContract contract) {
  try {
    if (notifying_) { throw Fault("Flash STOP observation requires an inactive bus observer"); }
    flash_.observe_stop(transition, contract);
  } catch (const Fault &fault) {
    throw Fault("MCU flash STOP observation transition=" +
                std::to_string(static_cast<unsigned>(transition)) + " pc=" + hex(context_.pc) +
                " ccr=" + hex(context_.ccr, 2) + ": " + fault.what());
  }
  ++revision_;
  ++code_revision_;
}

void McuBus::power_cycle() {
  memory_.power_cycle();
  reset();
}

void McuBus::advance_flash(const FlashMilestone milestone) {
  try {
    flash_.advance(milestone);
  } catch (const Fault &fault) {
    throw Fault(
        "MCU flash reference milestone=" + std::to_string(static_cast<unsigned>(milestone)) +
        " pc=" + hex(context_.pc) + " ccr=" + hex(context_.ccr, 2) + ": " + fault.what());
  }
  ++revision_;
  ++code_revision_;
}

}
