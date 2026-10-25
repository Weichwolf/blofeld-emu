// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/mcu_interrupts.hpp"

#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_gpio.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace blofeld {
namespace {
constexpr unsigned interrupt_mask = 0x10;
constexpr unsigned x_mask = 0x40;
constexpr unsigned edge_only = 0x80;
constexpr unsigned irq_enable = 0x40;
constexpr unsigned irq_pin = 2;
constexpr unsigned xirq_pin = 1;
constexpr unsigned priority_mask = 0xfe;
constexpr std::uint16_t irq_vector = 0xfff2;
constexpr unsigned xirq_vector = 0xfff4;
constexpr unsigned realtime_vector = 0xfff0;
constexpr unsigned vector_page = 0xff00;
constexpr unsigned first_vector = 0xff82;
constexpr unsigned test_address_mask = 15;
}

void McuInterrupts::reset() {
  const auto pins = pins_;
  *this = McuInterrupts{};
  pins_ = pins;
}

std::string_view McuInterrupts::register_name(const std::uint16_t offset) {
  switch (offset) {
    case hc12::interrupt_test_control: return "ITCR normal mode";
    case hc12::interrupt_test: return "ITEST normal mode";
    case hc12::irq_control: return "INTCR/IRQCR";
    case hc12::interrupt_priority: return "HPRIO";
    default: return {};
  }
}

std::uint8_t McuInterrupts::read(const std::uint16_t offset) const {
  switch (offset) {
    case hc12::interrupt_test_control: return test_select_;
    case hc12::interrupt_test: return 0;
    case hc12::irq_control: return control_;
    case hc12::interrupt_priority: return priority_;
    default: throw Fault("Unknown interrupt register read at " + hex(offset));
  }
}

void McuInterrupts::write(const ByteWrite write, const std::uint8_t ccr) {
  switch (write.address) {
    case hc12::interrupt_test_control:
      test_select_ = static_cast<std::uint8_t>(write.value & test_address_mask);
      return;
    case hc12::interrupt_test: return;
    case hc12::interrupt_priority:
      if ((ccr & interrupt_mask) != 0) {
        priority_ = static_cast<std::uint8_t>(write.value & priority_mask);
      }
      return;
    case hc12::irq_control: {
      unsigned edge = control_ & edge_only;
      if (!edge_written_) {
        edge = write.value & edge_only;
        edge_written_ = true;
      }
      control_ = static_cast<std::uint8_t>(edge | (write.value & irq_enable));
      if (edge != 0 && (pins_.known & irq_pin) == 0 && (control_ & irq_enable) != 0) {
        edge_known_ = false;
      }
      return;
    }
    default: throw Fault("Unknown interrupt register write at " + hex(write.address));
  }
}

void McuInterrupts::observe_pins(const PinSample sample) {
  if ((sample.levels & ~sample.known) != 0 || (sample.known & ~3U) != 0) {
    throw Fault("Invalid external interrupt pin sample");
  }
  if ((control_ & (edge_only | irq_enable)) == (edge_only | irq_enable)) {
    if ((sample.known & pins_.known & irq_pin) == 0) {
      edge_known_ = false;
    } else if ((pins_.levels & irq_pin) != 0 && (sample.levels & irq_pin) == 0) {
      edge_pending_ = true;
    }
  }
  pins_ = sample;
}

bool McuInterrupts::external_irq() const {
  if ((control_ & irq_enable) == 0) { return false; }
  if ((control_ & edge_only) != 0) {
    if (!edge_known_) { throw Fault("Unknown external IRQ edge history"); }
    return edge_pending_;
  }
  if ((pins_.known & irq_pin) == 0) { throw Fault("Unknown external IRQ pad level"); }
  return (pins_.levels & irq_pin) == 0;
}

bool McuInterrupts::valid_maskable(const std::uint16_t vector) {
  if (vector < first_vector || vector > irq_vector || (vector & 1U) != 0) { return false; }
  // Device-specific reserved vector slots in 9S12UF32DGV1 table 5-1.
  constexpr unsigned reserved90 = 0xff90;
  constexpr unsigned reserved92 = 0xff92;
  constexpr unsigned reservedba = 0xffba;
  constexpr unsigned reservedbc = 0xffbc;
  constexpr unsigned reservedd8 = 0xffd8;
  return vector != reserved90 && vector != reserved92 && vector != reservedba &&
         vector != reservedbc && vector != reservedd8;
}

std::optional<std::uint16_t>
McuInterrupts::choose(const std::span<const std::uint16_t> pending) const {
  const auto requested = static_cast<std::uint16_t>(vector_page | priority_);
  const auto promoted = valid_maskable(requested) ? requested : irq_vector;
  std::optional<std::uint16_t> result;
  for (const auto vector : pending) {
    if (!valid_maskable(vector)) {
      throw Fault("Invalid pending MCU interrupt vector " + hex(vector));
    }
    if (vector == promoted) { return vector; }
    if (!result || vector > *result) { result = vector; }
  }
  if (promoted != irq_vector && promoted != realtime_vector &&
      std::ranges::find(pending, realtime_vector) != pending.end() && pending.size() > 1) {
    throw Fault("Unverified RTI/HPRIO replacement arbitration");
  }
  return result;
}

std::optional<InterruptRequest> McuInterrupts::select(const std::span<const std::uint16_t> pending,
                                                      const std::uint8_t ccr) const {
  for (const auto vector : pending) {
    if (!valid_maskable(vector)) {
      throw Fault("Invalid pending MCU interrupt vector " + hex(vector));
    }
  }
  if ((ccr & x_mask) == 0) {
    if ((pins_.known & xirq_pin) == 0) { throw Fault("Unknown external XIRQ pad level"); }
    if ((pins_.levels & xirq_pin) == 0) {
      return InterruptRequest{.vector = xirq_vector, .kind = InterruptKind::xirq};
    }
  }
  if ((ccr & interrupt_mask) != 0) { return {}; }
  if (external_irq()) {
    const auto requested = static_cast<std::uint16_t>(vector_page | priority_);
    const auto promoted = valid_maskable(requested) ? requested : irq_vector;
    // External IRQ is the default highest source, unless an active peripheral
    // is promoted. The pending span contains only peripheral requests.
    if (std::ranges::find(pending, promoted) != pending.end()) {
      return InterruptRequest{.vector = promoted};
    }
    return InterruptRequest{.vector = irq_vector};
  }
  const auto vector = choose(pending);
  if (!vector) { return {}; }
  return InterruptRequest{.vector = *vector};
}

void McuInterrupts::acknowledge(const std::uint16_t vector) {
  if (vector == irq_vector) {
    edge_pending_ = false;
    edge_known_ = true;
  }
}

}
