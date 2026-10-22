// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_interrupts.hpp"

#include "blofeld/bus.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace blofeld {
namespace {
constexpr unsigned peripheral_priority = 0xfffffe;
constexpr unsigned core_priority = 0xffffff;
constexpr unsigned shi_mask = 0x0c;
constexpr unsigned timer_mask = 0x300;
constexpr unsigned timer_shift = 8;
constexpr unsigned timer_first = 0x54;
constexpr unsigned irqa_vector = 0x10;
constexpr unsigned irqa_priority_mask = 3;
constexpr unsigned irqa_mode = 4;
// DSP56371UM table4-5, not numeric vector-address order.
constexpr std::array<unsigned, 13> priorities{
    irqa_vector, 0x4c, 0x4a, 0x42, 0x48, 0x40, 0x44, 0x56, 0x54, 0x5a, 0x58, 0x5e, 0x5c};
}

void DspInterrupts::reset() {
  peripheral_ = 0;
  core_ = 0;
  irqa_edge_ = false;
  // External observations survive both reset types; they are not chip defaults.
}

bool DspInterrupts::handles(const std::uint32_t address) {
  return address == peripheral_priority || address == core_priority;
}

std::uint32_t DspInterrupts::read(const std::uint32_t address) const {
  if (!handles(address)) {
    throw Fault("Invalid DSP interrupt-priority read at X:" + hex(address, 6));
  }
  return address == core_priority ? core_ : peripheral_;
}

void DspInterrupts::write(const std::uint32_t address, const std::uint32_t value) {
  if (!handles(address)) {
    throw Fault("Invalid DSP interrupt-priority write at X:" + hex(address, 6));
  }
  const auto allowed =
      address == core_priority ? irqa_priority_mask | irqa_mode : shi_mask | timer_mask;
  if ((value & ~allowed) != 0) {
    throw Fault("Unimplemented DSP interrupt-priority fields at X:" + hex(address, 6) +
                " value=" + hex(value, 6));
  }
  if (address == peripheral_priority) {
    peripheral_ = value;
    return;
  }
  const bool was_enabled = (core_ & irqa_priority_mask) != 0;
  const bool enabled = (value & irqa_priority_mask) != 0;
  if (was_enabled && enabled && ((core_ ^ value) & irqa_mode) != 0) {
    throw Fault("Unqualified enabled DSP IRQA trigger-mode change at X:" + hex(address, 6));
  }
  if (!enabled) {
    irqa_edge_ = false;
  } else if (!was_enabled && (value & irqa_mode) != 0 && !irqa_.has_value()) {
    irqa_edge_ = std::nullopt;
  }
  core_ = value;
}

void DspInterrupts::observe_irqa(const std::optional<bool> level) {
  if ((core_ & irqa_priority_mask) == 0) {
    irqa_edge_ = false; // DSP56300FM 2.3.2.1: disabled detection latch stays reset.
  } else if ((core_ & irqa_mode) != 0) {
    if (!irqa_.has_value() || !level.has_value()) {
      if (!irqa_edge_.has_value() || !irqa_edge_.value()) { irqa_edge_ = std::nullopt; }
    } else if (irqa_.value() && !level.value()) {
      irqa_edge_ = true;
    }
  }
  irqa_ = level;
}

std::optional<DspInterruptRequest> DspInterrupts::irqa_request(const unsigned mask) const {
  if (mask > irqa_priority_mask) { throw Fault("Invalid DSP CPU interrupt mask"); }
  const auto encoded = core_ & irqa_priority_mask;
  if (encoded == 0 || encoded - 1 < mask) { return std::nullopt; }
  if ((core_ & irqa_mode) != 0) {
    if (irqa_edge_.has_value() && irqa_edge_.value()) {
      return DspInterruptRequest{.vector = irqa_vector, .level = encoded - 1};
    }
    if (!irqa_.has_value() || !irqa_edge_.has_value()) {
      throw Fault("Unqualified DSP IRQA edge history at X:0xffffff vector=0x10");
    }
    return std::nullopt;
  }
  if (!irqa_.has_value()) { throw Fault("Unknown DSP IRQA input at X:0xffffff vector=0x10"); }
  if (irqa_.value()) { return std::nullopt; }
  return DspInterruptRequest{.vector = irqa_vector, .level = encoded - 1};
}

void DspInterrupts::acknowledge_irqa() {
  if (!irqa_request(0)) { throw Fault("DSP IRQA acknowledgement has no pending vector=0x10"); }
  if ((core_ & irqa_mode) != 0) { irqa_edge_ = false; }
}

std::optional<DspInterruptRequest>
DspInterrupts::select(const std::span<const std::uint32_t> pending, const unsigned mask) const {
  constexpr unsigned maximum_mask = 3;
  if (mask > maximum_mask) { throw Fault("Invalid DSP CPU interrupt mask"); }
  std::optional<DspInterruptRequest> selected;
  auto best_rank = priorities.size();
  for (const auto vector : pending) {
    const auto *const found = std::ranges::find(priorities, vector);
    if (found == priorities.end()) {
      throw Fault("Unsupported DSP interrupt vector " + hex(vector, 2));
    }
    const auto rank = static_cast<std::size_t>(found - priorities.begin());
    const auto level = priority(vector);
    if (!level || *level < mask) { continue; }
    if (!selected || *level > selected->level || (*level == selected->level && rank < best_rank)) {
      selected = DspInterruptRequest{.vector = vector, .level = *level};
      best_rank = rank;
    }
  }
  return selected;
}

std::optional<unsigned> DspInterrupts::priority(const std::uint32_t vector) const {
  if (std::ranges::find(priorities, vector) == priorities.end()) {
    throw Fault("Unsupported DSP interrupt vector " + hex(vector, 2));
  }
  constexpr unsigned field_mask = 3;
  if (vector == irqa_vector) {
    const auto encoded = core_ & field_mask;
    return encoded == 0 ? std::nullopt : std::optional<unsigned>(encoded - 1);
  }
  const auto shift = vector >= timer_first ? timer_shift : 2U;
  const auto encoded = (peripheral_ >> shift) & field_mask;
  if (encoded == 0) { return std::nullopt; }
  return encoded - 1;
}
}
