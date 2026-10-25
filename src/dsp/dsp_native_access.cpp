// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/detail/dsp_native_access.hpp"

#include "common/bus.hpp"
#include "dsp/detail/dsp_reference_execution.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_native_access.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace blofeld {
namespace {
std::string access_context(const DspNativeAccess &access) {
  constexpr std::array names{"opcode", "extension", "read", "write", "bit", "vector"};
  constexpr std::array spaces{"P:", "X:", "Y:"};
  return "DSP native access " +
         std::string(access.stage == DspNativeAccessStage::before ? "before " : "after ") +
         names.at(static_cast<unsigned>(access.kind)) + " at " +
         spaces.at(static_cast<unsigned>(access.target.space)) + hex(access.target.address, 6) +
         " ordinal=" + std::to_string(access.ordinal) + ": ";
}
}

void DspNativeBus::begin(DspNativeAccessObserver observer, const DspNativeAccess context) {
  observer_ = std::move(observer);
  context_ = context;
}

void DspNativeBus::end() noexcept {
  observer_ = {};
}

void DspNativeBus::notify(const DspNativeAccess &access) const {
  try {
    observer_.observe(access);
  } catch (const std::exception &error) {
    throw Fault(access_context(access) + error.what());
  } catch (...) {
    throw Fault(access_context(access) + "Non-standard native access observer exception");
  }
}

DspNativeAccess DspNativeBus::before(DspNativeAccess access) {
  if (context_.ordinal == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("DSP native access ordinal exhausted");
  }
  access.pc = context_.pc;
  access.omr = context_.omr;
  access.ordinal = context_.ordinal++;
  notify(access);
  return access;
}

void DspNativeBus::after(DspNativeAccess access, const std::uint32_t value) const {
  access.stage = DspNativeAccessStage::after;
  access.value = value;
  notify(access);
}

std::uint32_t DspNativeBus::read(const DspAddress target, const DspNativeAccessKind kind) {
  if (reference_.intercepting()) { return reference_.read(target, kind); }
  if (!observer_.observe) { return bus_.read(target); }
  const auto access = before({.kind = kind, .target = target});
  const auto value = bus_.read(target);
  after(access, value);
  return value;
}

void DspNativeBus::write(const DspStore target) {
  if (reference_.intercepting()) {
    reference_.write(target);
    return;
  }
  if (!observer_.observe) {
    bus_.write(target);
    return;
  }
  const auto access = before({.kind = DspNativeAccessKind::write,
                              .target = {.space = target.space, .address = target.address},
                              .value = target.value});
  bus_.write(target);
  after(access, target.value);
}

std::optional<bool> DspNativeBus::bit(const DspBitAccess target) {
  const DspAddress address{.space = target.space, .address = target.address};
  if (reference_.intercepting()) {
    if (!DspBus::handles_peripheral_bit(address)) { return std::nullopt; }
    return reference_.bit(target);
  }
  if (!observer_.observe || !DspBus::handles_peripheral_bit(address)) {
    return bus_.access_peripheral_bit(target);
  }
  const auto access =
      before({.kind = DspNativeAccessKind::bit,
              .target = address,
              .bit = DspNativeAccessBit{.index = target.bit, .operation = target.operation}});
  const auto value = bus_.access_peripheral_bit(target);
  if (!value) { throw Fault("DSP peripheral bit route changed during native observation"); }
  after(access, *value ? 1U : 0U);
  return value;
}

std::uint32_t DspNativeBus::peek_program(const std::uint32_t address) {
  return reference_.intercepting() ? reference_.peek_program(address) : bus_.peek_program(address);
}

void DspNativeBus::reset_peripherals() {
  if (reference_.intercepting()) {
    reference_.reset_peripherals();
  } else {
    bus_.reset_peripherals();
  }
}

DspNativeAccessScope::DspNativeAccessScope(DspNativeBus &bus,
                                           DspNativeAccessObserver observer,
                                           const DspNativeAccess context)
    : bus_(bus) {
  bus_.begin(std::move(observer), context);
}

DspNativeAccessScope::~DspNativeAccessScope() {
  bus_.end();
}
}
