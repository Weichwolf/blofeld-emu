// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_reference_execution.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_reference_execution.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <string>
#include <variant>

namespace blofeld {
namespace {
bool same_access(const DspNativeAccess &a, const DspNativeAccess &b) {
  return a.kind == b.kind && a.stage == b.stage && a.target.space == b.target.space &&
         a.target.address == b.target.address && a.value == b.value && a.bit == b.bit &&
         a.ordinal == b.ordinal && a.pc == b.pc && a.omr == b.omr;
}

bool same_effect(const DspReferenceEffect &a, const DspReferenceEffect &b) {
  if (a.index() != b.index()) { return false; }
  const auto *access = std::get_if<DspNativeAccess>(&a);
  return access == nullptr || same_access(*access, std::get<DspNativeAccess>(b));
}

std::string effect_context(const DspReferenceEffect &effect) {
  const auto *access = std::get_if<DspNativeAccess>(&effect);
  if (access == nullptr) { return "peripheral-reset"; }
  constexpr std::array names{"opcode", "extension", "read", "write", "bit", "vector"};
  constexpr std::array spaces{"P:", "X:", "Y:"};
  return std::string(names.at(static_cast<unsigned>(access->kind))) + " at " +
         spaces.at(static_cast<unsigned>(access->target.space)) + hex(access->target.address, 6) +
         " ordinal=" + std::to_string(access->ordinal);
}
}

void DspReferenceTape::begin(const DspNativeAccess context) {
  if (state_.active) { throw Fault("Native reference instruction is already pending"); }
  clear();
  state_.context = context;
  state_.active = true;
}

void DspReferenceTape::clear() {
  state_ = {};
  cursor_ = 0;
  ordinal_ = 0;
  mode_ = DspReferenceMode::ordinary;
}

void DspReferenceTape::restore(const DspReferenceState &state) {
  if (state.used > state.entries.size() || (state.active != state.pending.has_value()) ||
      (state.pending && state.used == state.entries.size())) {
    throw Fault("Invalid trusted native reference continuation");
  }
  state_ = state;
  cursor_ = 0;
  ordinal_ = 0;
  mode_ = DspReferenceMode::ordinary;
}

void DspReferenceTape::rewind() {
  if (!state_.active || state_.pending) {
    throw Fault("Native reference recomputation requires an admitted access boundary");
  }
  cursor_ = 0;
  ordinal_ = 0;
}

void DspReferenceTape::verify_complete() const {
  if (cursor_ != state_.used || state_.pending) {
    throw Fault("Native reference execution lost an admitted effect");
  }
}

void DspReferenceTape::check_capacity() const {
  if (state_.used == state_.entries.size()) {
    throw Fault("Native reference instruction exceeds its admitted-effect bound");
  }
}

DspNativeAccess DspReferenceTape::decorate(DspNativeAccess access) {
  access.pc = state_.context.pc;
  access.omr = state_.context.omr;
  access.ordinal = ordinal_++;
  return access;
}

std::uint32_t DspReferenceTape::sample(const DspReferenceEffect &effect) {
  if (!state_.active || mode_ != DspReferenceMode::attempt) {
    throw Fault("Unexpected native " + effect_context(effect) +
                " while restoring an instruction base");
  }
  if (cursor_ < state_.used) {
    const auto *saved = std::get_if<DspReferenceRecord>(&state_.entries.at(cursor_++));
    if (saved == nullptr || !same_effect(saved->effect, effect)) {
      throw Fault("Native recomputation changed admitted " + effect_context(effect));
    }
    return saved->value;
  }
  if (state_.used == state_.entries.size()) {
    throw Fault("Native effect bound exceeded before " + effect_context(effect));
  }
  state_.pending = effect;
  throw DspReferenceYield{};
}

std::uint32_t DspReferenceTape::read(const DspAddress target, const DspNativeAccessKind kind) {
  if (kind == DspNativeAccessKind::vector) {
    throw Fault("Unqualified native reference interrupt-vector read at P:" +
                hex(target.address, 6));
  }
  return sample(decorate({.kind = kind, .target = target}));
}

void DspReferenceTape::write(const DspStore target) {
  static_cast<void>(sample(decorate({.kind = DspNativeAccessKind::write,
                                     .target = {.space = target.space, .address = target.address},
                                     .value = target.value})));
}

bool DspReferenceTape::bit(const DspBitAccess target) {
  return sample(decorate(
             {.kind = DspNativeAccessKind::bit,
              .target = {.space = target.space, .address = target.address},
              .bit = DspNativeAccessBit{.index = target.bit, .operation = target.operation}})) != 0;
}

void DspReferenceTape::reset_peripherals() {
  static_cast<void>(sample(DspPeripheralReset{}));
}

std::uint32_t DspReferenceTape::peek_program(const std::uint32_t address) {
  if (!state_.active || mode_ != DspReferenceMode::attempt) {
    throw Fault("Unexpected native decoder lookahead at P:" + hex(address, 6) +
                " while restoring an instruction base");
  }
  if (cursor_ < state_.used) {
    const auto *saved = std::get_if<DspReferenceLookahead>(&state_.entries.at(cursor_++));
    if (saved == nullptr || saved->address != address) {
      throw Fault("Native recomputation changed decoder lookahead at P:" + hex(address, 6));
    }
    return saved->value;
  }
  check_capacity();
  const auto value = bus_.peek_program(address);
  state_.entries.at(state_.used++) = DspReferenceLookahead{.address = address, .value = value};
  ++cursor_;
  return value;
}

std::uint32_t DspReferenceTape::perform(const DspReferenceEffect &effect) {
  const auto *access = std::get_if<DspNativeAccess>(&effect);
  if (access == nullptr) {
    bus_.reset_peripherals();
    return 0;
  }
  if (access->kind == DspNativeAccessKind::write) {
    if (!access->value) { throw Fault("Native reference write lost its source value"); }
    bus_.write({.space = access->target.space,
                .address = access->target.address,
                .value = *access->value});
    return *access->value;
  }
  if (access->kind != DspNativeAccessKind::bit) { return bus_.read(access->target); }
  if (!access->bit) { throw Fault("Native reference bit request lost its operation"); }
  const auto value = bus_.access_peripheral_bit({.space = access->target.space,
                                                 .address = access->target.address,
                                                 .bit = access->bit->index,
                                                 .operation = access->bit->operation});
  if (!value) { throw Fault("Native reference bit route changed before admission"); }
  return *value ? 1U : 0U;
}

DspReferenceEffect DspReferenceTape::commit() {
  if (!state_.active || !state_.pending || mode_ != DspReferenceMode::ordinary) {
    throw Fault("Native reference admission requires an inactive pending access");
  }
  const auto effect = *state_.pending;
  std::uint32_t value = 0;
  try {
    value = perform(effect);
  } catch (const std::exception &error) {
    throw Fault("Native reference " + effect_context(effect) + ": " + error.what());
  }
  state_.entries.at(state_.used++) = DspReferenceRecord{.effect = effect, .value = value};
  state_.pending.reset();
  auto completed = effect;
  if (auto *access = std::get_if<DspNativeAccess>(&completed)) {
    access->stage = DspNativeAccessStage::after;
    access->value = value;
  }
  return completed;
}

DspReferenceScope::DspReferenceScope(DspReferenceTape &tape, const DspReferenceMode mode)
    : tape_(tape), previous_(tape.mode()) {
  tape_.set_mode(mode);
}

DspReferenceScope::~DspReferenceScope() {
  tape_.set_mode(previous_);
}
}
