// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_reference_execution.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_reference_execution.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <optional>
#include <string>
#include <variant>

namespace blofeld {
namespace {
constexpr unsigned word_mask = 0xffffff;

bool same_access(const DspNativeAccess &a, const DspNativeAccess &b) {
  return a.kind == b.kind && a.stage == b.stage && a.target.space == b.target.space &&
         a.target.address == b.target.address && a.value == b.value && a.bit == b.bit &&
         a.ordinal == b.ordinal && a.pc == b.pc && a.omr == b.omr;
}

bool same_effect(const DspReferenceEffect &a, const DspReferenceEffect &b) {
  if (a.index() != b.index()) { return false; }
  if (const auto *ack = std::get_if<DspInterruptAcknowledge>(&a)) {
    const auto other = std::get<DspInterruptAcknowledge>(b).request;
    return ack->request.vector == other.vector && ack->request.level == other.level;
  }
  const auto *access = std::get_if<DspNativeAccess>(&a);
  return access == nullptr || same_access(*access, std::get<DspNativeAccess>(b));
}

std::string effect_context(const DspReferenceEffect &effect) {
  if (const auto *ack = std::get_if<DspInterruptAcknowledge>(&effect)) {
    return "interrupt-acknowledge vector=" + hex(ack->request.vector, 6) +
           " level=" + std::to_string(ack->request.level);
  }
  const auto *access = std::get_if<DspNativeAccess>(&effect);
  if (access == nullptr) { return "peripheral-reset"; }
  constexpr std::array names{"opcode", "extension", "read", "write", "bit", "vector"};
  constexpr std::array spaces{"P:", "X:", "Y:"};
  return std::string(names.at(static_cast<unsigned>(access->kind))) + " at " +
         spaces.at(static_cast<unsigned>(access->target.space)) + hex(access->target.address, 6) +
         " ordinal=" + std::to_string(access->ordinal);
}

bool program_read(const DspReferenceEffect &effect) {
  const auto *access = std::get_if<DspNativeAccess>(&effect);
  return access != nullptr && access->target.space == DspSpace::p &&
         access->stage == DspNativeAccessStage::before &&
         (access->kind == DspNativeAccessKind::opcode ||
          access->kind == DspNativeAccessKind::extension);
}
}

void DspReferenceTape::begin(const DspNativeAccess context,
                             const std::optional<DspReferenceInterrupt> interrupt) {
  if (state_.active) { throw Fault("Native reference instruction is already pending"); }
  finish();
  state_.context = context;
  state_.interrupt = interrupt;
  state_.active = true;
}

void DspReferenceTape::clear() {
  state_ = {};
  cursor_ = 0;
  ordinal_ = 0;
  mode_ = DspReferenceMode::ordinary;
}

void DspReferenceTape::finish() {
  const auto fetch = state_.fetch;
  clear();
  state_.fetch = fetch;
}

std::optional<std::uint32_t> DspReferenceTape::latched_program_word() const {
  if (state_.fetch.active() && state_.pending && program_read(*state_.pending)) {
    return state_.fetch.peek(std::get<DspNativeAccess>(*state_.pending).target.address);
  }
  return state_.latched_program_word;
}

void DspReferenceTape::check_admission() const {
  if (state_.fetch.active() && state_.pending && program_read(*state_.pending)) {
    state_.fetch.check_consume(std::get<DspNativeAccess>(*state_.pending).target.address);
  }
}

std::optional<std::uint32_t> DspReferenceTape::opcode() const {
  for (std::size_t index = 0; index < state_.used; ++index) {
    const auto *record = std::get_if<DspReferenceRecord>(&state_.entries.at(index));
    if (record == nullptr) { continue; }
    const auto *access = std::get_if<DspNativeAccess>(&record->effect);
    if (access != nullptr && access->kind == DspNativeAccessKind::opcode) { return record->value; }
  }
  return std::nullopt;
}

void DspReferenceTape::restore(const DspReferenceState &state) {
  if (state.used > state.entries.size() || (state.active != state.pending.has_value()) ||
      (state.pending && state.used == state.entries.size()) ||
      (state.latched_program_word && (!state.pending || !program_read(*state.pending) ||
                                      *state.latched_program_word > word_mask))) {
    throw Fault("Invalid trusted native reference continuation");
  }
  state_ = state;
  cursor_ = 0;
  ordinal_ = 0;
  mode_ = DspReferenceMode::ordinary;
}

const DspReferenceEffect &DspReferenceTape::check_program_latch() const {
  if (!state_.active || !state_.pending || mode_ != DspReferenceMode::ordinary) {
    throw Fault("Program latch requires a pending native program read");
  }
  if (state_.fetch.active() || state_.latched_program_word || !program_read(*state_.pending)) {
    throw Fault("Unavailable program latch for native " + effect_context(*state_.pending) +
                " pc=" + hex(state_.context.pc, 6) + " omr=" + hex(state_.context.omr, 6));
  }
  return *state_.pending;
}

void DspReferenceTape::latch_program() {
  const auto effect = check_program_latch();
  try {
    state_.latched_program_word = perform(effect);
  } catch (const std::exception &error) {
    throw Fault("Native program latch " + effect_context(effect) + ": " + error.what());
  }
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
  if (kind == DspNativeAccessKind::vector && !state_.interrupt) {
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

void DspReferenceTape::acknowledge_interrupt(const DspInterruptRequest request) {
  static_cast<void>(sample(DspInterruptAcknowledge{.request = request}));
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
  const auto fetched = state_.fetch.peek(address);
  const auto value = fetched ? *fetched : bus_.peek_program(address);
  state_.entries.at(state_.used++) = DspReferenceLookahead{.address = address, .value = value};
  ++cursor_;
  return value;
}

std::uint32_t DspReferenceTape::perform(const DspReferenceEffect &effect) {
  if (const auto *ack = std::get_if<DspInterruptAcknowledge>(&effect)) {
    bus_.acknowledge_latched_interrupt(ack->request);
    return 0;
  }
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
  check_admission();
  std::uint32_t value = 0;
  try {
    if (state_.fetch.active() && program_read(effect)) {
      value = state_.fetch.consume(std::get<DspNativeAccess>(effect).target.address);
    } else {
      value = state_.latched_program_word ? *state_.latched_program_word : perform(effect);
    }
  } catch (const std::exception &error) {
    throw Fault("Native reference " + effect_context(effect) + ": " + error.what());
  }
  state_.entries.at(state_.used++) = DspReferenceRecord{.effect = effect, .value = value};
  state_.pending.reset();
  state_.latched_program_word.reset();
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
