// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_io.hpp"

#include "dsp/dsp_bus.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <cstddef>
#include <exception>
#include <span>
#include <stdexcept>

namespace blofeld {
namespace {
DspSpace board_space(const dsp56300::Space space) {
  switch (space) {
    case dsp56300::Space::program: return DspSpace::p;
    case dsp56300::Space::x: return DspSpace::x;
    case dsp56300::Space::y: return DspSpace::y;
    default: throw std::invalid_argument("Unknown DSP address space");
  }
}

DspBitOperation board_action(const dsp56300::BitAction action) {
  switch (action) {
    case dsp56300::BitAction::test: return DspBitOperation::test;
    case dsp56300::BitAction::clear: return DspBitOperation::clear;
    case dsp56300::BitAction::set: return DspBitOperation::set;
    case dsp56300::BitAction::change: return DspBitOperation::change;
    default: throw std::invalid_argument("Unknown DSP bit operation");
  }
}
}

std::span<const dsp56300::MemoryWindow>
DspIo::memory_windows(const dsp56300::Space space) noexcept {
  if ((space != dsp56300::Space::x && space != dsp56300::Space::y) || failure_) { return {}; }
  const bool x = space == dsp56300::Space::x;
  const auto source = bus_.data_windows(x ? DspSpace::x : DspSpace::y);
  auto &target = windows_.at(x ? 0U : 1U);
  for (std::size_t index = 0; index < source.size(); ++index) {
    const auto &entry = source.at(index);
    target.at(index) = {.address = entry.address,
                        .words = entry.words,
                        .known = entry.known,
                        .writable = entry.writable};
  }
  return target;
}

bool DspIo::fail(const Operation operation,
                 const dsp56300::Space space,
                 const dsp56300::Word address,
                 const dsp56300::Word pc,
                 const dsp56300::Word value,
                 const dsp56300::BitAction action) noexcept {
  if (!failure_) {
    failure_ = Failure{.operation = operation,
                       .space = space,
                       .address = address,
                       .pc = pc,
                       .value = value,
                       .action = action,
                       .cause = std::current_exception()};
  }
  return false;
}

void DspIo::prepare_peripheral(const DspAddress target, const unsigned pc) const {
  constexpr unsigned peripheral_base = 0xffff80;
  if (target.space != DspSpace::p && target.address >= peripheral_base &&
      peripheral_observer_.preparing != nullptr) {
    peripheral_observer_.preparing(peripheral_observer_.context, pc);
  }
}

void DspIo::observe_peripheral(const DspAddress target,
                               const unsigned pc,
                               const bool reschedule) const {
  constexpr unsigned peripheral_base = 0xffff80;
  if (target.space != DspSpace::p && target.address >= peripheral_base &&
      peripheral_observer_.completed != nullptr) {
    peripheral_observer_.completed(peripheral_observer_.context, pc, reschedule);
  }
}

bool DspIo::read(const dsp56300::Space space,
                 const dsp56300::Word address,
                 const dsp56300::Word pc,
                 dsp56300::Word &value) noexcept {
  if (failure_) { return false; }
  try {
    const DspAddress target{.space = board_space(space), .address = address};
    prepare_peripheral(target, pc);
    value = bus_.read(target);
    observe_peripheral({.space = board_space(space), .address = address}, pc);
    if (space == dsp56300::Space::program && program_observer_.completed != nullptr) {
      program_observer_.completed(program_observer_.context, address);
    }
    return true;
  } catch (...) { return fail(Operation::read, space, address, pc); }
}

bool DspIo::write(const dsp56300::Space space,
                  const dsp56300::Word address,
                  const dsp56300::Word pc,
                  const dsp56300::Word value) noexcept {
  if (failure_) { return false; }
  try {
    const DspStore store{.space = board_space(space), .address = address, .value = value};
    prepare_peripheral({.space = store.space, .address = store.address}, pc);
    bus_.write(store);
    observe_peripheral({.space = board_space(space), .address = address}, pc, true);

    return true;
  } catch (...) { return fail(Operation::write, space, address, pc, value); }
}

bool DspIo::bit(const dsp56300::Space space,
                const dsp56300::Word address,
                const dsp56300::Word pc,
                const dsp56300::Word bit,
                const dsp56300::BitAction action,
                bool &previous) noexcept {
  if (failure_) { return false; }
  try {
    prepare_peripheral({.space = board_space(space), .address = address}, pc);
    const auto actual = bus_.access_peripheral_bit({.space = board_space(space),
                                                    .address = address,
                                                    .bit = bit,
                                                    .operation = board_action(action)});
    if (actual) {
      previous = *actual;
      observe_peripheral({.space = board_space(space), .address = address},
                         pc,
                         action != dsp56300::BitAction::test);
      return true;
    }
    if (action != dsp56300::BitAction::test) {
      return fail(Operation::bit, space, address, pc, bit, action);
    }
    const auto value = bus_.read({.space = board_space(space), .address = address});
    previous = (value & (dsp56300::Word{1} << bit)) != 0;
    observe_peripheral({.space = board_space(space), .address = address}, pc);
    return true;
  } catch (...) { return fail(Operation::bit, space, address, pc, bit, action); }
}

bool DspIo::reset_peripherals(const dsp56300::Word pc) noexcept {
  if (failure_) { return false; }
  try {
    if (peripheral_observer_.preparing != nullptr) {
      peripheral_observer_.preparing(peripheral_observer_.context, pc);
    }
    bus_.reset_peripherals();
    if (peripheral_observer_.completed != nullptr) {
      peripheral_observer_.completed(peripheral_observer_.context, pc, true);
    }
    return true;
  } catch (...) { return fail(Operation::reset, dsp56300::Space::program, pc, pc); }
}
} // namespace blofeld
