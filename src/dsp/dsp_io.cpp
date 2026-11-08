// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_io.hpp"

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <cstddef>
#include <exception>
#include <span>
#include <stdexcept>

namespace blofeld {
namespace {
bool register_snapshot(const DspPeripheralObserver &observer, const DspAddress target) {
  constexpr unsigned c_direction = 0xffffbe;
  constexpr unsigned c_control = 0xffffbf;
  constexpr unsigned d_direction = 0xffffd6;
  constexpr unsigned d_control = 0xffffd7;
  constexpr unsigned e_direction = 0xffff9e;
  constexpr unsigned e_control = 0xffff9f;
  constexpr unsigned f_direction = 0xfffff6;
  constexpr unsigned f_control = 0xfffff7;
  if (!observer.stable_register_reads) { return false; }
  // GPIO direction/control and interrupt priorities change only through actual
  // CPU writes/RESET. DMA cannot target them. GPIO data remains a live read.
  if (target.space == DspSpace::x) {
    if (DspDma::handles(target.address) || DspInterrupts::handles(target.address)) { return true; }
    switch (target.address) {
      case c_direction:
      case c_control:
      case d_direction:
      case d_control: return true;
      default: return false;
    }
  }
  if (target.space == DspSpace::y) {
    switch (target.address) {
      case e_direction:
      case e_control:
      case f_direction:
      case f_control: return true;
      default: return false;
    }
  }
  return false;
}

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
    auto cause = std::current_exception();
    const bool read = operation == Operation::read ||
                      (operation == Operation::bit && action == dsp56300::BitAction::test);
    const DspAddress target{.space = space == dsp56300::Space::y ? DspSpace::y : DspSpace::x,
                            .address = address};
    if (read && (space == dsp56300::Space::x || space == dsp56300::Space::y) &&
        register_snapshot(peripheral_observer_, target)) {
      try {
        prepare_peripheral(target, pc);
      } catch (...) { cause = std::current_exception(); }
    }
    failure_ = Failure{.operation = operation,
                       .space = space,
                       .address = address,
                       .pc = pc,
                       .value = value,
                       .action = action,
                       .cause = cause};
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
    const bool snapshot = register_snapshot(peripheral_observer_, target);
    if (!snapshot) { prepare_peripheral(target, pc); }
    value = bus_.read(target);
    if (!snapshot) { observe_peripheral(target, pc); }
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
    const DspAddress target{.space = board_space(space), .address = address};
    const bool snapshot =
        action == dsp56300::BitAction::test && register_snapshot(peripheral_observer_, target);
    if (!snapshot) { prepare_peripheral(target, pc); }
    const auto actual = bus_.access_peripheral_bit({.space = board_space(space),
                                                    .address = address,
                                                    .bit = bit,
                                                    .operation = board_action(action)});
    if (actual) {
      previous = *actual;
      if (!snapshot) { observe_peripheral(target, pc, action != dsp56300::BitAction::test); }
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
