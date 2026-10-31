// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/own_dsp_io.hpp"

#include "dsp/dsp_bus.hpp"
#include "dsp56300/core.hpp"

#include <exception>
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

bool OwnDspIo::fail(const Operation operation,
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

bool OwnDspIo::read(const dsp56300::Space space,
                    const dsp56300::Word address,
                    const dsp56300::Word pc,
                    dsp56300::Word &value) noexcept {
  if (failure_) { return false; }
  try {
    value = bus_.read({.space = board_space(space), .address = address});
    return true;
  } catch (...) { return fail(Operation::read, space, address, pc); }
}

bool OwnDspIo::write(const dsp56300::Space space,
                     const dsp56300::Word address,
                     const dsp56300::Word pc,
                     const dsp56300::Word value) noexcept {
  if (failure_) { return false; }
  try {
    bus_.write({.space = board_space(space), .address = address, .value = value});
    return true;
  } catch (...) { return fail(Operation::write, space, address, pc, value); }
}

bool OwnDspIo::bit(const dsp56300::Space space,
                   const dsp56300::Word address,
                   const dsp56300::Word pc,
                   const dsp56300::Word bit,
                   const dsp56300::BitAction action,
                   bool &previous) noexcept {
  if (failure_) { return false; }
  try {
    const auto actual = bus_.access_peripheral_bit({.space = board_space(space),
                                                    .address = address,
                                                    .bit = bit,
                                                    .operation = board_action(action)});
    if (actual) {
      previous = *actual;
      return true;
    }
    if (action != dsp56300::BitAction::test) {
      return fail(Operation::bit, space, address, pc, bit, action);
    }
    const auto value = bus_.read({.space = board_space(space), .address = address});
    previous = (value & (dsp56300::Word{1} << bit)) != 0;
    return true;
  } catch (...) { return fail(Operation::bit, space, address, pc, bit, action); }
}

bool OwnDspIo::reset_peripherals(const dsp56300::Word pc) noexcept {
  if (failure_) { return false; }
  try {
    bus_.reset_peripherals();
    return true;
  } catch (...) { return fail(Operation::reset, dsp56300::Space::program, pc, pc); }
}
} // namespace blofeld
