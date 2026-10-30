// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_flash.hpp"
#include "mcu/mcu_flash_timing.hpp"
#include "mcu/mcu_flash_timing_state.hpp"

#include <cstdint>
#include <exception>
#include <optional>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned program = 0x20, sector = 0x40, mass = 0x41, verify = 5;
constexpr unsigned row_bytes = 64, array_bytes = 32768;

bool same_rates(const FlashClockBoundary &a, const FlashClockBoundary &b) {
  return a.timebase_hz == b.timebase_hz && a.oscillator_hz == b.oscillator_hz &&
         a.bus_period_ticks == b.bus_period_ticks;
}
}

void McuFlash::configure_timing(const FlashTimingInputs inputs) {
  if (inputs == timing_.inputs) { return; }
  require_running();
  if (active_ || buffer_state_ != BufferState::empty || timing_.clock) {
    throw Fault("MCU flash timing profile requires a fresh inactive owner");
  }
  if (inputs.contract != FlashTimingContract::uf32_formula_candidate ||
      !inputs.transfer_bus_cycles || *inputs.transfer_bus_cycles == 0 ||
      (inputs.verify_unit != FlashVerifyUnit::unqualified &&
       inputs.verify_unit != FlashVerifyUnit::words &&
       inputs.verify_unit != FlashVerifyUnit::bytes)) {
    throw Fault("MCU flash timing profile requires explicit qualified input hypotheses");
  }
  timing_.inputs = inputs;
  clock_.reset();
}

FlashTimeBasis McuFlash::timing_basis() const {
  if (!timing_.clock) { throw Fault("MCU flash has no native clock basis"); }
  const auto &c = *timing_.clock;
  return {.timebase_hz = c.timebase_hz,
          .oscillator_hz = c.oscillator_hz,
          .nvm_divisor = config_.oscillator_divisor().value_or(0),
          .bus_period_ticks = c.bus_period_ticks};
}

FlashTimedOperation McuFlash::timed_operation() const {
  switch (buffered_.command) {
    case program:
      if (active_ && active_->command == program &&
          active_->offset / row_bytes == buffered_.offset / row_bytes) {
        return FlashTimedOperation::same_row_burst;
      }
      return FlashTimedOperation::word_program;
    case sector: return FlashTimedOperation::sector_erase;
    case mass: return FlashTimedOperation::mass_erase;
    case verify:
      if (timing_.inputs.verify_unit == FlashVerifyUnit::words) {
        return FlashTimedOperation::verify_words;
      }
      if (timing_.inputs.verify_unit == FlashVerifyUnit::bytes) {
        return FlashTimedOperation::verify_bytes;
      }
      throw Fault("MCU blank-check timing requires an explicit scan-unit candidate");
    default: throw Fault("Unsupported timed MCU flash command at " + hex(buffered_.offset));
  }
}

unsigned McuFlash::timed_verify_scans(const FlashTimedOperation operation) const {
  if (operation != FlashTimedOperation::verify_words &&
      operation != FlashTimedOperation::verify_bytes) {
    return 0;
  }
  constexpr unsigned erased = 0xff;
  if (operation == FlashTimedOperation::verify_bytes) {
    for (unsigned i = 0; i < array_bytes; ++i) {
      if (memory_.read_flash(static_cast<std::uint16_t>(i)) != erased) { return i + 1; }
    }
    return array_bytes;
  }
  for (unsigned i = 0; i < array_bytes; i += 2) {
    if (memory_.read_flash(static_cast<std::uint16_t>(i)) != erased ||
        memory_.read_flash(static_cast<std::uint16_t>(i + 1)) != erased) {
      return (i / 2) + 1;
    }
  }
  return array_bytes / 2;
}

std::uint64_t McuFlash::timed_duration(const FlashTimedOperation operation) const {
  return flash_duration_ticks(timing_basis(), operation, timed_verify_scans(operation));
}

FlashTimingState McuFlash::plan_launch() const {
  auto next = timing_;
  if (next.inputs.contract == FlashTimingContract::unqualified) {
    if (next.clock && !clock_) {
      throw Fault(
          "Native MCU flash launch requires an explicit timing profile or reference clocks");
    }
    return next;
  }
  if (!next.clock || next.transfer || next.origin || next.pending_operation ||
      static_cast<bool>(active_) != next.completion.has_value()) {
    throw Fault("Inconsistent native MCU flash launch/calendar state");
  }
  const auto operation = timed_operation();
  next.pending_operation = operation;
  if (active_) {
    next.origin = next.completion;
    next.transfer = next.completion;
    // Pending verify observes the array only after the preceding real mutation.
    if (operation != FlashTimedOperation::verify_words &&
        operation != FlashTimedOperation::verify_bytes) {
      next.pending_duration = timed_duration(operation);
    }
  } else {
    if (!next.inputs.transfer_bus_cycles) {
      throw Fault("MCU flash launch lost its explicit transfer-phase candidate");
    }
    next.origin = next.clock->ticks;
    next.pending_duration = timed_duration(operation);
    const auto delay = flash_transfer_ticks(timing_basis(),
                                            {.bus_cycles = *next.inputs.transfer_bus_cycles,
                                             .total_duration_ticks = *next.pending_duration});
    next.transfer = flash_deadline(*next.origin, delay);
    static_cast<void>(flash_deadline(*next.origin, *next.pending_duration));
  }
  return next;
}

void McuFlash::timed_transfer() {
  if (!timing_.origin || !timing_.pending_operation || !timing_.transfer) {
    throw Fault("MCU flash timed transfer lost its accepted job metadata");
  }
  const auto duration = timing_.pending_duration ? *timing_.pending_duration
                                                 : timed_duration(*timing_.pending_operation);
  const auto deadline = flash_deadline(*timing_.origin, duration);
  if (deadline <= *timing_.transfer) { throw Fault("MCU flash transfer cannot follow completion"); }
  transfer();
  timing_.completion = deadline;
  timing_.transfer.reset();
  timing_.origin.reset();
  timing_.pending_duration.reset();
  timing_.pending_operation.reset();
}

std::optional<std::uint64_t> McuFlash::next_timed_event() const {
  require_running();
  if (!timing_.transfer) { return timing_.completion; }
  if (!timing_.completion || *timing_.transfer < *timing_.completion) { return timing_.transfer; }
  return timing_.completion;
}

unsigned McuFlash::advance_time(const FlashClockBoundary boundary) {
  require_running();
  if (timing_.clock && (boundary.ticks < timing_.clock->ticks ||
                        ((active_ || buffer_state_ == BufferState::launched) &&
                         !same_rates(boundary, *timing_.clock)))) {
    throw Fault("MCU flash native clock changed rate or moved backwards while active");
  }
  timing_.clock = boundary;
  unsigned mutations = 0;
  try {
    while (const auto next = next_timed_event()) {
      if (*next > boundary.ticks || (*next == boundary.ticks && !boundary.include_endpoint)) {
        break;
      }
      timing_.clock->ticks = *next;
      if (timing_.completion == next) {
        complete();
        timing_.completion.reset();
        ++mutations;
      }
      if (timing_.transfer == next) {
        timed_transfer();
        ++mutations;
      }
    }
    timing_.clock = boundary;
  } catch (const std::exception &error) {
    timing_.faulted = true;
    throw Fault("MCU flash autonomous event ticks=" + std::to_string(timing_.clock->ticks) + ": " +
                error.what());
  }
  return mutations;
}

void McuFlash::cancel_timing() {
  const auto inputs = timing_.inputs;
  const auto clock = timing_.clock;
  timing_ = {};
  timing_.inputs = inputs;
  timing_.clock = clock;
}
}
