// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_dsp_schedule.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <string>
#include <tuple>

namespace blofeld {
class McuDspSchedule::Guard {
public:
  explicit Guard(McuDspSchedule &schedule)
      : schedule_(schedule), exceptions_(std::uncaught_exceptions()) {
    schedule_.active_ = true;
  }

  ~Guard() {
    schedule_.active_ = false;
    if (std::uncaught_exceptions() > exceptions_) { schedule_.faulted_ = true; }
  }

  Guard(const Guard &) = delete;
  Guard &operator=(const Guard &) = delete;

private:
  McuDspSchedule &schedule_;
  int exceptions_;
};

namespace {
bool same_cpu(const CpuState &first, const CpuState &second) {
  return std::tie(first.pc, first.d, first.x, first.y, first.sp, first.ccr) ==
         std::tie(second.pc, second.d, second.x, second.y, second.sp, second.ccr);
}
}

McuDspScheduleCheckpoint::McuDspScheduleCheckpoint(const DspScheduleCheckpoint &dsp) : dsp_(dsp) {}

McuDspSchedule::McuDspSchedule(const McuDspScheduleInputs inputs, McuMachine &mcu, DspMachine &dsp)
    : inputs_(inputs),
      mcu_(mcu),
      dsp_(dsp),
      dsp_schedule_(inputs.dsp, dsp),
      timebase_hz_(dsp.run_clock().timebase_hz()),
      end_ticks_(mcu_ticks()),
      expected_mcu_cycles_(mcu.run_clock().bus_cycles()) {
  mcu_.validate_run_boundary();
  if (inputs.maximum_dsp_steps_per_cycle == 0 ||
      (inputs.shared_write_order != McuDspTieOrder::unqualified &&
       inputs.shared_write_order != McuDspTieOrder::dsp_before_mcu &&
       inputs.shared_write_order != McuDspTieOrder::mcu_before_dsp)) {
    throw Fault("MCU/DSP schedule requires an explicit cycle budget and known tie contract");
  }
  if (dsp_schedule_.horizon_ticks() > end_ticks_ && dsp.run_clock().elapsed_ticks() != 0) {
    throw Fault("MCU/DSP schedule starts beyond its MCU horizon with an already running DSP");
  }
}

bool McuDspSchedule::faulted() const {
  return faulted_ || mcu_.timing_faulted() || dsp_.timing_faulted();
}

void McuDspSchedule::healthy() const {
  if (active_ || faulted()) {
    throw Fault("MCU/DSP schedule requires a healthy inactive boundary");
  }
}

std::uint64_t McuDspSchedule::mcu_ticks() const {
  const auto &clock = mcu_.run_clock();
  if (timebase_hz_ < clock.timebase_hz() || timebase_hz_ % clock.timebase_hz() != 0) {
    throw Fault("MCU/DSP reference timebase conversion must be an exact positive integer");
  }
  const auto factor = timebase_hz_ / clock.timebase_hz();
  if (clock.elapsed_ticks() > std::numeric_limits<std::uint64_t>::max() / factor) {
    throw Fault("MCU/DSP common time conversion exhausted");
  }
  return clock.elapsed_ticks() * factor;
}

void McuDspSchedule::consumed() const {
  if (mcu_ticks() != end_ticks_ || mcu_.run_clock().bus_cycles() != expected_mcu_cycles_) {
    throw Fault("MCU/DSP schedule requires every offered native MCU period to be consumed");
  }
}

void McuDspSchedule::advance(const DspScheduleBoundary boundary, const DspObserver &before_dsp) {
  if (end_ticks_ < inputs_.dsp.epoch_ticks) { return; }
  try {
    dsp_schedule_.advance_to(
        dsp_, end_ticks_, boundary, inputs_.maximum_dsp_steps_per_cycle, before_dsp);
  } catch (const Fault &fault) {
    throw Fault("Scheduled peer execution mcu-pc=" + hex(cycle_.context.pc) + " dsp-pc=" +
                hex(dsp_.cpu().state().pc, 6) + " mcu-cycle-kind=" + std::string(1, cycle_.kind) +
                " mcu-address=" + (cycle_.address ? hex(*cycle_.address) : "none") +
                " mcu-end=" + std::to_string(end_ticks_) + ": " + fault.what());
  }
}

void McuDspSchedule::begin_cycle(const Hc12BusCycle cycle,
                                 const bool shared_write,
                                 const DspObserver &before_dsp) {
  healthy();
  const Guard guard(*this);
  consumed();
  if (shared_write_) { throw Fault("MCU/DSP shared write observation was not delivered"); }
  const bool write = cycle.kind == 'w' || cycle.kind == 'W';
  if (shared_write && (!write || !cycle.address || cycle.size != 1)) {
    throw Fault("Unqualified MCU/DSP shared write cycle geometry");
  }
  const auto factor = timebase_hz_ / mcu_.run_clock().timebase_hz();
  const auto period = mcu_.run_clock().next_cycle_ticks(mcu_.bus());
  if (expected_mcu_cycles_ == std::numeric_limits<std::uint64_t>::max() ||
      period > (std::numeric_limits<std::uint64_t>::max() - end_ticks_) / factor) {
    throw Fault("MCU/DSP next native MCU period exhausted common time");
  }
  cycle_ = cycle;
  end_ticks_ += period * factor;
  ++expected_mcu_cycles_;
  offered_ = true;
  shared_write_ = shared_write;
  if (!shared_write) {
    advance(DspScheduleBoundary::through, before_dsp);
    return;
  }
  advance(DspScheduleBoundary::before, before_dsp);
  const bool tie = end_ticks_ >= inputs_.dsp.epoch_ticks &&
                   dsp_schedule_.next_completion_ticks(dsp_) == end_ticks_;
  if (tie) {
    if (ties_ == std::numeric_limits<std::uint64_t>::max()) {
      throw Fault("MCU/DSP shared write tie counter exhausted");
    }
    ++ties_;
    if (inputs_.shared_write_order == McuDspTieOrder::unqualified) {
      throw Fault("Unqualified simultaneous DSP completion and MCU shared write at " +
                  hex(*cycle.address) + " mcu-pc=" + hex(cycle.context.pc) +
                  " dsp-pc=" + hex(dsp_.cpu().state().pc, 6));
    }
  }
  if (!tie || inputs_.shared_write_order != McuDspTieOrder::mcu_before_dsp) {
    advance(DspScheduleBoundary::through, before_dsp);
  }
}

void McuDspSchedule::observe_write(const BusEvent &event,
                                   const std::function<void()> &observe_signals,
                                   const DspObserver &before_dsp) {
  healthy();
  const Guard guard(*this);
  consumed();
  if (!offered_ || !shared_write_ || !event.write || !cycle_.address ||
      event.address != *cycle_.address || !observe_signals) {
    throw Fault("MCU/DSP shared observation must match its completed native write cycle");
  }
  observe_signals();
  consumed();
  advance(DspScheduleBoundary::through, before_dsp);
  shared_write_ = false;
}

void McuDspSchedule::end_instruction() {
  healthy();
  const Guard guard(*this);
  mcu_.validate_run_boundary();
  consumed();
  if (shared_write_) { throw Fault("MCU/DSP shared write observation was not delivered"); }
  if (end_ticks_ >= inputs_.dsp.epoch_ticks &&
      (dsp_schedule_.horizon_ticks() != end_ticks_ ||
       dsp_schedule_.next_completion_ticks(dsp_) <= end_ticks_)) {
    throw Fault("MCU/DSP completed instruction lost its pending DSP deadline");
  }
  offered_ = false;
}

McuDspScheduleCheckpoint McuDspSchedule::checkpoint() const {
  healthy();
  mcu_.validate_run_boundary();
  consumed();
  if (offered_ || shared_write_) {
    throw Fault("MCU/DSP schedule capture requires a completed instruction observation");
  }
  McuDspScheduleCheckpoint result(dsp_schedule_.checkpoint(dsp_));
  result.inputs_ = inputs_;
  result.mcu_ = mcu_.cpu().state();
  result.timebase_hz_ = timebase_hz_;
  result.end_ticks_ = end_ticks_;
  result.mcu_cycles_ = expected_mcu_cycles_;
  result.ties_ = ties_;
  return result;
}

void McuDspSchedule::restore(const McuDspScheduleCheckpoint &checkpoint) {
  if (active_) { throw Fault("Cannot restore an active MCU/DSP schedule"); }
  mcu_.validate_run_boundary();
  if (checkpoint.inputs_ != inputs_ || checkpoint.timebase_hz_ != timebase_hz_ ||
      mcu_ticks() != checkpoint.end_ticks_ ||
      mcu_.run_clock().bus_cycles() != checkpoint.mcu_cycles_ ||
      !same_cpu(mcu_.cpu().state(), checkpoint.mcu_)) {
    throw Fault("MCU/DSP schedule restore requires compatible inputs and coordinated native MCU");
  }
  dsp_schedule_.restore(checkpoint.dsp_, dsp_);
  end_ticks_ = checkpoint.end_ticks_;
  expected_mcu_cycles_ = checkpoint.mcu_cycles_;
  ties_ = checkpoint.ties_;
  offered_ = false;
  shared_write_ = false;
  faulted_ = false;
}
}
