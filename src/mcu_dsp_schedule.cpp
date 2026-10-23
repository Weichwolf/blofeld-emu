// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_dsp_schedule.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_cop.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_reset.hpp"
#include "blofeld/mcu_run_clock.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <string>
#include <string_view>
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
      expected_mcu_cycles_(mcu.run_clock().bus_cycles()),
      expected_mcu_epoch_(mcu.run_clock().native_epoch()),
      expected_idle_cycles_(mcu.idle_bus_cycles()) {
  mcu_.validate_run_boundary();
  if (inputs.maximum_dsp_steps_per_cycle == 0 ||
      (inputs.shared_write_order != McuDspTieOrder::unqualified &&
       inputs.shared_write_order != McuDspTieOrder::dsp_before_mcu &&
       inputs.shared_write_order != McuDspTieOrder::mcu_before_dsp) ||
      (inputs.wait_order != McuDspWaitOrder::unqualified &&
       inputs.wait_order != McuDspWaitOrder::dsp_before_mcu)) {
    throw Fault("MCU/DSP schedule requires an explicit cycle budget and known tie contract");
  }
  if (dsp_schedule_.horizon_ticks() > end_ticks_ && dsp.run_clock().elapsed_ticks() != 0) {
    throw Fault("MCU/DSP schedule starts beyond its MCU horizon with an already running DSP");
  }
}

bool McuDspSchedule::faulted() const {
  return faulted_ || mcu_.timing_faulted() || dsp_.timing_faulted() ||
         mcu_.run_clock().native_epoch() != expected_mcu_epoch_;
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
  if (mcu_ticks() != end_ticks_ || mcu_.run_clock().bus_cycles() != expected_mcu_cycles_ ||
      mcu_.idle_bus_cycles() != expected_idle_cycles_) {
    throw Fault("MCU/DSP schedule requires every offered native MCU period to be consumed");
  }
}

void McuDspSchedule::advance(const DspScheduleBoundary boundary,
                             const DspObserver &before_dsp,
                             const DspCompletionObserver &after_dsp,
                             const DspExternalEvents &external) {
  if (end_ticks_ < inputs_.dsp.epoch_ticks) { return; }
  try {
    dsp_schedule_.advance_to(dsp_,
                             end_ticks_,
                             boundary,
                             inputs_.maximum_dsp_steps_per_cycle,
                             before_dsp,
                             after_dsp,
                             external);
  } catch (const Fault &fault) {
    if (reset_context_) {
      throw Fault("Scheduled reset peer execution mcu-request-pc=" + hex(reset_context_->pc) +
                  " dsp-pc=" + hex(dsp_.cpu().state().pc, 6) +
                  " mcu-end=" + std::to_string(end_ticks_) + ": " + fault.what());
    }
    if (wait_context_) {
      throw Fault("Scheduled idle peer execution mcu-pc=" + hex(wait_context_->pc) +
                  " dsp-pc=" + hex(dsp_.cpu().state().pc, 6) +
                  " mcu-idle-period=" + std::to_string(expected_idle_cycles_) +
                  " mcu-end=" + std::to_string(end_ticks_) + ": " + fault.what());
    }
    throw Fault("Scheduled peer execution mcu-pc=" + hex(cycle_.context.pc) + " dsp-pc=" +
                hex(dsp_.cpu().state().pc, 6) + " mcu-cycle-kind=" + std::string(1, cycle_.kind) +
                " mcu-address=" + (cycle_.address ? hex(*cycle_.address) : "none") +
                " mcu-end=" + std::to_string(end_ticks_) + ": " + fault.what());
  }
}

void McuDspSchedule::offer_period(const bool waiting) {
  const auto factor = timebase_hz_ / mcu_.run_clock().timebase_hz();
  const auto period = mcu_.run_clock().next_period(mcu_.bus(), waiting);
  if (expected_mcu_cycles_ == std::numeric_limits<std::uint64_t>::max() ||
      period.duration_ticks > (std::numeric_limits<std::uint64_t>::max() - end_ticks_) / factor) {
    throw Fault("MCU/DSP next native MCU period exhausted common time");
  }
  end_ticks_ += period.duration_ticks * factor;
  cop_deadline_ = period.cop_deadline;
  if (!cop_deadline_) { ++expected_mcu_cycles_; }
}

void McuDspSchedule::begin_cycle(const Hc12BusCycle cycle,
                                 const bool shared_write,
                                 const DspObserver &before_dsp,
                                 const DspCompletionObserver &after_dsp,
                                 const DspExternalEvents &external) {
  healthy();
  const Guard guard(*this);
  consumed();
  if (resetting_) {
    const auto reset = mcu_.system_reset_state();
    if (!reset || reset->phase != McuResetPhase::ready || shared_write || cycle.context.pc != 0 ||
        cycle.context.ccr != hc12::reset_ccr ||
        std::string_view("VfP").find(cycle.kind) == std::string_view::npos) {
      throw Fault(
          "MCU/DSP reset permits only the native CPU vector sequence after pad arbitration");
    }
  }
  if (waiting_) { throw Fault("MCU/DSP wait slice requires a completed idle boundary"); }
  if (shared_write_) { throw Fault("MCU/DSP shared write observation was not delivered"); }
  const bool write = cycle.kind == 'w' || cycle.kind == 'W';
  if (shared_write && (!write || !cycle.address || cycle.size != 1)) {
    throw Fault("Unqualified MCU/DSP shared write cycle geometry");
  }
  cycle_ = cycle;
  offer_period();
  offered_ = true;
  shared_write_ = shared_write && !cop_deadline_;
  if (!shared_write_) {
    advance(DspScheduleBoundary::through, before_dsp, after_dsp, external);
    return;
  }
  advance(DspScheduleBoundary::before, before_dsp, after_dsp, external);
  const bool tie =
      end_ticks_ >= inputs_.dsp.epoch_ticks && dsp_schedule_.next_event_ticks(dsp_) == end_ticks_;
  if (tie) {
    if (ties_ == std::numeric_limits<std::uint64_t>::max()) {
      throw Fault("MCU/DSP shared write tie counter exhausted");
    }
    ++ties_;
    if (inputs_.shared_write_order == McuDspTieOrder::unqualified) {
      const std::string event = dsp_schedule_.next_completion_ticks(dsp_) == end_ticks_
                                    ? "DSP completion"
                                    : "external event";
      throw Fault("Unqualified simultaneous " + event + " and MCU shared write at " +
                  hex(*cycle.address) + " mcu-pc=" + hex(cycle.context.pc) +
                  " dsp-pc=" + hex(dsp_.cpu().state().pc, 6));
    }
  }
  if (!tie || inputs_.shared_write_order != McuDspTieOrder::mcu_before_dsp) {
    advance(DspScheduleBoundary::through, before_dsp, after_dsp, external);
  }
}

void McuDspSchedule::observe_write(const BusEvent &event,
                                   const std::function<void()> &observe_signals,
                                   const DspObserver &before_dsp,
                                   const DspCompletionObserver &after_dsp,
                                   const DspExternalEvents &external) {
  healthy();
  const Guard guard(*this);
  consumed();
  if (!offered_ || !shared_write_ || !event.write || !cycle_.address ||
      event.address != *cycle_.address || !observe_signals) {
    throw Fault("MCU/DSP shared observation must match its completed native write cycle");
  }
  observe_signals();
  consumed();
  advance(DspScheduleBoundary::through, before_dsp, after_dsp, external);
  shared_write_ = false;
}

void McuDspSchedule::end_instruction() {
  healthy();
  const Guard guard(*this);
  if (resetting_) { throw Fault("MCU/DSP system reset requires explicit reset completion"); }
  if (waiting_) { throw Fault("MCU/DSP wait slice requires an explicit idle completion"); }
  finish_boundary();
}

void McuDspSchedule::begin_wait_period(const McuWaitPeriod period,
                                       const DspObserver &before_dsp,
                                       const DspCompletionObserver &after_dsp,
                                       const DspExternalEvents &external) {
  healthy();
  const Guard guard(*this);
  consumed();
  const auto state = mcu_.cpu().state();
  if (resetting_) { throw Fault("MCU/DSP WAI cannot clock an asserted system reset"); }
  if (inputs_.wait_order != McuDspWaitOrder::dsp_before_mcu || !mcu_.cpu().waiting() || offered_ ||
      shared_write_ || state.pc != period.context.pc || state.ccr != period.context.ccr) {
    throw Fault("MCU/DSP idle period requires native WAI and an explicit DSP-before-MCU contract");
  }
  if (expected_idle_cycles_ == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("MCU/DSP idle period counter exhausted");
  }
  waiting_ = true;
  wait_context_ = period.context;
  offer_period(true);
  if (!cop_deadline_) { ++expected_idle_cycles_; }
  advance(DspScheduleBoundary::through, before_dsp, after_dsp, external);
  wait_context_.reset();
}

void McuDspSchedule::end_wait() {
  healthy();
  const Guard guard(*this);
  if (resetting_) { throw Fault("MCU/DSP WAI cannot clock an asserted system reset"); }
  if (inputs_.wait_order != McuDspWaitOrder::dsp_before_mcu || !mcu_.cpu().waiting() || offered_) {
    throw Fault(
        "MCU/DSP idle completion requires native WAI and its explicit observation contract");
  }
  finish_boundary();
  waiting_ = false;
}

void McuDspSchedule::finish_boundary() {
  mcu_.validate_run_boundary();
  consumed();
  if (shared_write_) { throw Fault("MCU/DSP shared write observation was not delivered"); }
  if (end_ticks_ >= inputs_.dsp.epoch_ticks &&
      (dsp_schedule_.horizon_ticks() != end_ticks_ ||
       dsp_schedule_.next_event_ticks(dsp_) <= end_ticks_)) {
    throw Fault("MCU/DSP completed instruction lost its pending DSP deadline");
  }
  offered_ = false;
}

void McuDspSchedule::begin_system_reset(const McuResetSource source,
                                        const unsigned synchronization_cycles,
                                        const McuDspResetOrder order) {
  if (active_ || faulted_ || resetting_ || dsp_.timing_faulted() ||
      mcu_.run_clock().native_epoch() != expected_mcu_epoch_ ||
      order != McuDspResetOrder::dsp_before_mcu) {
    throw Fault(
        "MCU/DSP reset requires an inactive coordinated epoch and explicit DSP-before-MCU order");
  }
  const Guard guard(*this);
  consumed();
  if ((waiting_ && !cop_deadline_) || shared_write_) {
    throw Fault("MCU/DSP reset cannot discard an unconsumed idle/shared-write offer");
  }
  if (cop_deadline_ &&
      (!mcu_.run_clock().faulted() || mcu_.bus().cop_state().reset != McuCopReset::timeout)) {
    throw Fault("MCU/DSP shortened period requires the actual owned COP timeout");
  }
  mcu_.begin_system_reset(source, synchronization_cycles);
  expected_mcu_epoch_ = mcu_.run_clock().native_epoch();
  offered_ = false;
  waiting_ = false;
  cop_deadline_ = false;
  resetting_ = true;
}

void McuDspSchedule::begin_reset_period(const McuResetPeriod period,
                                        const DspObserver &before_dsp,
                                        const DspCompletionObserver &after_dsp,
                                        const DspExternalEvents &external) {
  healthy();
  const Guard guard(*this);
  consumed();
  if (!resetting_ || !mcu_.system_reset_active() || period.native_epoch != expected_mcu_epoch_ ||
      period.duration_ticks != mcu_.run_clock().next_reset_edge_ticks()) {
    throw Fault("MCU/DSP reset edge must match its owned SYSCLK duration and epoch");
  }
  const auto factor = timebase_hz_ / mcu_.run_clock().timebase_hz();
  if (period.duration_ticks > (std::numeric_limits<std::uint64_t>::max() - end_ticks_) / factor) {
    throw Fault("MCU/DSP reset edge exhausted common time");
  }
  end_ticks_ += period.duration_ticks * factor;
  reset_context_ = period.context;
  advance(DspScheduleBoundary::through, before_dsp, after_dsp, external);
  reset_context_.reset();
}

void McuDspSchedule::end_system_reset() {
  healthy();
  const Guard guard(*this);
  if (!resetting_ || mcu_.system_reset_active()) {
    throw Fault("MCU/DSP reset completion requires its completed native vector boundary");
  }
  finish_boundary();
  resetting_ = false;
}

McuDspScheduleCheckpoint McuDspSchedule::checkpoint() const {
  healthy();
  if (resetting_) {
    mcu_.validate_reset_boundary();
  } else {
    mcu_.validate_run_boundary();
  }
  consumed();
  if (offered_ || shared_write_ || waiting_) {
    throw Fault("MCU/DSP schedule capture requires a completed instruction observation");
  }
  McuDspScheduleCheckpoint result(dsp_schedule_.checkpoint(dsp_));
  result.inputs_ = inputs_;
  if (resetting_) {
    result.reset_ = mcu_.system_reset_state();
    result.reset_edge_ticks_ = mcu_.run_clock().next_reset_edge_ticks();
    result.reset_oscillator_hz_ = mcu_.run_clock().oscillator_hz();
  } else {
    result.mcu_ = mcu_.cpu().state();
  }
  result.timebase_hz_ = timebase_hz_;
  result.end_ticks_ = end_ticks_;
  result.mcu_cycles_ = expected_mcu_cycles_;
  result.mcu_epoch_ = expected_mcu_epoch_;
  result.mcu_idle_cycles_ = expected_idle_cycles_;
  result.ties_ = ties_;
  return result;
}

void McuDspSchedule::restore(const McuDspScheduleCheckpoint &checkpoint) {
  if (active_) { throw Fault("Cannot restore an active MCU/DSP schedule"); }
  if (checkpoint.reset_) {
    mcu_.validate_reset_boundary();
  } else {
    mcu_.validate_run_boundary();
  }
  if (checkpoint.inputs_ != inputs_ || checkpoint.timebase_hz_ != timebase_hz_ ||
      mcu_ticks() != checkpoint.end_ticks_ ||
      mcu_.run_clock().bus_cycles() != checkpoint.mcu_cycles_ ||
      mcu_.run_clock().native_epoch() != checkpoint.mcu_epoch_ ||
      mcu_.idle_bus_cycles() != checkpoint.mcu_idle_cycles_ ||
      (checkpoint.reset_
           ? mcu_.system_reset_state() != checkpoint.reset_ ||
                 mcu_.run_clock().next_reset_edge_ticks() != checkpoint.reset_edge_ticks_ ||
                 mcu_.run_clock().oscillator_hz() != checkpoint.reset_oscillator_hz_
           : !same_cpu(mcu_.cpu().state(), checkpoint.mcu_))) {
    throw Fault("MCU/DSP schedule restore requires compatible inputs and coordinated native MCU");
  }
  dsp_schedule_.restore(checkpoint.dsp_, dsp_);
  end_ticks_ = checkpoint.end_ticks_;
  expected_mcu_cycles_ = checkpoint.mcu_cycles_;
  expected_mcu_epoch_ = checkpoint.mcu_epoch_;
  expected_idle_cycles_ = checkpoint.mcu_idle_cycles_;
  ties_ = checkpoint.ties_;
  offered_ = false;
  shared_write_ = false;
  waiting_ = false;
  wait_context_.reset();
  reset_context_.reset();
  resetting_ = checkpoint.reset_.has_value();
  cop_deadline_ = false;
  faulted_ = false;
}
}
