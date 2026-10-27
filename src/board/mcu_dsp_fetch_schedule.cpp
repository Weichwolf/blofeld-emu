// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"

#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <string>

namespace blofeld {
class McuDspFetchSchedule::Guard {
public:
  explicit Guard(McuDspFetchSchedule &schedule)
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
  McuDspFetchSchedule &schedule_;
  int exceptions_;
};

McuDspFetchSchedule::McuDspFetchSchedule(const McuDspFetchInputs inputs,
                                         McuMachine &mcu,
                                         DspMachine &dsp)
    : inputs_(inputs),
      mcu_(mcu),
      dsp_(dsp),
      run_(inputs.dsp, dsp),
      end_ticks_(mcu_ticks()),
      expected_cycles_(mcu.run_clock().bus_cycles()),
      expected_epoch_(mcu.run_clock().native_epoch()),
      expected_idle_(mcu.idle_bus_cycles()) {
  mcu_.validate_run_boundary();
  if (inputs.dsp.pipeline != DspFetchPipeline::automatic_memory ||
      inputs.maximum_dsp_cycles_per_period == 0 || inputs.epoch_ticks < end_ticks_ ||
      (inputs.shared_write_order != McuDspTieOrder::unqualified &&
       inputs.shared_write_order != McuDspTieOrder::dsp_before_mcu &&
       inputs.shared_write_order != McuDspTieOrder::mcu_before_dsp) ||
      (inputs.wait_order != McuDspWaitOrder::unqualified &&
       inputs.wait_order != McuDspWaitOrder::dsp_before_mcu)) {
    throw Fault("MCU/DSP fetch schedule requires an automatic pipeline, future epoch, explicit "
                "budget and known tie contract");
  }
  static_cast<void>(deadline());
}

bool McuDspFetchSchedule::faulted() const {
  return faulted_ || run_.faulted() || mcu_.timing_faulted() || dsp_.timing_faulted() ||
         mcu_.run_clock().native_epoch() != expected_epoch_;
}

void McuDspFetchSchedule::healthy() const {
  if (active_ || faulted()) {
    throw Fault("MCU/DSP fetch schedule requires a healthy inactive boundary");
  }
}

std::uint64_t McuDspFetchSchedule::mcu_ticks() const {
  const auto &clock = mcu_.run_clock();
  const auto timebase = inputs_.dsp.timebase_hz;
  if (timebase < clock.timebase_hz() || timebase % clock.timebase_hz() != 0) {
    throw Fault("MCU/DSP fetch timebase conversion must be an exact positive integer");
  }
  const auto factor = timebase / clock.timebase_hz();
  if (clock.elapsed_ticks() > std::numeric_limits<std::uint64_t>::max() / factor) {
    throw Fault("MCU/DSP fetch common time conversion exhausted");
  }
  return clock.elapsed_ticks() * factor;
}

void McuDspFetchSchedule::consumed() const {
  if (mcu_ticks() != end_ticks_ || mcu_.run_clock().bus_cycles() != expected_cycles_ ||
      mcu_.idle_bus_cycles() != expected_idle_) {
    throw Fault("MCU/DSP fetch schedule requires every offered native MCU period to be consumed");
  }
}

std::uint64_t McuDspFetchSchedule::deadline() const {
  const auto period = run_.next_cycle_ticks(dsp_);
  const auto elapsed = run_.elapsed_ticks();
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (elapsed > maximum - period || inputs_.epoch_ticks > maximum - elapsed - period) {
    throw Fault("MCU/DSP fetch absolute next-cycle deadline exhausted");
  }
  return inputs_.epoch_ticks + elapsed + period;
}

void McuDspFetchSchedule::advance(const bool through,
                                  const CycleInputs &inputs,
                                  const Observer &after_dsp) {
  try {
    while (true) {
      const auto next = deadline();
      if (next > end_ticks_ || (next == end_ticks_ && !through)) { return; }
      if (period_cycles_ == inputs_.maximum_dsp_cycles_per_period) {
        throw Fault("MCU/DSP fetch per-period cycle budget exhausted");
      }
      if (!inputs) { throw Fault("MCU/DSP fetch requires explicit native cycle inputs"); }
      const auto result = run_.advance_cycle(dsp_, inputs());
      ++period_cycles_;
      if (after_dsp) { after_dsp(result); }
    }
  } catch (const Fault &error) {
    if (wait_context_) {
      throw Fault("Scheduled pipeline idle peer mcu-pc=" + hex(wait_context_->pc) +
                  " dsp-pc=" + hex(dsp_.cpu().state().pc, 6) +
                  " mcu-idle-period=" + std::to_string(expected_idle_) +
                  " mcu-end=" + std::to_string(end_ticks_) + ": " + error.what());
    }
    throw Fault("Scheduled pipeline peer mcu-pc=" + hex(cycle_.context.pc) + " dsp-pc=" +
                hex(dsp_.cpu().state().pc, 6) + " mcu-kind=" + std::string(1, cycle_.kind) +
                " mcu-address=" + (cycle_.address ? hex(*cycle_.address) : "none") +
                " mcu-end=" + std::to_string(end_ticks_) + ": " + error.what());
  }
}

void McuDspFetchSchedule::offer_period(const bool waiting) {
  const auto period = mcu_.run_clock().next_period(mcu_.bus(), waiting);
  if (period.cop_deadline) {
    const auto context = wait_context_.value_or(cycle_.context);
    throw Fault("Unqualified MCU/DSP fetch COP deadline mcu-pc=" + hex(context.pc));
  }
  const auto factor = inputs_.dsp.timebase_hz / mcu_.run_clock().timebase_hz();
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (expected_cycles_ == maximum || period.duration_ticks > (maximum - end_ticks_) / factor) {
    throw Fault("MCU/DSP fetch next native MCU period exhausted common time");
  }
  end_ticks_ += period.duration_ticks * factor;
  ++expected_cycles_;
  period_cycles_ = 0;
}

void McuDspFetchSchedule::begin_cycle(const Hc12BusCycle cycle,
                                      const bool shared_write,
                                      const CycleInputs &inputs,
                                      const Observer &after_dsp) {
  healthy();
  const Guard guard(*this);
  consumed();
  if (waiting_) { throw Fault("MCU/DSP fetch wait slice requires a completed idle boundary"); }
  if (shared_write_) { throw Fault("MCU/DSP fetch shared write observation was not delivered"); }
  const bool write = cycle.kind == 'w' || cycle.kind == 'W';
  if (shared_write && (!write || !cycle.address || cycle.size != 1)) {
    throw Fault("Unqualified MCU/DSP fetch shared write cycle geometry");
  }
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  cycle_ = cycle;
  offer_period();
  offered_ = true;
  shared_write_ = shared_write;
  advance(!shared_write, inputs, after_dsp);
  if (!shared_write || deadline() != end_ticks_) { return; }
  if (ties_ == maximum) { throw Fault("MCU/DSP fetch shared write tie counter exhausted"); }
  ++ties_;
  if (inputs_.shared_write_order == McuDspTieOrder::unqualified) {
    throw Fault("Unqualified simultaneous DSP pipeline cycle and MCU shared write at " +
                hex(*cycle.address) + " mcu-pc=" + hex(cycle.context.pc) +
                " dsp-pc=" + hex(dsp_.cpu().state().pc, 6));
  }
  if (inputs_.shared_write_order == McuDspTieOrder::dsp_before_mcu) {
    advance(true, inputs, after_dsp);
  }
}

void McuDspFetchSchedule::observe_write(const BusEvent &event,
                                        const std::function<void()> &observe_signals,
                                        const CycleInputs &inputs,
                                        const Observer &after_dsp) {
  healthy();
  const Guard guard(*this);
  consumed();
  if (!offered_ || !shared_write_ || !event.write || !cycle_.address ||
      event.address != *cycle_.address || !observe_signals) {
    throw Fault("MCU/DSP fetch shared observation must match its consumed native write cycle");
  }
  observe_signals();
  consumed();
  advance(true, inputs, after_dsp);
  shared_write_ = false;
}

void McuDspFetchSchedule::end_instruction() {
  healthy();
  const Guard guard(*this);
  if (waiting_) { throw Fault("MCU/DSP fetch wait slice requires an explicit idle completion"); }
  if (mcu_.cpu().waiting() && inputs_.wait_order != McuDspWaitOrder::dsp_before_mcu) {
    throw Fault("Unqualified MCU/DSP fetch WAI boundary");
  }
  finish_boundary();
}

void McuDspFetchSchedule::finish_boundary() {
  mcu_.validate_run_boundary();
  consumed();
  if (shared_write_) { throw Fault("MCU/DSP fetch shared write observation was not delivered"); }
  if (deadline() <= end_ticks_) {
    throw Fault("MCU/DSP fetch completed boundary lost its pending DSP deadline");
  }
  offered_ = false;
}

void McuDspFetchSchedule::begin_wait_period(const McuWaitPeriod period,
                                            const CycleInputs &inputs,
                                            const Observer &after_dsp) {
  healthy();
  const Guard guard(*this);
  consumed();
  const auto state = mcu_.cpu().state();
  if (inputs_.wait_order != McuDspWaitOrder::dsp_before_mcu || !mcu_.cpu().waiting() || offered_ ||
      shared_write_ || state.pc != period.context.pc || state.ccr != period.context.ccr) {
    throw Fault(
        "MCU/DSP fetch idle period requires native WAI and an explicit DSP-before-MCU contract");
  }
  if (expected_idle_ == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("MCU/DSP fetch idle period counter exhausted");
  }
  waiting_ = true;
  wait_context_ = period.context;
  offer_period(true);
  ++expected_idle_;
  advance(true, inputs, after_dsp);
  wait_context_.reset();
}

void McuDspFetchSchedule::end_wait() {
  healthy();
  const Guard guard(*this);
  if (inputs_.wait_order != McuDspWaitOrder::dsp_before_mcu || !mcu_.cpu().waiting() || offered_) {
    throw Fault(
        "MCU/DSP fetch idle completion requires native WAI and its explicit observation contract");
  }
  finish_boundary();
  waiting_ = false;
}
}
