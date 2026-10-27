// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"

#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_cop.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

namespace blofeld {
struct McuDspFetchCheckpoint::Storage {
  McuCheckpoint mcu;
  DspFetchScheduleCheckpoint dsp;
  McuDspFetchInputs inputs;
  const McuMachine *mcu_owner;
  const DspMachine *dsp_owner;
  std::uint64_t end_ticks, cycles, epoch, idle, reset_edges, ties;
  bool resetting;
  std::shared_ptr<const DspFetchEventIdentity> external;
  DspExternalOrder external_order;
};

McuDspFetchCheckpoint::McuDspFetchCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

McuDspFetchCheckpoint::~McuDspFetchCheckpoint() = default;

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
                                         DspMachine &dsp,
                                         DspFetchExternalEvents external)
    : inputs_(inputs),
      mcu_(mcu),
      dsp_(dsp),
      dsp_schedule_({.dsp = inputs.dsp,
                     .epoch_ticks = inputs.epoch_ticks,
                     .initial_horizon_ticks = std::min(inputs.epoch_ticks, mcu_ticks())},
                    dsp),
      external_(std::move(external)),
      initial_mcu_ticks_(mcu_ticks()),
      end_ticks_(mcu_ticks()),
      expected_cycles_(mcu.run_clock().bus_cycles()),
      expected_epoch_(mcu.run_clock().native_epoch()),
      expected_idle_(mcu.idle_bus_cycles()),
      expected_reset_edges_(mcu.run_clock().reset_sysclk_edges()) {
  mcu_.validate_run_boundary();
  if (inputs.dsp.pipeline != DspFetchPipeline::automatic_memory ||
      inputs.maximum_dsp_cycles_per_period == 0 ||
      (!inputs.initial_reset && inputs.epoch_ticks < end_ticks_) ||
      (inputs.initial_reset && inputs.epoch_ticks > end_ticks_) ||
      (inputs.shared_write_order != McuDspTieOrder::unqualified &&
       inputs.shared_write_order != McuDspTieOrder::dsp_before_mcu &&
       inputs.shared_write_order != McuDspTieOrder::mcu_before_dsp) ||
      (inputs.external_write_order != McuExternalWriteOrder::unqualified &&
       inputs.external_write_order != McuExternalWriteOrder::external_before_mcu &&
       inputs.external_write_order != McuExternalWriteOrder::mcu_before_external) ||
      (inputs.wait_order != McuDspWaitOrder::unqualified &&
       inputs.wait_order != McuDspWaitOrder::dsp_before_mcu) ||
      (inputs.reset_order != McuDspResetOrder::unqualified &&
       inputs.reset_order != McuDspResetOrder::dsp_before_mcu)) {
    throw Fault("MCU/DSP fetch schedule requires an automatic pipeline, future epoch, explicit "
                "budget and known tie contract");
  }
  if (inputs.initial_reset) {
    const auto initial = inputs.initial_reset.value();
    dsp_schedule_.begin_hardware_reset({.mode_known = initial.mode_known,
                                        .mode_levels = initial.mode_levels,
                                        .pinit = initial.pinit,
                                        .recovery_cycles = initial.recovery_cycles});
  }
  const Guard guard(*this);
  static_cast<void>(dsp_schedule_.advance_to(
      {.horizon = {.ticks = inputs.initial_reset ? end_ticks_ : initial_mcu_ticks_}},
      {},
      {},
      checked_events()));
  static_cast<void>(deadline());
}

bool McuDspFetchSchedule::faulted() const {
  return faulted_ || dsp_schedule_.faulted() || mcu_.timing_faulted() || dsp_.timing_faulted() ||
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
      mcu_.idle_bus_cycles() != expected_idle_ ||
      mcu_.run_clock().reset_sysclk_edges() != expected_reset_edges_) {
    throw Fault("MCU/DSP fetch schedule requires every offered native MCU period to be consumed");
  }
}

std::uint64_t McuDspFetchSchedule::deadline() const {
  if (run().hardware_reset_active()) {
    const auto remaining = run().hardware_reset_remaining_ticks();
    if (!remaining) { return std::numeric_limits<std::uint64_t>::max(); }
    const auto elapsed = run().elapsed_ticks();
    constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
    if (elapsed > maximum - *remaining || inputs_.epoch_ticks > maximum - elapsed - *remaining) {
      throw Fault("MCU/DSP fetch reset recovery exhausted common time");
    }
    return inputs_.epoch_ticks + elapsed + *remaining;
  }
  const auto period = run().next_cycle_ticks(dsp_);
  const auto elapsed = run().elapsed_ticks();
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
    advance_horizon(
        {.ticks = end_ticks_,
         .native = through ? DspScheduleBoundary::through : DspScheduleBoundary::before,
         .external = through ? DspScheduleBoundary::through : DspScheduleBoundary::before},
        inputs,
        after_dsp);
  } catch (const Fault &error) {
    if (reset_context_) {
      throw Fault("Scheduled pipeline reset peer mcu-request-pc=" + hex(reset_context_->pc) +
                  " dsp-pc=" + hex(dsp_.cpu().state().pc, 6) +
                  " reset-edge=" + std::to_string(expected_reset_edges_) +
                  " mcu-end=" + std::to_string(end_ticks_) + ": " + error.what());
    }
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
  if (period.cop_deadline && inputs_.reset_order != McuDspResetOrder::dsp_before_mcu) {
    const auto context = wait_context_.value_or(cycle_.context);
    throw Fault("Unqualified MCU/DSP fetch COP deadline mcu-pc=" + hex(context.pc));
  }
  const auto factor = inputs_.dsp.timebase_hz / mcu_.run_clock().timebase_hz();
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (expected_cycles_ == maximum || period.duration_ticks > (maximum - end_ticks_) / factor) {
    throw Fault("MCU/DSP fetch next native MCU period exhausted common time");
  }
  end_ticks_ += period.duration_ticks * factor;
  cop_deadline_ = period.cop_deadline;
  if (!cop_deadline_) { ++expected_cycles_; }
  period_cycles_ = 0;
}

void McuDspFetchSchedule::begin_cycle(const Hc12BusCycle cycle,
                                      const bool shared_write,
                                      const CycleInputs &inputs,
                                      const Observer &after_dsp) {
  healthy();
  const Guard guard(*this);
  consumed();
  if (resetting_) {
    const auto reset = mcu_.system_reset_state();
    if (!reset || reset->phase != McuResetPhase::ready || shared_write || cycle.context.pc != 0 ||
        cycle.context.ccr != hc12::reset_ccr ||
        std::string_view("VfP").find(cycle.kind) == std::string_view::npos) {
      throw Fault("MCU/DSP fetch reset permits only its native vector sequence after pad sampling");
    }
  }
  if (waiting_) { throw Fault("MCU/DSP fetch wait slice requires a completed idle boundary"); }
  if (shared_write_) { throw Fault("MCU/DSP fetch shared write observation was not delivered"); }
  const bool write = cycle.kind == 'w' || cycle.kind == 'W';
  if (shared_write && (!write || !cycle.address || cycle.size != 1)) {
    throw Fault("Unqualified MCU/DSP fetch shared write cycle geometry");
  }
  cycle_ = cycle;
  offer_period();
  offered_ = true;
  shared_write_ = shared_write && !cop_deadline_;
  shared_write_observed_ = false;
  advance(!shared_write_, inputs, after_dsp);
  if (shared_write_) { prepare_shared_write(inputs, after_dsp); }
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
  observing_signals_ = true;
  try {
    observe_signals();
  } catch (...) {
    observing_signals_ = false;
    throw;
  }
  observing_signals_ = false;
  shared_write_observed_ = true;
  consumed();
  advance(true, inputs, after_dsp);
  shared_write_ = false;
}

void McuDspFetchSchedule::end_instruction() {
  healthy();
  const Guard guard(*this);
  if (resetting_) { throw Fault("MCU/DSP fetch reset requires explicit native completion"); }
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
  verify_external_boundary();
  offered_ = false;
}

void McuDspFetchSchedule::begin_wait_period(const McuWaitPeriod period,
                                            const CycleInputs &inputs,
                                            const Observer &after_dsp) {
  healthy();
  const Guard guard(*this);
  consumed();
  if (resetting_) { throw Fault("MCU/DSP fetch WAI cannot clock an asserted reset"); }
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
  if (!cop_deadline_) { ++expected_idle_; }
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

void McuDspFetchSchedule::begin_system_reset(const McuResetSource source,
                                             const unsigned synchronization_cycles) {
  if (active_ || faulted_ || resetting_ || dsp_schedule_.faulted() || dsp_.timing_faulted() ||
      mcu_.run_clock().native_epoch() != expected_epoch_ ||
      inputs_.reset_order != McuDspResetOrder::dsp_before_mcu) {
    throw Fault("MCU/DSP fetch reset requires an inactive coordinated epoch and explicit order");
  }
  const Guard guard(*this);
  consumed();
  static_cast<void>(deadline());
  if ((waiting_ && !cop_deadline_) || shared_write_) {
    throw Fault("MCU/DSP fetch reset cannot discard an unconsumed idle/shared-write offer");
  }
  if (cop_deadline_ &&
      (!mcu_.run_clock().faulted() || mcu_.bus().cop_state().reset != McuCopReset::timeout)) {
    throw Fault("MCU/DSP fetch shortened period requires the actual owned COP timeout");
  }
  mcu_.begin_system_reset(source, synchronization_cycles);
  expected_epoch_ = mcu_.run_clock().native_epoch();
  expected_reset_edges_ = mcu_.run_clock().reset_sysclk_edges();
  offered_ = false;
  waiting_ = false;
  cop_deadline_ = false;
  resetting_ = true;
}

void McuDspFetchSchedule::begin_reset_period(const McuResetPeriod period,
                                             const CycleInputs &inputs,
                                             const Observer &after_dsp) {
  healthy();
  const Guard guard(*this);
  consumed();
  if (!resetting_ || !mcu_.system_reset_active() || period.native_epoch != expected_epoch_ ||
      period.duration_ticks != mcu_.run_clock().next_reset_edge_ticks()) {
    throw Fault("MCU/DSP fetch reset edge must match its owned SYSCLK duration and epoch");
  }
  const auto factor = inputs_.dsp.timebase_hz / mcu_.run_clock().timebase_hz();
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (expected_reset_edges_ == maximum || period.duration_ticks > (maximum - end_ticks_) / factor) {
    throw Fault("MCU/DSP fetch reset edge exhausted common time");
  }
  end_ticks_ += period.duration_ticks * factor;
  ++expected_reset_edges_;
  period_cycles_ = 0;
  reset_context_ = period.context;
  advance(true, inputs, after_dsp);
  reset_context_.reset();
}

void McuDspFetchSchedule::end_system_reset() {
  healthy();
  const Guard guard(*this);
  if (!resetting_ || mcu_.system_reset_active()) {
    throw Fault("MCU/DSP fetch reset completion requires its completed native vector boundary");
  }
  finish_boundary();
  resetting_ = false;
}

McuDspFetchCheckpoint McuDspFetchSchedule::checkpoint() const {
  healthy();
  if (resetting_) {
    mcu_.validate_reset_boundary();
  } else {
    mcu_.validate_run_boundary();
  }
  consumed();
  if (offered_ || shared_write_ || waiting_ || cop_deadline_ || wait_context_ || reset_context_) {
    throw Fault("MCU/DSP fetch capture requires completed native observations");
  }
  if (deadline() <= end_ticks_) { throw Fault("MCU/DSP fetch capture lost its peer deadline"); }
  verify_external_boundary();
  return McuDspFetchCheckpoint(std::make_shared<const McuDspFetchCheckpoint::Storage>(
      McuDspFetchCheckpoint::Storage{.mcu = mcu_.checkpoint(),
                                     .dsp = dsp_schedule_.checkpoint(),
                                     .inputs = inputs_,
                                     .mcu_owner = &mcu_,
                                     .dsp_owner = &dsp_,
                                     .end_ticks = end_ticks_,
                                     .cycles = expected_cycles_,
                                     .epoch = expected_epoch_,
                                     .idle = expected_idle_,
                                     .reset_edges = expected_reset_edges_,
                                     .ties = ties_,
                                     .resetting = resetting_,
                                     .external = external_.identity,
                                     .external_order = external_.order}));
}

void McuDspFetchSchedule::restore(const McuDspFetchCheckpoint &checkpoint) {
  if (active_ || !checkpoint.storage_ || checkpoint.storage_->mcu_owner != &mcu_ ||
      checkpoint.storage_->dsp_owner != &dsp_ || checkpoint.storage_->inputs != inputs_ ||
      checkpoint.storage_->external != external_.identity ||
      checkpoint.storage_->external_order != external_.order) {
    throw Fault("MCU/DSP fetch restore requires matching inactive owners and input contracts");
  }
  const Guard guard(*this);
  faulted_ = true; // A partial native restore can never become a healthy schedule.
  const auto &saved = *checkpoint.storage_;
  mcu_.restore(saved.mcu);
  dsp_schedule_.restore(saved.dsp);
  end_ticks_ = saved.end_ticks;
  expected_cycles_ = saved.cycles;
  expected_epoch_ = saved.epoch;
  expected_idle_ = saved.idle;
  expected_reset_edges_ = saved.reset_edges;
  ties_ = saved.ties;
  resetting_ = saved.resetting;
  offered_ = false;
  shared_write_ = false;
  waiting_ = false;
  cop_deadline_ = false;
  wait_context_.reset();
  reset_context_.reset();
  period_cycles_ = 0;
  cycle_ = {};
  observing_signals_ = false;
  shared_write_observed_ = false;
  if (mcu_.system_reset_active() != resetting_) {
    throw Fault("MCU/DSP fetch restore lost its native reset boundary");
  }
  if (resetting_) {
    mcu_.validate_reset_boundary();
  } else {
    mcu_.validate_run_boundary();
  }
  consumed();
  if (deadline() <= end_ticks_) { throw Fault("MCU/DSP fetch restore lost its peer deadline"); }
  verify_external_boundary();
  faulted_ = false;
}

void McuDspFetchSchedule::validate_dsp_reset_observation() const {
  if (faulted() || (active_ && !observing_signals_)) {
    throw Fault("DSP fetch reset observation requires a healthy native signal boundary");
  }
  if (!active_) {
    if (mcu_.system_reset_active()) {
      mcu_.validate_reset_boundary();
    } else {
      mcu_.validate_run_boundary();
    }
  }
  consumed();
  if (((offered_ || shared_write_ || waiting_ || wait_context_ || reset_context_) &&
       !observing_signals_) ||
      cop_deadline_ || inputs_.epoch_ticks > end_ticks_ ||
      run().elapsed_ticks() != end_ticks_ - inputs_.epoch_ticks) {
    throw Fault("DSP fetch reset cannot discard unconsumed native peer time");
  }
}

void McuDspFetchSchedule::begin_dsp_reset(const DspHardwareResetInputs inputs) {
  validate_dsp_reset_observation();
  dsp_schedule_.begin_hardware_reset(inputs);
}

void McuDspFetchSchedule::release_dsp_reset() {
  validate_dsp_reset_observation();
  dsp_schedule_.release_hardware_reset();
}

}
