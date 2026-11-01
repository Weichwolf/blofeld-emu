// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"

#include "board/detail/mcu_dsp_boundary.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_cop.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace blofeld {
namespace {
std::string_view operation(const dsp56300::FaultKind kind) {
  switch (kind) {
    case dsp56300::FaultKind::io_read:
    case dsp56300::FaultKind::memory_read: return "read";
    case dsp56300::FaultKind::io_write:
    case dsp56300::FaultKind::memory_write: return "write";
    case dsp56300::FaultKind::io_bit: return "bit";
    case dsp56300::FaultKind::unsupported_instruction: return "unsupported instruction";
    case dsp56300::FaultKind::unsupported_mode: return "unsupported mode";
    case dsp56300::FaultKind::stack: return "stack";
    case dsp56300::FaultKind::invalid_instruction: return "invalid instruction";
    case dsp56300::FaultKind::host_event: return "host event";
    case dsp56300::FaultKind::none: return "missing fault";
    default: return "invalid fault";
  }
}

std::string_view space_name(const dsp56300::Space space) {
  switch (space) {
    case dsp56300::Space::program: return "P";
    case dsp56300::Space::x: return "X";
    case dsp56300::Space::y: return "Y";
    default: return "unknown-space";
  }
}

class SignalObservation {
public:
  explicit SignalObservation(bool &active) : active_(active) { active_ = true; }

  ~SignalObservation() { active_ = false; }

  SignalObservation(const SignalObservation &) = delete;
  SignalObservation &operator=(const SignalObservation &) = delete;

private:
  bool &active_;
};
}

void McuDspRuntime::reject_native(const dsp56300::Fault &fault) const {
  std::string message = "MCU/DSP native " + std::string(operation(fault.kind)) + " " +
                        std::string(space_name(fault.space)) + ":" + hex(fault.address, 6) +
                        " pc=" + hex(fault.pc, 6) + " value=" + hex(fault.value, 6) +
                        " mcu-pc=" + hex(mcu_.cpu().state().pc) + " tick=" +
                        std::to_string(inputs_.epoch_ticks + dsp_.clock().elapsed_ticks());
  const auto &failure = dsp_.io().failure();
  if (failure && failure->cause) {
    try {
      std::rethrow_exception(failure->cause);
    } catch (const std::exception &error) { message += ": " + std::string(error.what()); }
  }
  throw Fault(message);
}

class McuDspRuntime::Guard {
public:
  explicit Guard(McuDspRuntime &runtime)
      : runtime_(runtime),
        exceptions_(std::uncaught_exceptions()),
        previously_active_(runtime.active_) {
    runtime_.active_ = true;
  }

  ~Guard() {
    runtime_.active_ = previously_active_;
    if (std::uncaught_exceptions() > exceptions_) { runtime_.faulted_ = true; }
  }

  Guard(const Guard &) = delete;
  Guard &operator=(const Guard &) = delete;

private:
  McuDspRuntime &runtime_;
  int exceptions_;
  bool previously_active_;
};

struct McuDspRuntimeCheckpoint::Storage {
  McuCheckpoint mcu;
  OwnDspTimelineCheckpoint dsp;
  McuDspRuntimeInputs inputs;
  McuDspRuntime::State state;
  std::shared_ptr<const bool> external_identity;
  DspPeerOrder external_order;
  unsigned external_budget;
};

McuDspRuntimeCheckpoint::McuDspRuntimeCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

McuDspRuntimeCheckpoint::~McuDspRuntimeCheckpoint() = default;

McuDspRuntime::McuDspRuntime(const McuDspRuntimeInputs inputs,
                             McuMachine &mcu,
                             OwnDspMachine &dsp,
                             DspRuntimeEvents external)
    : inputs_(inputs), mcu_(mcu), dsp_(dsp), external_(std::move(external)) {
  const bool has_external = external_.identity != nullptr;
  const bool ordered = external_.order == DspPeerOrder::dsp_first ||
                       external_.order == DspPeerOrder::peer_first ||
                       external_.order == DspPeerOrder::reject_tie;
  if (!inputs_.initial_reset || inputs_.epoch_ticks != 0 ||
      inputs_.maximum_cycles_per_period == 0 ||
      has_external != static_cast<bool>(external_.next_ticks) ||
      has_external != static_cast<bool>(external_.observe) ||
      (has_external && (!ordered || external_.maximum_events_per_period == 0)) ||
      (!has_external && (external_.order != DspPeerOrder::unqualified ||
                         external_.maximum_events_per_period != 0))) {
    throw Fault(
        "MCU/DSP runtime requires held reset, explicit budgets and coherent event ownership");
  }
  state_.end_ticks = mcu_ticks();
  state_.cycles = mcu_.run_clock().bus_cycles();
  state_.epoch = mcu_.run_clock().native_epoch();
  state_.idle = mcu_.idle_bus_cycles();
  state_.reset_edges = mcu_.run_clock().reset_sysclk_edges();
  state_.resetting = mcu_.system_reset_active();
  if (state_.resetting) {
    if (!inputs_.reset_dsp_first) { throw Fault("Unqualified MCU/DSP initial reset ordering"); }
    mcu_.validate_reset_boundary();
  } else {
    mcu_.validate_run_boundary();
  }
  timeline_ = std::make_unique<OwnDspTimeline>(
      dsp_, inputs_.timing, inputs_.epoch_ticks, inputs_.initial_reset);
  const Guard guard(*this);
  const auto first = next_external();
  if (first && *first < state_.end_ticks) {
    throw Fault("MCU/DSP external event predates its initial native MCU boundary tick=" +
                std::to_string(*first) + " initial-tick=" + std::to_string(state_.end_ticks) +
                " mcu-pc=" + hex(mcu_.cpu().state().pc) +
                " dsp-pc=" + hex(dsp_.cpu().registers().pc, 6));
  }
  advance(true, true, {}, {});
}

std::uint64_t McuDspRuntime::mcu_ticks() const {
  const auto common = dsp_.clock().timebase_hz();
  const auto local = mcu_.run_clock().timebase_hz();
  if (local == 0 || common < local || common % local != 0) {
    throw Fault("MCU/DSP runtime requires an exact positive common timebase");
  }
  const auto factor = common / local;
  if (mcu_.run_clock().elapsed_ticks() > std::numeric_limits<std::uint64_t>::max() / factor) {
    throw Fault("MCU/DSP runtime exhausted common time");
  }
  return mcu_.run_clock().elapsed_ticks() * factor;
}

bool McuDspRuntime::faulted() const {
  return faulted_ || timeline_->faulted() || dsp_.clock().faulted() || mcu_.timing_faulted() ||
         mcu_.run_clock().native_epoch() != state_.epoch;
}

void McuDspRuntime::healthy() const {
  if (active_ || faulted()) {
    throw Fault("MCU/DSP runtime requires its healthy inactive native epoch");
  }
  timeline_->validate_boundary();
}

void McuDspRuntime::consumed() const {
  if (mcu_ticks() != state_.end_ticks || mcu_.run_clock().bus_cycles() != state_.cycles ||
      mcu_.idle_bus_cycles() != state_.idle ||
      mcu_.run_clock().reset_sysclk_edges() != state_.reset_edges) {
    throw Fault("MCU/DSP runtime lost a consumed native period");
  }
}

std::uint64_t McuDspRuntime::completed_cycles() const {
  return dsp_.clock().native_cycle_units();
}

std::uint64_t McuDspRuntime::deadline() const {
  if (!dsp_.clock().hardware_reset_active()) { return timeline_->next_deadline(); }
  const auto remaining = dsp_.clock().hardware_reset_remaining_ticks();
  if (!remaining) { return std::numeric_limits<std::uint64_t>::max(); }
  const auto elapsed = dsp_.clock().elapsed_ticks();
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (*remaining > maximum - elapsed || elapsed + *remaining > maximum - inputs_.epoch_ticks) {
    throw Fault("MCU/DSP reset deadline exhausted common time");
  }
  return inputs_.epoch_ticks + elapsed + *remaining;
}

std::optional<std::uint64_t> McuDspRuntime::query_external() const {
  if (!external_.identity) { return std::nullopt; }
  const auto before = detail::mcu_boundary(mcu_);
  const auto registers = dsp_.cpu().registers();
  const auto elapsed = dsp_.clock().elapsed_ticks();
  const auto result = external_.next_ticks();
  if (detail::mcu_boundary(mcu_) != before || dsp_.cpu().registers() != registers ||
      dsp_.clock().elapsed_ticks() != elapsed) {
    throw Fault("MCU/DSP event query changed CPU state or time");
  }
  timeline_->validate_boundary();
  return result;
}

std::optional<std::uint64_t> McuDspRuntime::next_external() {
  const auto result = query_external();
  if (state_.pending_external && result != state_.pending_external) {
    throw Fault("MCU/DSP source changed its pinned pending deadline");
  }
  if (result && !state_.pending_external) {
    if ((state_.last_external && *result < *state_.last_external) ||
        (state_.closed_external && *result <= *state_.closed_external) ||
        (state_.last_native &&
         (*result < *state_.last_native ||
          (*result == *state_.last_native && external_.order != DspPeerOrder::dsp_first)))) {
      throw Fault("MCU/DSP external source moved backwards into a consumed timeline");
    }
  }
  state_.pending_external = result;
  return result;
}

void McuDspRuntime::observe_external(const std::uint64_t ticks) {
  if (++period_events_ > external_.maximum_events_per_period) {
    throw Fault("MCU/DSP external source exceeded its period budget");
  }
  const auto before = detail::mcu_boundary(mcu_);
  const auto registers = dsp_.cpu().registers();
  const auto elapsed = dsp_.clock().elapsed_ticks();
  external_.observe(ticks);
  if (detail::mcu_boundary(mcu_) != before || dsp_.cpu().registers() != registers ||
      dsp_.clock().elapsed_ticks() != elapsed) {
    throw Fault("MCU/DSP external observation changed CPU state or time");
  }
  timeline_->validate_boundary();
  state_.last_external = ticks;
  state_.pending_external.reset();
}

void McuDspRuntime::advance_native(const std::uint64_t ticks,
                                   const CycleInputs &inputs,
                                   const Observer &observer) {
  if (dsp_.clock().hardware_reset_active()) {
    static_cast<void>(timeline_->advance_reset_to(ticks));
    state_.last_native = ticks;
    return;
  }
  if (!inputs || ++period_cycles_ > inputs_.maximum_cycles_per_period) {
    throw Fault("MCU/DSP runtime lacks native cycle inputs or exceeded its period budget");
  }
  const auto before = detail::mcu_boundary(mcu_);
  const auto registers = dsp_.cpu().registers();
  auto request = inputs();
  timeline_->validate_boundary();
  if (detail::mcu_boundary(mcu_) != before || dsp_.cpu().registers() != registers) {
    throw Fault("MCU/DSP cycle input provider changed CPU state or time");
  }
  const auto last = dsp_.bus().last_dma_cycle();
  request.cycle = last ? *last + 1 : 0;
  const auto start = inputs_.epoch_ticks + dsp_.clock().elapsed_ticks();
  const auto result = timeline_->advance_one(ticks, OwnDspBoundary::through, request);
  if (!result) {
    throw Fault("MCU/DSP native cycle missed its deadline pc=" + hex(dsp_.cpu().registers().pc, 6));
  }
  if (result->native && result->native->fault.kind != dsp56300::FaultKind::none) {
    reject_native(result->native->fault);
  }
  state_.last_native = ticks;
  if (observer) {
    const auto after = dsp_.cpu().registers();
    observer({.start_ticks = start,
              .completion_ticks = ticks,
              .cycle = request.cycle,
              .result = *result});
    timeline_->validate_boundary();
    if (detail::mcu_boundary(mcu_) != before || dsp_.cpu().registers() != after) {
      throw Fault("MCU/DSP cycle observer changed CPU state or time");
    }
  }
}

void McuDspRuntime::advance_reset(const std::uint64_t ticks, const bool through) {
  if (!dsp_.clock().hardware_reset_active()) { return; }
  const auto until = !through && deadline() == ticks ? ticks - 1 : ticks;
  static_cast<void>(timeline_->advance_reset_to(until));
}

void McuDspRuntime::advance(const bool native_through,
                            const bool external_through,
                            const CycleInputs &inputs,
                            const Observer &observer) {
  while (true) {
    const auto event = next_external();
    const auto native = deadline();
    const bool external_due =
        event && (*event < state_.end_ticks || (*event == state_.end_ticks && external_through));
    const bool native_due =
        native < state_.end_ticks || (native == state_.end_ticks && native_through);
    if (!external_due && !native_due) { break; }
    if (external_due && native_due && *event == native &&
        external_.order == DspPeerOrder::reject_tie) {
      throw Fault("Unqualified simultaneous MCU/DSP external and native event");
    }
    if (external_due && (!native_due || *event < native ||
                         (*event == native && external_.order == DspPeerOrder::peer_first))) {
      advance_reset(*event, false);
      observe_external(*event);
    } else {
      advance_native(native, inputs, observer);
    }
  }
  advance_reset(state_.end_ticks, native_through);
  if (external_through) { state_.closed_external = state_.end_ticks; }
}

void McuDspRuntime::offer_period(const bool waiting) {
  const auto period = mcu_.run_clock().next_period(mcu_.bus(), waiting);
  if (period.cop_deadline && !inputs_.reset_dsp_first) {
    throw Fault("Unqualified MCU/DSP COP reset deadline pc=" + hex(cycle_.context.pc));
  }
  const auto factor = dsp_.clock().timebase_hz() / mcu_.run_clock().timebase_hz();
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (state_.cycles == maximum || period.duration_ticks > (maximum - state_.end_ticks) / factor) {
    throw Fault("MCU/DSP runtime period exhausted common time");
  }
  state_.end_ticks += period.duration_ticks * factor;
  state_.cop_deadline = period.cop_deadline;
  if (!state_.cop_deadline) { ++state_.cycles; }
  period_cycles_ = 0;
  period_events_ = 0;
}

void McuDspRuntime::prepare_write(const CycleInputs &inputs, const Observer &observer) {
  const auto reject = [this](const std::string_view reason) {
    throw Fault(
        std::string(reason) + " at " + (cycle_.address ? hex(*cycle_.address) : "<unaddressed>") +
        " mcu-pc=" + hex(cycle_.context.pc) + " dsp-pc=" + hex(dsp_.cpu().registers().pc, 6) +
        " tick=" + std::to_string(state_.end_ticks));
  };
  const bool native = deadline() == state_.end_ticks;
  const auto input = next_external();
  const bool external = input && *input == state_.end_ticks;
  if (native) {
    ++state_.ties;
    if (inputs_.writes != McuWriteOrder::before_mcu && inputs_.writes != McuWriteOrder::after_mcu) {
      reject("Unqualified simultaneous DSP cycle and MCU shared write");
    }
  }
  if (external && inputs_.external_writes != McuWriteOrder::before_mcu &&
      inputs_.external_writes != McuWriteOrder::after_mcu) {
    reject("Unqualified simultaneous external event and MCU shared write");
  }
  const bool native_first = inputs_.writes == McuWriteOrder::before_mcu;
  const bool external_first = inputs_.external_writes == McuWriteOrder::before_mcu;
  if (native && external &&
      (external_.order == DspPeerOrder::reject_tie ||
       (native_first != external_first &&
        (external_.order == DspPeerOrder::peer_first) != external_first))) {
    reject("Cyclic MCU/DSP/external shared-write ordering");
  }
  advance(native_first, external_first, inputs, observer);
  const auto pending = next_external();
  if (pending && *pending == state_.end_ticks &&
      inputs_.external_writes != McuWriteOrder::before_mcu &&
      inputs_.external_writes != McuWriteOrder::after_mcu) {
    reject("Unqualified newborn external event at MCU shared write");
  }
}

void McuDspRuntime::begin_cycle(const Hc12BusCycle cycle,
                                const bool shared_write,
                                const CycleInputs &inputs,
                                const Observer &observer) {
  healthy();
  const Guard guard(*this);
  consumed();
  if (state_.waiting || state_.shared_write ||
      (shared_write &&
       ((cycle.kind != 'w' && cycle.kind != 'W') || !cycle.address || cycle.size != 1))) {
    throw Fault("MCU/DSP runtime requires a consumed byte-sized shared write or native cycle");
  }
  cycle_ = cycle;
  offer_period(false);
  state_.offered = true;
  state_.shared_write = shared_write && !state_.cop_deadline;
  advance(!state_.shared_write, !state_.shared_write, inputs, observer);
  if (state_.shared_write) { prepare_write(inputs, observer); }
}

void McuDspRuntime::observe_write(const BusEvent &event,
                                  const std::function<void()> &observe,
                                  const CycleInputs &inputs,
                                  const Observer &observer) {
  healthy();
  const Guard guard(*this);
  consumed();
  if (!state_.offered || !state_.shared_write || !event.write || !cycle_.address ||
      *cycle_.address != event.address || !observe) {
    throw Fault("MCU/DSP observation must match the consumed shared-write period");
  }
  const auto before = detail::mcu_boundary(mcu_);
  {
    const SignalObservation signals(observing_signals_);
    observe();
  }
  if (faulted() || detail::mcu_boundary(mcu_) != before) {
    throw Fault("MCU/DSP signal observation changed MCU ownership or faulted its runtime");
  }
  consumed();
  const auto next = next_external();
  if (next && *next == state_.end_ticks && inputs_.external_writes != McuWriteOrder::after_mcu) {
    throw Fault("Newborn external event cannot precede its MCU write cause");
  }
  advance(true, true, inputs, observer);
  state_.shared_write = false;
}

void McuDspRuntime::finish_boundary() {
  mcu_.validate_run_boundary();
  consumed();
  if (state_.shared_write || state_.cop_deadline || deadline() <= state_.end_ticks) {
    throw Fault("MCU/DSP completion lost a shared write, reset or native deadline");
  }
  const auto event = next_external();
  if (event && *event <= state_.end_ticks) {
    throw Fault("MCU/DSP completion lost an external event");
  }
  state_.offered = false;
}

void McuDspRuntime::end_instruction() {
  healthy();
  const Guard guard(*this);
  if (state_.resetting || state_.waiting || (mcu_.cpu().waiting() && !inputs_.wait_dsp_first)) {
    throw Fault("MCU/DSP native completion requires its correct reset/wait boundary");
  }
  finish_boundary();
}

void McuDspRuntime::begin_wait_period(const McuWaitPeriod period,
                                      const CycleInputs &inputs,
                                      const Observer &observer) {
  healthy();
  const Guard guard(*this);
  consumed();
  const auto cpu = mcu_.cpu().state();
  if (!inputs_.wait_dsp_first || state_.resetting || state_.offered || state_.shared_write ||
      !mcu_.cpu().waiting() || cpu.pc != period.context.pc || cpu.ccr != period.context.ccr) {
    throw Fault("MCU/DSP idle offer requires native WAI and explicit ordering");
  }
  state_.waiting = true;
  offer_period(true);
  if (!state_.cop_deadline) { ++state_.idle; }
  advance(true, true, inputs, observer);
}

void McuDspRuntime::end_wait() {
  healthy();
  const Guard guard(*this);
  // A pending or pre-observed IRQ can cancel advance_wait before any native
  // period is offered. Its empty boundary still closes, without owning time.
  if (!inputs_.wait_dsp_first || state_.resetting || !mcu_.cpu().waiting() ||
      (!state_.waiting && state_.offered)) {
    throw Fault("MCU/DSP idle boundary is missing");
  }
  finish_boundary();
  state_.waiting = false;
}

void McuDspRuntime::begin_reset_period(const McuResetPeriod period,
                                       const CycleInputs &inputs,
                                       const Observer &observer) {
  healthy();
  const Guard guard(*this);
  consumed();
  if (!state_.resetting || !mcu_.system_reset_active() || period.native_epoch != state_.epoch ||
      period.duration_ticks != mcu_.run_clock().next_reset_edge_ticks()) {
    throw Fault("MCU/DSP reset offer must match its real SYSCLK edge");
  }
  const auto factor = dsp_.clock().timebase_hz() / mcu_.run_clock().timebase_hz();
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (state_.reset_edges == maximum ||
      period.duration_ticks > (maximum - state_.end_ticks) / factor) {
    throw Fault("MCU/DSP reset edge exhausted common time");
  }
  state_.end_ticks += period.duration_ticks * factor;
  ++state_.reset_edges;
  period_cycles_ = 0;
  period_events_ = 0;
  advance(true, true, inputs, observer);
}

void McuDspRuntime::end_system_reset() {
  healthy();
  const Guard guard(*this);
  if (!state_.resetting || mcu_.system_reset_active()) {
    throw Fault("MCU/DSP reset is incomplete");
  }
  finish_boundary();
  state_.resetting = false;
}

void McuDspRuntime::begin_system_reset(const McuResetSource source,
                                       const unsigned synchronization_cycles) {
  // The MCU can have latched an owned COP timeout. Only its explicit reset
  // entry may recover that timing failure; the DSP owner must remain healthy.
  if (active_ || faulted_ || timeline_->faulted() || dsp_.clock().faulted() ||
      mcu_.run_clock().native_epoch() != state_.epoch) {
    throw Fault("MCU/DSP reset requires its healthy inactive DSP/native epoch");
  }
  timeline_->validate_boundary();
  const Guard guard(*this);
  consumed();
  if (!inputs_.reset_dsp_first || state_.resetting || state_.shared_write ||
      (state_.waiting && !state_.cop_deadline) ||
      (state_.cop_deadline &&
       (!mcu_.run_clock().faulted() || mcu_.bus().cop_state().reset != McuCopReset::timeout))) {
    throw Fault("MCU/DSP system reset lacks its consumed native reset cause");
  }
  mcu_.begin_system_reset(source, synchronization_cycles);
  state_.epoch = mcu_.run_clock().native_epoch();
  state_.reset_edges = mcu_.run_clock().reset_sysclk_edges();
  state_.offered = false;
  state_.waiting = false;
  state_.cop_deadline = false;
  state_.resetting = true;
}

void McuDspRuntime::validate_dsp_reset_observation() const {
  if (active_ && !observing_signals_) {
    throw Fault("MCU/DSP reset control requires a consumed signal or inactive operation boundary");
  }
  if (active_) {
    if (faulted() || mcu_.run_clock().native_epoch() != state_.epoch) {
      throw Fault("MCU/DSP reset control lost its healthy native epoch");
    }
    timeline_->validate_boundary();
  } else {
    healthy();
    if (mcu_.system_reset_active()) {
      mcu_.validate_reset_boundary();
    } else {
      mcu_.validate_run_boundary();
    }
  }
  consumed();
  if (state_.cop_deadline ||
      (!observing_signals_ && (state_.offered || state_.waiting || state_.shared_write)) ||
      inputs_.epoch_ticks + dsp_.clock().elapsed_ticks() != state_.end_ticks) {
    throw Fault("MCU/DSP reset control cannot discard unconsumed peer time pc=" +
                hex(dsp_.cpu().registers().pc, 6) + " mcu-pc=" + hex(mcu_.cpu().state().pc) +
                " tick=" + std::to_string(state_.end_ticks));
  }
}

void McuDspRuntime::begin_dsp_reset(const OwnDspResetInputs inputs) {
  validate_dsp_reset_observation();
  const Guard guard(*this);
  timeline_->begin_hardware_reset(inputs);
}

void McuDspRuntime::release_dsp_reset() {
  validate_dsp_reset_observation();
  const Guard guard(*this);
  timeline_->release_hardware_reset();
}

McuDspRuntimeCheckpoint McuDspRuntime::checkpoint() const {
  healthy();
  consumed();
  if (state_.offered || state_.waiting || state_.shared_write || state_.cop_deadline) {
    throw Fault("MCU/DSP snapshot requires a consumed operation boundary");
  }
  if (state_.resetting) {
    mcu_.validate_reset_boundary();
  } else {
    mcu_.validate_run_boundary();
  }
  return McuDspRuntimeCheckpoint(std::make_shared<const McuDspRuntimeCheckpoint::Storage>(
      McuDspRuntimeCheckpoint::Storage{.mcu = mcu_.checkpoint(),
                                       .dsp = timeline_->checkpoint(),
                                       .inputs = inputs_,
                                       .state = state_,
                                       .external_identity = external_.identity,
                                       .external_order = external_.order,
                                       .external_budget = external_.maximum_events_per_period}));
}

const McuDspRuntimeCheckpoint::Storage &
McuDspRuntime::saved_state(const McuDspRuntimeCheckpoint &checkpoint,
                           const DspRuntimeEvents &external) {
  if (!checkpoint.storage_) { throw std::invalid_argument("Empty MCU/DSP runtime checkpoint"); }
  const auto &saved = *checkpoint.storage_;
  if (saved.external_identity != external.identity || saved.external_order != external.order ||
      saved.external_budget != external.maximum_events_per_period ||
      static_cast<bool>(external.identity) != static_cast<bool>(external.next_ticks) ||
      static_cast<bool>(external.identity) != static_cast<bool>(external.observe)) {
    throw Fault("MCU/DSP clone requires its matching external identity and contract");
  }
  return saved;
}

McuDspRuntime::McuDspRuntime(const McuDspRuntimeCheckpoint &checkpoint,
                             McuMachine &mcu,
                             OwnDspMachine &dsp,
                             DspRuntimeEvents external)
    : inputs_(saved_state(checkpoint, external).inputs),
      mcu_(mcu),
      dsp_(dsp),
      external_(std::move(external)),
      state_(saved_state(checkpoint, external_).state) {
  const auto &saved = saved_state(checkpoint, external_);
  mcu_.restore(saved.mcu);
  timeline_ = std::make_unique<OwnDspTimeline>(dsp_, saved.dsp);
  consumed();
  // The board commits its matching source cursor after these staged owners.
  // Validate the retained pending deadline on the next offer, before any event.
}
}
