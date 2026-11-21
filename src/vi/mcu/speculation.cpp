// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/speculation.hpp"

#if defined(BLOFELD_CENSUS)
#include "census.hpp"
#include "mcu/state_probe.hpp"

#include <array>
#endif

#include "display/state.hpp"
#include "execution.hpp"
#include "io/controls.hpp"
#include "io/coupling.hpp"
#include "io/dsp_reset.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "mcu/devices.hpp"
#include "mcu/effects.hpp"
#include "mcu/epoch.hpp"
#include "mcu/io.hpp"
#include "mcu/memory.hpp"
#include "mcu/output.hpp"
#include "mcu/program.hpp"

#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <format>
#include <initializer_list>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace blofeld::vi::mcu {
namespace {
constexpr unsigned known_shift = 8, direction_shift = 16;
constexpr std::uint64_t byte_mask = 0xff;
constexpr cpu12::Address xirq_vector = 0xfff4;

}

Speculation::Speculation(Coupling &coupling,
                         Memory &memory,
                         Devices &devices,
                         Io &io,
                         cpu12::Core &core,
                         const Program &program,
                         PeerPort &peer,
                         MidiPort &midi,
                         DspReset &reset,
                         ExecutionReport &report,
                         DisplayState *display,
                         const Controls *controls)
    : coupling_(coupling),
      memory_(memory),
      devices_(devices),
      io_(io),
      core_(core),
      program_(program),
      peer_(peer),
      midi_(midi),
      report_(report),
      display_(display),
      controls_(controls),
      effects_(coupling.source_storage(), 64) {
  state_.reset = reset.state();
  pedal_ = controls_ != nullptr && controls_->pedal();
}

std::uint64_t Speculation::ticks() const noexcept {
  return devices_.timestamp(core_).value_or(std::numeric_limits<std::uint64_t>::max());
}

void Speculation::publish() noexcept {
  coupling_.publish(effects_.entries(), ticks());
}

bool Speculation::observe(const Coupling::Input &update) noexcept {
  bool valid = true;
  const auto now = ticks();
  if (state_.ready && !epoch_.atomic() && update.ticks >= state_.irq_window &&
      update.ticks <= now) {
    for (const auto signal : {Epoch::Signal::interrupt, Epoch::Signal::announce}) {
      const auto before = history_.at(now, signal);
      if (!before ||
          !epoch_.record(
              {.ticks = now, .value = *before, .mask = ~std::uint64_t{0}, .signal = signal})) {
        return false;
      }
    }
  }
  for (std::size_t index = 0; index < update.values.size(); ++index) {
    const Epoch::Read change{.ticks = update.ticks,
                             .value = update.values[index],
                             .mask = ~std::uint64_t{0},
                             .signal = static_cast<Epoch::Signal>(index)};
    if (!history_.write(change)) { return false; }
    valid = epoch_.changed(change) && valid;
  }
  if (history_.yield() || epoch_.yield()) {
    action_ = Action::confirm;
    core_.request_yield();
    if (valid && !epoch_.atomic()) { valid = epoch_.restart(update.ticks); }
  }
  return valid;
}

bool Speculation::poll() noexcept {
  if (coupling_.stopped()) { return false; }
  if (const auto update = coupling_.input(seen_)) {
    seen_ = update->serial;
    pending_ = update;
    const bool valid = observe(*update);
    if (valid) {
      coupling_.acknowledge(update->serial);
      pending_.reset();
    }
  }
  if (epoch_.cancelled()) { return false; }
  if (history_.error() != Epoch::Error::none || epoch_.error() != Epoch::Error::none) {
    return false;
  }
  return epoch_.attained(coupling_.known());
}

bool Speculation::cancelled() noexcept {
  if (!poll()) {
    if (epoch_.cancelled()) { static_cast<void>(epoch_.refuse()); }
    return true;
  }
  publish();
  return false;
}

std::optional<std::uint64_t> Speculation::input(const Epoch::Signal signal,
                                                const std::uint64_t time,
                                                const std::uint64_t mask) noexcept {
  if (!poll()) { return {}; }
  const auto value = history_.at(time, signal);
  if (!value || !epoch_.record({.ticks = time, .value = *value, .mask = mask, .signal = signal})) {
    return {};
  }
  if (epoch_.yield()) { core_.request_yield(); }
  return value;
}

bool Speculation::emit(const Effects::Kind kind,
                       const std::uint64_t value,
                       const std::uint64_t time) noexcept {
  if (!poll()) { return epoch_.refuse(); }
  if (kind != Effects::Kind::midi && !devices_.advance(core_)) { return epoch_.refuse(); }
  if (kind == Effects::Kind::data) { state_.data = static_cast<std::uint32_t>(value); }
  if (kind == Effects::Kind::strobe) {
    const bool next = value != 0;
    if (next && !state_.strobe) {
      const auto announcement = input(Epoch::Signal::announce, time);
      if (!announcement) { return epoch_.refuse(); }
      if (*announcement != 0 && *announcement != state_.accepted_announcement) {
        state_.accepted_announcement = *announcement;
        state_.reply = true;
      }
    }
    state_.strobe = next;
  }
  const bool accepted = effects_.append({.ticks = time, .value = value, .kind = kind});
  if (effects_.yield()) { core_.request_yield(); }
  publish();
  return accepted;
}

bool Speculation::settle(const std::uint64_t time) noexcept {
  publish();
  coupling_.request(time);
  while (!coupling_.stopped()) {
    const auto observed = coupling_.observation();
    if (!poll()) { return epoch_.refuse(); }
    if (epoch_.settled(time)) { return true; }
    coupling_.wait(observed);
  }
  return false;
}

bool Speculation::data(PeerPort::Sample &sample) noexcept {
  return data(sample, byte_mask);
}

bool Speculation::data(PeerPort::Sample &sample, const std::uint8_t mask) noexcept {
  if (state_.reset.held) {
    sample = {};
    return true;
  }
  const auto observed = std::uint64_t{mask} | (std::uint64_t{mask} << known_shift) |
                        (std::uint64_t{mask} << direction_shift);
  const auto value = input(Epoch::Signal::data, ticks(), observed);
  if (!value) { return epoch_.refuse(); }
  sample = {.mask = static_cast<std::uint8_t>((*value >> direction_shift) & byte_mask),
            .known = static_cast<std::uint8_t>((*value >> known_shift) & byte_mask),
            .value = static_cast<std::uint8_t>(*value & byte_mask)};
  return true;
}

bool Speculation::read(const std::uint8_t mask,
                       const std::uint64_t time,
                       PeerPort::Sample &sample) noexcept {
  if (state_.reset.held) {
    sample = {};
    return true;
  }
  const auto direction = static_cast<std::uint8_t>((state_.data >> direction_shift) & byte_mask);
  if (mask != byte_mask || direction != 0 || !state_.reply) { return data(sample, mask); }
  if (!settle(time)) { return false; }
  const bool accepted = epoch_.consume(time, [&] {
    const auto value = peer_.read_dsp_data({.mask = mask, .direction = direction, .ticks = time});
    if (!value) { return false; }
    sample = *value;
    state_.reply = peer_.view().reply;
    return true;
  });
  if (accepted) {
#if defined(BLOFELD_CENSUS)
    Census::event(Census::Event::destructive, time);
#endif
    action_ = Action::confirm;
    core_.request_yield();
  }
  return accepted;
}

std::optional<std::uint64_t> Speculation::pending_irq() noexcept {
  if (state_.reset.held || !state_.ready) { return {}; }
  const auto time = ticks();
  const auto level = input(Epoch::Signal::interrupt, time, 1);
  const auto announcement = input(Epoch::Signal::announce, time);
  const auto generation = input(Epoch::Signal::generation, time);
  if (!level || !announcement || !generation) { return {}; }
  if (*level != 0 || (*announcement != 0 && *announcement != state_.accepted_announcement) ||
      history_.raised(Epoch::Signal::interrupt, state_.irq_window, time)) {
    return generation;
  }
  return {};
}

bool Speculation::irq(std::uint64_t &generation) noexcept {
  const auto pending = pending_irq();
  if (!pending) { return false; }
  generation = *pending;
  return true;
}

bool Speculation::mode(const bool selected, const bool wait, Shi::Mode &mode) noexcept {
  if (!selected || state_.reset.held) { return false; }
  const auto time = ticks();
  if (wait && !settle(time)) { return false; }
  const auto value = input(Epoch::Signal::mode, time);
  if (!value || *value == 0) {
    if (wait && !epoch_.cancelled()) {
      refused_ = "SPI mode is unknown at its settled MCU instant";
    }
    return false;
  }
  mode = {.bits = static_cast<unsigned>(*value & byte_mask),
          .falling = (*value & (std::uint64_t{1} << known_shift)) != 0,
          .continuous = (*value & (std::uint64_t{1} << (known_shift + 1))) != 0};
  return true;
}

bool Speculation::await_midi(const std::uint64_t time) noexcept {
  publish();
  while (!coupling_.stopped()) {
    const auto observed = coupling_.observation();
    if (!poll()) { return epoch_.refuse(); }
    const auto limit = midi_.input_limit();
    if (!limit || *limit > time) { return true; }
    midi_.request_input(time);
    midi_.before_mcu_wait(time);
    coupling_.wait(observed);
  }
  return false;
}

bool Speculation::midi(MidiPort::Event &event) const noexcept {
  const auto next = midi_.peek_input(MidiPort::Receiver::mcu, state_.input_cursor);
  if (!next) { return false; }
  event = *next;
  return true;
}

void Speculation::pop_midi() noexcept {
  ++state_.input_cursor;
}

DspReset::State Speculation::reset_state() const noexcept {
  return state_.reset;
}

bool Speculation::assert_reset() noexcept {
  if (!state_.reset.held) {
    if (state_.reset.epoch == std::numeric_limits<std::uint64_t>::max()) { return false; }
    ++state_.reset.epoch;
    state_.reset.held = true;
    state_.irq_window = ticks();
  }
  return emit(Effects::Kind::reset_hold, state_.reset.epoch, ticks());
}

bool Speculation::release_reset() noexcept {
  state_.reset.held = false;
  return emit(Effects::Kind::reset_release, state_.reset.epoch, ticks());
}

bool Speculation::check() const {
  if (refused_ != nullptr) {
    throw std::runtime_error(std::format("{}; PC=${:04x} time={} known={} reset={} epoch={}",
                                         refused_,
                                         core_.registers().pc,
                                         ticks(),
                                         coupling_.known(),
                                         state_.reset.held,
                                         state_.reset.epoch));
  }
  if (coupling_.error() != nullptr) { throw std::runtime_error(coupling_.error()); }
  if (effects_.error() != Effects::Error::none || epoch_.error() != Epoch::Error::none ||
      history_.error() != Epoch::Error::none) {
    const auto status = effects_.diagnostic();
    throw std::runtime_error(
        std::format("MCU speculation invariant: output={} read={} history={} PC=${:04x}; "
                    "expected={}/{}/{} actual={}/{}/{} attained={} written={} applied={} "
                    "replayed={} time={} known={}",
                    static_cast<unsigned>(effects_.error()),
                    static_cast<unsigned>(epoch_.error()),
                    static_cast<unsigned>(history_.error()),
                    core_.registers().pc,
                    static_cast<unsigned>(status.expected.kind),
                    status.expected.ticks,
                    status.expected.value,
                    static_cast<unsigned>(status.actual.kind),
                    status.actual.ticks,
                    status.actual.value,
                    status.attained,
                    status.written,
                    status.applied,
                    status.replayed,
                    ticks(),
                    coupling_.known()));
  }
  if (controls_ != nullptr && controls_->pedal() != pedal_) {
    throw std::runtime_error("D2 external pedal history is not bound");
  }
  return !coupling_.stopped() && core_.state().execution != cpu12::ExecutionState::stopped;
}

void Speculation::bind() {
  const bool mapping = memory_.commit_mapping();
  const bool usb = devices_.usb.commit_mapping();
  if (!mapping && !usb && program_.contains(mounted_, core_.registers().pc)) { return; }
  auto selected = program_.admit(memory_, core_.registers().pc);
  auto next = memory_.regions();
  const auto base = static_cast<std::uint16_t>(unsigned{memory_.read_mapping(0x11)} << known_shift);
  for (const auto region : devices_.usb.regions(base)) {
    if (!region.bytes.empty()) { next.push_back(region); }
  }
  core_.set_memory(cpu12::Memory::mapped(next));
  program_.install(core_, selected);
  if (binding_ == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("MCU binding generation exhausted");
  }
  ++binding_;
  grants_.swap(next);
  mounted_.swap(selected);
}

void Speculation::advance() {
  if (!devices_.advance(core_)) {
    if (coupling_.stopped() || epoch_.cancelled()) { return; }
    static_cast<void>(check());
    throw std::runtime_error(devices_.error() != nullptr ? devices_.error()
                                                         : "MCU device clock failed");
  }
}

void Speculation::ready() {
  const bool now = core_.interrupt_ready(cpu12::InterruptKind::xirq);
  if (now != state_.ready) {
    state_.ready = now;
    state_.irq_window = ticks();
  }
}

Speculation::Checkpoint Speculation::snapshot() const {
  auto state = state_;
  state.instructions = report_.instructions;
  return {.cpu = core_.save(),
          .memory = memory_.save(),
          .devices = devices_.save(),
          .io = io_.save(),
          .state = state,
          .binding = binding_};
}

void Speculation::save() {
  checkpoint_.emplace(snapshot());
}

bool Speculation::stage_ordinary(const std::uint64_t target) {
  const auto time = ticks();
  if (!midi_.rendering_at(time) || !check() || time < target || action_ != Action::run ||
      epoch_.atomic() || state_.reset.held || replaying_ || effects_.yield() || epoch_.yield() ||
      history_.yield()) {
    return false;
  }
  if (further_windows_ != 0) {
    --further_windows_;
    return true;
  }
  if (staged_ || core_.state().execution != cpu12::ExecutionState::waiting) { return false; }
  ready();
  staged_.emplace(
      StagedCheckpoint{.tick = time, .effects = effects_.size(), .snapshot = snapshot()});
  further_windows_ = 2;
  return true;
}

void Speculation::promote_staged() {
  if (!staged_ || state_.reset.held || epoch_.atomic() || action_ != Action::run ||
      effects_.yield() || epoch_.yield() || history_.yield() ||
      coupling_.known() <= staged_->tick || !poll()) {
    return;
  }
  auto &saved = *staged_;
  const auto known = coupling_.known();
  const auto consumed = coupling_.consumed();
  if (consumed < saved.effects) { return; }
  const auto cursor = saved.snapshot.state.input_cursor;
  if (state_.input_cursor < cursor ||
      !effects_.promote(
          {.ticks = saved.tick, .known = known, .position = saved.effects, .received = consumed}) ||
      !epoch_.promote(saved.tick) || !history_.retire(saved.tick)) {
    throw std::logic_error("An ordinary checkpoint could not promote its settled prefix");
  }
  midi_.publish_time(saved.tick);
  for (std::uint64_t count = 0; count < cursor; ++count) { midi_.pop_input(); }
  state_.input_cursor -= cursor;
  saved.snapshot.state.input_cursor = 0;
  if (display_ != nullptr) { display_->capture(saved.snapshot.io.display.controller()); }
  checkpoint_.emplace(std::move(saved.snapshot));
  staged_.reset();
}

void Speculation::restore() {
#if defined(BLOFELD_CENSUS)
  const auto conflict = epoch_.conflict();
  if (!conflict) { throw std::logic_error("MCU rollback census has no conflict timestamp"); }
  const auto cause = epoch_.census().cause;
#endif
  if (!checkpoint_ || !effects_.acknowledge(coupling_.consumed(), coupling_.cut()) ||
      !effects_.rollback() || !epoch_.rollback()) {
    throw std::logic_error("MCU rollback has no coherent retained prefix");
  }
  const bool retained =
      checkpoint_->binding == binding_ && memory_.restore_bound(checkpoint_->memory);
#if defined(BLOFELD_CENSUS)
  constexpr std::array causes{Census::Event::data,
                              Census::Event::interrupt,
                              Census::Event::mode,
                              Census::Event::reply,
                              Census::Event::generation,
                              Census::Event::announce,
                              Census::Event::pressure};
  Census::event(Census::Event::rollback, conflict.value());
  Census::event(Census::Event::discarded,
                conflict.value(),
                report_.instructions - checkpoint_->state.instructions);
  Census::event(causes[static_cast<unsigned>(cause)], conflict.value());
  if (!retained) { Census::event(Census::Event::reinstall, conflict.value()); }
  if (auto *probe = StateProbe::active()) {
    probe->rollback(report_.instructions - checkpoint_->state.instructions, retained);
  }
#endif
  if (!retained) { memory_.restore(checkpoint_->memory); }
  devices_.restore(checkpoint_->devices);
  io_.restore(checkpoint_->io);
  core_.restore(checkpoint_->cpu);
  state_ = checkpoint_->state;
  report_.instructions = state_.instructions;
  if (!retained) { bind(); }
  publish();
}

void Speculation::confirm() {
  const auto time = ticks();
  promote_staged();
  further_windows_ = 0;
  publish();
  coupling_.request(time);
  coupling_.checkpoint(time);
  while (check() && !coupling_.pause_after(time)) {
    const auto observed = coupling_.observation();
    if (!poll()) {
      if (epoch_.cancelled()) {
        recover();
        return;
      }
      static_cast<void>(check());
      return;
    }
    promote_staged();
    if (!coupling_.pause_after(time)) { coupling_.wait_checkpoint(observed); }
  }
  if (!check()) { return; }
  promote_staged();
  staged_.reset();
  ready();
  if (!effects_.acknowledge(coupling_.consumed(), coupling_.cut()) || !effects_.checkpoint(time) ||
      !epoch_.attained(coupling_.known()) ||
      !(state_.reset.held && coupling_.known() == time ? epoch_.checkpoint_quiescent(time)
                                                       : epoch_.checkpoint(time)) ||
      !history_.retire(time)) {
    throw std::logic_error("MCU commit did not confirm the complete native boundary");
  }
  midi_.publish_time(time);
  for (std::uint64_t count = 0; count < state_.input_cursor; ++count) { midi_.pop_input(); }
  state_.input_cursor = 0;
  if (display_ != nullptr) { io_.capture_display(*display_); }
  save();
#if defined(BLOFELD_CENSUS)
  if (auto *probe = StateProbe::active()) {
    probe->commit(report_.instructions, core_.progress(), time);
  }
#endif
  coupling_.replace(time);
  action_ = Action::run;
}

void Speculation::recover() {
  if (!pending_) { throw std::logic_error("Invalidated MCU epoch has no source event"); }
  staged_.reset();
  further_windows_ = 0;
  coupling_.cancel_checkpoint();
  const auto target = pending_->ticks;
  const auto serial = pending_->serial;
  restore();
  replaying_ = true;
  while (check() && ticks() < target) { step(target); }
  replaying_ = false;
  // Stop can cut this replay short while its receiver is shutting down.
  if (coupling_.stopped()) {
    static_cast<void>(check());
    return;
  }
  if (!effects_.finish_replay()) {
    const auto status = effects_.diagnostic();
    std::string prefix;
    for (const auto &entry : effects_.entries()) {
      prefix +=
          std::format(" [{}/{}/{}]", static_cast<unsigned>(entry.kind), entry.ticks, entry.value);
    }
    throw std::logic_error(std::format("MCU replay omitted an applied output; target={} time={} "
                                       "applied={} replayed={} known={} entries={}",
                                       target,
                                       ticks(),
                                       status.applied,
                                       status.replayed,
                                       coupling_.known(),
                                       prefix));
  }
  pending_.reset();
  coupling_.acknowledge(serial);
  publish();
}

bool Speculation::interrupt() {
  const auto external = pending_irq();
  const auto local = devices_.pending();
  if (external || (local && core_.interrupt_ready(cpu12::InterruptKind::maskable))) {
    active_ = true;
    const auto entered = core_.interrupt(
        {.vector = external ? xirq_vector : *local,
         .kind = external ? cpu12::InterruptKind::xirq : cpu12::InterruptKind::maskable});
    active_ = false;
    if (entered.fault != cpu12::Fault::none) {
      if (epoch_.refusal() == Epoch::Refusal::epoch_invalidated) { return true; }
      throw std::runtime_error("D2 actual MCU interrupt entry failed");
    }
    if (entered.accepted) {
      if (external) { static_cast<void>(emit(Effects::Kind::acknowledge, *external, ticks())); }
      devices_.activity(cpu12::ExecutionState::running);
      ready();
      return true;
    }
  }
  return false;
}

std::uint64_t Speculation::deadline(const std::uint64_t target) const {
  auto deadline = devices_.clock.at_ticks(target);
  if (!deadline) { throw std::overflow_error("D2 MCU window exceeds its clock domain"); }
  if (const auto local_deadline = devices_.deadline();
      local_deadline && *local_deadline < *deadline) {
    deadline = local_deadline;
  }
  if (const auto input_change = history_.next_irq(devices_.clock.ticks());
      input_change && *input_change < target) {
    if (const auto at = devices_.clock.at_ticks(*input_change); at && *at < *deadline) {
      deadline = at;
    }
  }
  return *deadline;
}

void Speculation::step(const std::uint64_t target) {
  if (!poll()) {
    if (epoch_.cancelled() && !active_ && !replaying_) { recover(); }
    return;
  }
  advance();
  if (!check() || epoch_.cancelled()) { return; }
  ready();
  if (interrupt()) { return; }
  if (core_.state().returned_frame) { core_.close_return_window(); }
  const auto until = deadline(target);
  if (core_.state().execution == cpu12::ExecutionState::waiting) {
    devices_.idle = until - core_.progress().estimated_cycles;
    advance();
    publish();
    return;
  }
  bind();
  active_ = true;
  const auto result = core_.run_until(until - devices_.idle);
  active_ = false;
  report_.instructions += result.retired;
#if defined(BLOFELD_CENSUS)
  if (auto *probe = StateProbe::active()) { probe->executed(result.retired, replaying_); }
#endif
  if (coupling_.stopped()) { return; }
  if (result.fault != cpu12::Fault::none) {
    if (epoch_.refusal() == Epoch::Refusal::epoch_invalidated) { return; }
    static_cast<void>(check());
    throw std::runtime_error(
        std::format("D2 MCU fault={} PC=${:04x} address=${:04x} value=${:04x}; reason={}; {}",
                    static_cast<unsigned>(result.fault),
                    result.pc,
                    result.address,
                    result.value,
                    io_.error() != nullptr ? io_.error() : "device refused access",
                    io_.context()));
  }
  if (!io_.complete_events()) {
    if (epoch_.cancelled() || coupling_.stopped()) { return; }
    static_cast<void>(check());
    throw std::runtime_error(
        std::format("D2 MCU completion failed; PC=${:04x} time={} known={} reset={} epoch={}; {}",
                    core_.registers().pc,
                    ticks(),
                    coupling_.known(),
                    state_.reset.held,
                    state_.reset.epoch,
                    io_.error() != nullptr ? io_.error() : io_.context()));
  }
  advance();
  if (result.state == cpu12::ExecutionState::waiting) {
    static_cast<void>(emit(Effects::Kind::wait, 0, devices_.clock.ticks()));
  }
  devices_.activity(result.state);
#if defined(BLOFELD_CENSUS)
  record_wait(result);
#endif
  publish();
  if (epoch_.atomic() && !replaying_) { confirm(); }
}

#if defined(BLOFELD_CENSUS)
void Speculation::record_wait(const cpu12::RunResult &result) {
  if (result.state == cpu12::ExecutionState::waiting && result.retired != 0) {
    if (auto *probe = StateProbe::active()) {
      probe->wait(StateProbe::capture(core_, memory_, devices_, io_, report_.instructions), true);
      if (!replaying_ && probe->stopping(ticks())) { confirm(); }
    }
  }
}
#endif

void Speculation::run() {
  bind();
  ready();
  save();
  const Output channels(effects_, devices_, core_, this);
  coupling_.bind();
  publish();
  while (check()) {
    if (action_ == Action::confirm || history_.yield()) {
      confirm();
      continue;
    }
    const auto origin = ticks();
    if (coupling_.window() > std::numeric_limits<std::uint64_t>::max() - origin) {
      throw std::overflow_error("MCU speculative window exceeds its clock domain");
    }
    const auto target = origin + coupling_.window();
    while (check() && ticks() < target && !effects_.yield() && !epoch_.yield() &&
           !history_.yield()) {
      step(target);
      if (staged_) { promote_staged(); }
    }
    if (epoch_.cancelled()) {
      recover();
      continue;
    }
    if (!stage_ordinary(target)) { confirm(); }
  }
}
}
