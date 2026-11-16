// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/channels.hpp"

#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "mcu/devices.hpp"
#include "mcu/effects.hpp"
#include "mcu/output.hpp"

#include <cpu12/core.hpp>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <stop_token>

namespace blofeld::vi::mcu {
namespace {
thread_local const Output *current_output = nullptr;
constexpr unsigned known_shift = 8, direction_shift = 16;
constexpr std::uint64_t byte_mask = 0xff;
}

Output::Output(Effects &effects, const Devices &devices, cpu12::Core &core)
    : effects_(effects), devices_(devices), core_(core) {
  if (current_output != nullptr) { throw std::logic_error("Nested MCU output ownership"); }
  current_output = this;
}

Output::~Output() {
  current_output = nullptr;
}

const Output *Output::current() noexcept {
  return current_output;
}

bool Output::put(const Effects::Kind kind,
                 const std::uint64_t value,
                 const std::optional<std::uint64_t> ticks) const noexcept {
  const auto time = ticks ? ticks : devices_.timestamp(core_);
  if (!time) { return effects_.fail(Effects::Error::time); }
  const bool accepted = effects_.append({.ticks = *time, .value = value, .kind = kind});
  if (effects_.yield()) { core_.request_yield(); }
  return accepted;
}

bool deliver(const Effects::Entry &entry,
             PeerPort &peer,
             Shi &shi,
             MidiPort &midi,
             const std::stop_token &stop) {
  switch (entry.kind) {
    case Effects::Kind::data:
      return peer.mcu_data(
          {.mask = static_cast<std::uint8_t>((entry.value >> direction_shift) & byte_mask),
           .known = static_cast<std::uint8_t>((entry.value >> known_shift) & byte_mask),
           .value = static_cast<std::uint8_t>(entry.value & byte_mask)},
          entry.ticks,
          stop);
    case Effects::Kind::strobe: return peer.mcu_strobe(entry.value != 0);
    case Effects::Kind::select: static_cast<void>(shi.select(entry.value != 0)); return true;
    case Effects::Kind::word: return shi.transfer(static_cast<std::uint32_t>(entry.value)).accepted;
    case Effects::Kind::midi:
      return midi.write_output(
          {.ticks = entry.ticks, .byte = static_cast<std::uint8_t>(entry.value)});
    case Effects::Kind::time: midi.publish_time(entry.ticks); return true;
    default: return false;
  }
}

bool blofeld_mcu_shi_select(Shi *const channel, const bool active, Shi::Mode &mode) noexcept {
  if (const auto *output = Output::current()) {
    if (!output->put(Effects::Kind::select, std::uint64_t{active})) { return false; }
    const auto configured = channel->configuration(active);
    if (!configured) { return false; }
    mode = *configured;
    return true;
  }
  const auto selected = channel->select(active);
  if (!selected) { return false; }
  mode = *selected;
  return true;
}

bool blofeld_mcu_shi_mode(Shi *const channel, Shi::Mode &mode, const std::stop_token &stop) {
  const auto selected =
      Output::current() != nullptr ? channel->configuration(true) : channel->wait_mode(stop);
  if (!selected) { return false; }
  mode = *selected;
  return true;
}

bool blofeld_mcu_shi_submit(Shi *const channel,
                            const std::uint32_t word,
                            const std::stop_token &stop) {
  if (const auto *output = Output::current()) { return output->put(Effects::Kind::word, word); }
  return channel->submit(word, stop).accepted;
}

bool blofeld_mcu_peer_interrupt(const PeerPort *const channel, std::uint64_t &generation) noexcept {
  const auto pending = channel->mcu_interrupt();
  if (!pending) { return false; }
  generation = *pending;
  return true;
}

void blofeld_mcu_peer_data(const PeerPort *const channel, PeerPort::Sample &sample) noexcept {
  sample = channel->dsp_data();
}

bool blofeld_mcu_peer_read(PeerPort *const channel,
                           const std::uint8_t mask,
                           const std::uint64_t ticks,
                           PeerPort::Sample &sample) noexcept {
  const auto next = channel->read_dsp_data({.mask = mask, .ticks = ticks});
  if (!next) { return false; }
  sample = *next;
  return true;
}

void blofeld_mcu_peer_hold(PeerPort *const channel) noexcept {
  channel->hold_reset();
}

bool blofeld_mcu_peer_publish(PeerPort *const channel,
                              const PeerPort::Sample &sample,
                              const std::uint64_t ticks,
                              const std::stop_token &stop) {
  if (const auto *output = Output::current()) {
    const auto value = (std::uint64_t{sample.mask} << direction_shift) |
                       (std::uint64_t{sample.known} << known_shift) | sample.value;
    return output->put(Effects::Kind::data, value);
  }
  return channel->mcu_data(sample, ticks, stop);
}

bool blofeld_mcu_peer_strobe(PeerPort *const channel, const bool active) noexcept {
  if (const auto *output = Output::current()) {
    return output->put(Effects::Kind::strobe, std::uint64_t{active});
  }
  return channel->mcu_strobe(active);
}

bool blofeld_mcu_time_publish(ExecutionTime *const channel, const std::uint64_t ticks) noexcept {
  return channel->publish_mcu(ticks);
}

bool blofeld_mcu_time_enter(ExecutionTime *const channel,
                            const std::uint64_t ticks,
                            const std::stop_token &stop) {
  return channel->settle_dsp(ticks, stop) && channel->enter_mcu(ticks, stop);
}

void blofeld_mcu_time_leave(ExecutionTime *const channel) noexcept {
  channel->leave_mcu();
}

void blofeld_mcu_time_hold(ExecutionTime *const channel) noexcept {
  channel->hold_dsp();
}

bool blofeld_mcu_midi_limit(const MidiPort *const channel, std::uint64_t &ticks) noexcept {
  const auto limit = channel->input_limit();
  if (!limit) { return false; }
  ticks = *limit;
  return true;
}

bool blofeld_mcu_midi_peek(const MidiPort *const channel, MidiPort::Event &event) noexcept {
  const auto next = channel->peek_input();
  if (!next) { return false; }
  event = *next;
  return true;
}

void blofeld_mcu_midi_pop(MidiPort *const channel) noexcept {
  channel->pop_input();
}

bool blofeld_mcu_midi_write(MidiPort *const channel, const MidiPort::Event &event) noexcept {
  if (const auto *output = Output::current()) {
    return output->put(Effects::Kind::midi, event.byte, event.ticks);
  }
  return channel->write_output(event);
}

bool blofeld_mcu_midi_time(MidiPort *const channel, const std::uint64_t ticks) noexcept {
  if (const auto *output = Output::current()) { return output->put(Effects::Kind::time, 0, ticks); }
  channel->publish_time(ticks);
  return true;
}

bool blofeld_mcu_midi_await(MidiPort *const channel,
                            const std::uint64_t ticks,
                            const std::stop_token &stop) noexcept {
  return channel->await_input(ticks, stop);
}

void blofeld_mcu_reset_state(const DspReset *const channel, DspReset::State &state) noexcept {
  state = channel->state();
}

bool blofeld_mcu_reset_assert(DspReset *const channel) noexcept {
  return channel->assert_reset();
}
}
