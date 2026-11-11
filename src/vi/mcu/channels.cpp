// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/channels.hpp"

#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"

#include <cstdint>
#include <stop_token>

namespace blofeld::vi::mcu {
bool blofeld_mcu_shi_select(Shi *const channel, const bool active, Shi::Mode &mode) noexcept {
  const auto selected = channel->select(active);
  if (!selected) { return false; }
  mode = *selected;
  return true;
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
  channel->hold_mcu_data();
}

bool blofeld_mcu_peer_publish(PeerPort *const channel,
                              const PeerPort::Sample &sample,
                              const std::uint64_t ticks,
                              const std::stop_token &stop) {
  return channel->mcu_data(sample, ticks, stop);
}

bool blofeld_mcu_peer_strobe(PeerPort *const channel, const bool active) noexcept {
  return channel->mcu_strobe(active);
}

bool blofeld_mcu_time_publish(ExecutionTime *const channel, const std::uint64_t ticks) noexcept {
  return channel->publish_mcu(ticks);
}

bool blofeld_mcu_time_enter(ExecutionTime *const channel,
                            const std::uint64_t ticks,
                            const std::stop_token &stop) {
  return channel->enter_mcu(ticks, stop);
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
  return channel->write_output(event);
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
