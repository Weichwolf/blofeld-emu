// SPDX-License-Identifier: GPL-3.0-only
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "mcu/channels.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <future>
#include <iostream>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
using blofeld::vi::DspReset;
using blofeld::vi::ExecutionTime;
using blofeld::vi::MidiPort;
using blofeld::vi::PeerPort;
using blofeld::vi::Shi;
using namespace blofeld::vi::mcu;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void timed_reply() {
  PeerPort peer;
  constexpr PeerPort::Sample untouched{.mask = 0x3f, .known = 0x72, .value = 0xa5};
  constexpr PeerPort::Sample report{.mask = 0xff, .known = 0xff, .value = 0x0d};
  constexpr std::uint64_t tick = 31;
  std::uint64_t generation = tick;
  require(!blofeld_mcu_peer_interrupt(&peer, generation) && generation == tick,
          "An absent request preserves the generation output");
  peer.dsp_data(report);
  require(peer.dsp_ready(true) && peer.dsp_ready(false) &&
              blofeld_mcu_peer_interrupt(&peer, generation) && generation == 1 &&
              blofeld_mcu_peer_strobe(&peer, true) && blofeld_mcu_peer_strobe(&peer, false),
          "Original announcement and actual MCU strobe establish shared reply ownership");
  require(peer.dsp_write(untouched, tick), "The actual DSP owner records one timestamped write");
  PeerPort::Sample sample = report;
  require(!blofeld_mcu_peer_read(&peer, report.mask, tick - 1, sample) && sample == report &&
              blofeld_mcu_peer_read(&peer, report.mask, tick, sample) && sample == untouched,
          "A rejected early read preserves both output and queued payload for its actual time");
  peer.dsp_data(untouched);
  blofeld_mcu_peer_data(&peer, sample);
  require(sample == untouched, "Raw latch views retain mask, unknown bits and value");
}

void queued_input() {
  PeerPort peer;
  constexpr std::uint64_t tick = 37;
  constexpr PeerPort::Sample held{.mask = 0xff, .known = 0x7f, .value = 0xa5};
  blofeld_mcu_peer_hold(&peer);
  require(blofeld_mcu_peer_publish(&peer, held, 0, {}) &&
              peer.sample_mcu_data(tick) == PeerPort::Sample{} && peer.release_mcu_data(tick, {}),
          "Held data stays private until its actual reset-release timestamp");
  require(peer.sample_mcu_data(tick - 1) == PeerPort::Sample{} &&
              peer.sample_mcu_data(tick) == held,
          "Native publication uses the same timestamped queue as the real DSP receiver");
  constexpr unsigned capacity = 256;
  constexpr std::uint8_t mask = 0xff;
  for (unsigned index = 0; index < capacity; ++index) {
    require(blofeld_mcu_peer_publish(
                &peer,
                {.mask = mask, .known = mask, .value = static_cast<std::uint8_t>(index)},
                tick + index + 1,
                {}),
            "Fill the real bounded producer queue with distinct ordered states");
  }
  std::promise<bool> outcome;
  auto completion = outcome.get_future();
  std::jthread producer([&](const std::stop_token &stop) {
    outcome.set_value(blofeld_mcu_peer_publish(&peer, held, tick + capacity + 1, stop));
  });
  const auto full = completion.wait_for(std::chrono::milliseconds(20));
  const auto first = peer.sample_mcu_data(tick + 1);
  const auto freed = completion.wait_for(std::chrono::seconds(1));
  producer.request_stop();
  producer.join();
  require(full == std::future_status::timeout && first.value == 0 &&
              freed == std::future_status::ready && completion.get() &&
              peer.sample_mcu_data(tick + capacity + 1) == held,
          "The genuine receiver wakes the native producer through one shared wait provider");
}

void midi_ownership() {
  MidiPort midi(MidiPort::InputMode::stream);
  constexpr std::uint64_t untouched = 31;
  std::uint64_t limit = untouched;
  require(blofeld_mcu_midi_limit(&midi, limit) && limit == 0,
          "Zero is a valid exclusive MIDI frontier, not an absent value");
  constexpr MidiPort::Event sentinel{.ticks = untouched, .byte = 0xa5};
  MidiPort::Event event = sentinel;
  require(!blofeld_mcu_midi_peek(&midi, event) && event == sentinel,
          "An absent input leaves the result untouched");
  constexpr std::array<std::uint8_t, 2> bytes{0x90, 60};
  require(midi.write_input(bytes, 0) && blofeld_mcu_midi_peek(&midi, event) && event.ticks == 0 &&
              event.byte == bytes[0],
          "The binding observes the original producer's MIDI stream");
  blofeld_mcu_midi_pop(&midi);
  require(midi.peek_input(MidiPort::Receiver::dsp) ==
                  MidiPort::Event{.ticks = 0, .byte = bytes[0]} &&
              blofeld_mcu_midi_peek(&midi, event) && event.byte == bytes[1] &&
              event.ticks == MidiPort::byte_ticks,
          "MCU retirement cannot consume the DSP owner's input or alter serial timing");
  require(blofeld_mcu_midi_write(&midi, event) && midi.read_output() == event &&
              midi.finish_input() && !blofeld_mcu_midi_limit(&midi, limit) && limit == 0 &&
              blofeld_mcu_midi_await(&midi, event.ticks, {}),
          "Actual output and end-of-stream admission share the original native channels");
}

void reset_and_time() {
  DspReset reset;
  DspReset::State state;
  blofeld_mcu_reset_state(&reset, state);
  require(!state.held && state.epoch == 0 && blofeld_mcu_reset_assert(&reset),
          "A native assertion advances the actual reset epoch");
  blofeld_mcu_reset_state(&reset, state);
  require(state.held && state.epoch == 1 && !reset.acknowledge(2),
          "The binding cannot acknowledge or fabricate a future reset generation");
  ExecutionTime time;
  blofeld_mcu_time_hold(&time);
  constexpr std::uint64_t tick = 37;
  require(blofeld_mcu_time_publish(&time, tick) && !blofeld_mcu_time_publish(&time, tick - 1) &&
              blofeld_mcu_time_enter(&time, tick, {}),
          "Local publication is monotonic and an actually held DSP permits MCU access");
  blofeld_mcu_time_leave(&time);
  require(blofeld_mcu_time_enter(&time, tick, {}),
          "Leaving the real transaction releases ownership even at the same timestamp");
  blofeld_mcu_time_leave(&time);
}

void shi_selection() {
  Shi shi;
  constexpr std::uint32_t control = 0xffff91;
  constexpr std::uint32_t clock = 0xffff90;
  constexpr unsigned word_bits = 24;
  require(shi.write({.address = control, .value = 9}) && shi.write({.address = clock, .value = 0}),
          "Configure the actual single-word phase-zero SPI mode");
  Shi::Mode mode;
  require(blofeld_mcu_shi_select(&shi, true, mode) && mode.bits == word_bits && !mode.continuous,
          "MCU boot selection observes the DSP owner's actual SHI configuration");
  require(!blofeld_mcu_shi_select(&shi, false, mode) && mode.bits == word_bits,
          "An absent selection preserves output and cannot invent a transfer mode");
}
}

int main() try {
  timed_reply();
  queued_input();
  midi_ownership();
  reset_and_time();
  shi_selection();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
