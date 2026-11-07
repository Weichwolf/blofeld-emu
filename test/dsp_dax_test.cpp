// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_dax_registers.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using namespace blofeld;
constexpr DspDaxRates rates{.core_hz = 120000000, .aci_hz = 11289600};
constexpr unsigned first_word = 0x123456, second_word = 0x789abc;
constexpr std::uint64_t first_event = 3, channel_interval = 256;
constexpr unsigned external_clock = 8;

void require(const bool value, const char *message) {
  if (!value) { throw std::runtime_error(message); }
}

template <typename Action> void rejects(Action action) {
  try {
    action();
  } catch (const Fault &) { return; }
  throw std::runtime_error("Expected a DAX register/clock fault");
}

DspDax transmitter() {
  DspDax device;
  device.write(dax::non_audio, 0);
  device.write(dax::audio_a, first_word);
  device.write(dax::audio_b, second_word);
  device.write(dax::control, external_clock);
  device.observe({.aci = false, .rates = rates, .timing = DspDaxTiming::reference_rising_edges});
  device.set_active(true);
  return device;
}

void fifo_deadlines() {
  auto device = transmitter();
  require(!device.request() && device.aci_edges_until_event() == first_event,
          "A loaded stereo FIFO starts at the independently known second ACI rise");
  device.advance_aci(first_event - 1, rates);
  require(!device.request() && device.aci_edges_until_event() == 1,
          "Partial input-clock progress cannot consume a word early");
  device.advance_aci(1, rates);
  require(device.request() && device.aci_edges_until_event() == channel_interval,
          "Actual first-word consumption requests the next DMA pair");
  device.write(dax::audio_a, first_word);
  require(device.request(), "One refill word does not acknowledge a stereo pair");
  device.write(dax::audio_a, second_word);
  require(!device.request(), "The second alias write completes the alternating pair");
  rejects([&] { device.write(dax::audio_a, first_word); });
  device.advance_aci(channel_interval, rates);
  require(!device.request() && device.aci_edges_until_event() == channel_interval,
          "B-word completion promotes the pending B register without an A request");
  device.advance_aci(channel_interval, rates);
  require(device.request(), "The next frame consumes A after exactly 256 ACI input periods");
  device.set_active(false);
  require(!device.request() && !device.aci_edges_until_event(),
          "Disabling the peripheral stops clock deadlines and DMA requests");
}

void span_partitions() {
  auto whole = transmitter();
  auto split = transmitter();
  constexpr std::uint64_t transitions = 4097;
  whole.advance_aci(transitions, rates);
  for (std::uint64_t index = 0; index < transitions; ++index) { split.advance_aci(1, rates); }
  require(whole.read(dax::status, dax::empty | dax::last_block_frame) ==
                  split.read(dax::status, dax::empty | dax::last_block_frame) &&
              whole.aci_edges_until_event() == split.aci_edges_until_event(),
          "Arbitrary host partitions preserve FIFO status and the next word deadline");
  rejects([&] { whole.advance_aci(0, rates); });
  auto changed = rates;
  changed.aci_hz *= 2;
  rejects([&] { whole.advance_aci(1, changed); });
  rejects([&] { static_cast<void>(whole.read(dax::status, dax::underrun)); });
  whole.reset();
  require(!whole.request() && !whole.aci_edges_until_event(),
          "Reset removes active transfer state");
}
}

int main() {
  try {
    fifo_deadlines();
    span_partitions();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
