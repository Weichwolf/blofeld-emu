// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/channels.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <future>
#include <iostream>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
using blofeld::vi::PeerPort;
using blofeld::vi::Shi;
using namespace blofeld::vi::dsp;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void shi_fifo() {
  Shi shi;
  constexpr std::uint32_t control = 0xffff91;
  constexpr std::uint32_t receive = 0xffff94;
  constexpr std::uint32_t single_word = 9;
  constexpr std::uint32_t payload = 0x53a2f1;
  constexpr std::uint32_t unknown = 0x123456;
  std::uint32_t value = unknown;
  require(!blofeld_dsp_shi_read(&shi, receive, value) && value == unknown,
          "An empty native FIFO fails without overwriting the caller's value");
  require(blofeld_dsp_shi_write(&shi, control, single_word) && shi.select(true).has_value() &&
              shi.transfer(payload).accepted,
          "Native transport and compiled register access share the same FIFO");
  require(blofeld_dsp_shi_read(&shi, receive, value) && value == payload &&
              !blofeld_dsp_shi_read(&shi, receive, value),
          "The real payload is consumed once, without a fabricated subsequent reply");
  require(!blofeld_dsp_shi_write(&shi, control, 3),
          "The binding preserves unsupported register-mode failure");
}

void fifo_wakeup() {
  Shi shi;
  constexpr std::uint32_t control = 0xffff91;
  constexpr std::uint32_t receive = 0xffff94;
  constexpr std::uint32_t fifo_mode = 0x29;
  constexpr std::uint32_t payload = 0xa5;
  constexpr unsigned capacity = 10;
  require(blofeld_dsp_shi_write(&shi, control, fifo_mode) && shi.select(true).has_value(),
          "Enable the actual bounded native FIFO");
  for (unsigned index = 0; index < capacity; ++index) {
    require(shi.transfer(index).accepted, "Fill each owned FIFO slot before producer submission");
  }
  std::promise<Shi::Transfer> outcome;
  auto completion = outcome.get_future();
  std::jthread producer(
      [&](const std::stop_token &stop) { outcome.set_value(shi.submit(payload, stop)); });
  const auto full = completion.wait_for(std::chrono::milliseconds(20));
  std::uint32_t value = payload;
  const bool consumed = blofeld_dsp_shi_read(&shi, receive, value);
  const auto freed = completion.wait_for(std::chrono::seconds(1));
  producer.request_stop();
  producer.join();
  require(full == std::future_status::timeout && consumed && value == 0 &&
              freed == std::future_status::ready && completion.get().accepted,
          "Imported register reads wake the same native producer waiting on actual FIFO space");
  for (unsigned index = 1; index <= capacity; ++index) {
    const auto expected = index == capacity ? payload : index;
    require(blofeld_dsp_shi_read(&shi, receive, value) && value == expected,
            "Ownership handoff preserves every queued word and the resumed producer payload");
  }
  require(!blofeld_dsp_shi_read(&shi, receive, value), "The shared FIFO is actually empty");
}

void timed_input() {
  PeerPort peer;
  constexpr PeerPort::Sample first{.mask = 0xff, .known = 0x7f, .value = 0xa5};
  constexpr PeerPort::Sample second{.mask = 0x3f, .known = 0xf0, .value = 0x52};
  constexpr std::uint64_t first_tick = 17;
  constexpr std::uint64_t second_tick = 29;
  require(peer.mcu_data(first, first_tick) && peer.mcu_data(second, second_tick),
          "Publish two actual timestamped producer changes");
  PeerPort::Sample sample = second;
  blofeld_dsp_peer_sample(&peer, first_tick - 1, sample);
  require(sample == PeerPort::Sample{}, "Native imports cannot read future producer state");
  blofeld_dsp_peer_sample(&peer, first_tick, sample);
  require(sample == first, "The binding retains mask, unknown bits and the actual value");
  blofeld_dsp_peer_sample(&peer, second_tick - 1, sample);
  require(sample == first, "Repeated input sampling does not consume a future transaction");
  blofeld_dsp_peer_sample(&peer, second_tick, sample);
  require(sample == second, "Only the actual instrument frontier admits the next input");
}

void report_output() {
  PeerPort peer;
  constexpr std::uint8_t byte_mask = 0xff;
  constexpr std::uint8_t report = 0x0d;
  blofeld_dsp_peer_data(&peer, byte_mask, byte_mask, report);
  require(blofeld_dsp_peer_ready(&peer, true) && blofeld_dsp_peer_ready(&peer, false) &&
              peer.mcu_interrupt() == 1 && peer.mcu_strobe(true) && peer.mcu_strobe(false),
          "The native report announcement and genuine MCU request establish response ownership");
  constexpr PeerPort::Sample data{.mask = 0xff, .known = 0x73, .value = 0xa5};
  blofeld_dsp_peer_data(&peer, data.mask, data.known, data.value);
  require(peer.dsp_data() == data, "Output publication retains unknown bits and drive direction");
  constexpr std::uint64_t written = 31;
  require(blofeld_dsp_peer_write(&peer, written) && blofeld_dsp_peer_ready(&peer, true),
          "An actual data write records the current sample with its original timestamp");
  require(!peer.read_dsp_data({.mask = byte_mask, .ticks = written - 1}) &&
              peer.read_dsp_data({.mask = byte_mask, .ticks = written}) == data,
          "A future reply read fails without consuming the later original data");
}

void interrupt_generations() {
  PeerPort peer;
  constexpr std::uint64_t untouched = 37;
  std::uint64_t generation = untouched;
  require(!blofeld_dsp_peer_interrupt(&peer, true, generation) && generation == untouched,
          "A missing interrupt neither invents a generation nor overwrites the output");
  require(peer.mcu_strobe(true) && peer.mcu_strobe(false) &&
              !blofeld_dsp_peer_interrupt(&peer, false, generation) &&
              blofeld_dsp_peer_interrupt(&peer, true, generation) && generation == 1,
          "The binding preserves actual level versus retained-edge admission");
  require(peer.mcu_strobe(true) && peer.acknowledge_dsp(generation) &&
              blofeld_dsp_peer_interrupt(&peer, true, generation) && generation == 2,
          "Acknowledging an older edge cannot erase the newer native request");
}
}

int main() try {
  shi_fifo();
  fifo_wakeup();
  timed_input();
  report_output();
  interrupt_generations();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
