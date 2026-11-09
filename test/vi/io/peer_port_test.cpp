// SPDX-License-Identifier: GPL-3.0-only
#include "io/peer_port.hpp"

#include <atomic>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
using blofeld::vi::PeerPort;
constexpr std::uint8_t drive_tag = 0x55;
constexpr std::uint8_t knowledge_tag = 0xaa;
constexpr std::uint64_t byte_mask = 0xff;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void data_views() {
  PeerPort peer;
  require(peer.mcu_data() == PeerPort::Sample{} && peer.dsp_data() == PeerPort::Sample{},
          "Reset has no attached driver or manufactured payload");
  constexpr PeerPort::Sample mcu{.mask = 0xff, .known = 0xf0, .value = 0xa5};
  constexpr PeerPort::Sample dsp{.mask = 0x7f, .known = 0x73, .value = 0x52};
  peer.mcu_data(mcu);
  peer.dsp_data(dsp);
  require(peer.mcu_data() == mcu && peer.mcu_data() == mcu && peer.dsp_data() == dsp,
          "Each endpoint retains its genuine drive mask, unknown bits and output latch");
  peer.reset_dsp();
  require(peer.mcu_data() == mcu && peer.dsp_data() == PeerPort::Sample{},
          "DSP reset releases only its output; the MCU's actual driver is retained");
}

void requests() {
  PeerPort peer;
  require(!peer.dsp_interrupt(true) && !peer.dsp_interrupt(false) && !peer.mcu_interrupt(),
          "No request precedes its actual source");
  require(peer.mcu_strobe(true) && peer.dsp_interrupt(true) == 1 && peer.dsp_interrupt(false) == 1,
          "The first low assertion provides edge and level requests");
  require(peer.mcu_strobe(true) && peer.dsp_interrupt(true) == 1,
          "Writing the asserted level again does not create another edge");
  require(peer.mcu_strobe(false) && !peer.dsp_interrupt(false) && peer.dsp_interrupt(true) == 1,
          "Release clears a level but retains the posted edge");
  require(peer.acknowledge_dsp(1) && !peer.dsp_interrupt(true), "Accept only the observed edge");
  require(peer.mcu_strobe(true) && peer.dsp_interrupt(true) == 2 && peer.acknowledge_dsp(1) &&
              peer.dsp_interrupt(true) == 2,
          "An older acknowledgement cannot erase a newer event");
  require(!peer.acknowledge_dsp(3) && peer.dsp_interrupt(true) == 2,
          "A future acknowledgement is rejected");
  require(peer.acknowledge_dsp(2) && !peer.dsp_interrupt(true) && peer.dsp_interrupt(false) == 2,
          "An accepted edge does not release the actual level");
  require(peer.dsp_ready(true) && peer.mcu_interrupt() == 1 && peer.dsp_ready(false) &&
              !peer.mcu_interrupt(),
          "Level-sensitive XIRQ does not retain an ended pulse behind the MCU mask");
  peer.reset_dsp();
  require(!peer.mcu_interrupt() && !peer.dsp_interrupt(true) && peer.dsp_interrupt(false) == 2,
          "Reset cancels stale edge requests without changing the MCU's input level");
  require(peer.dsp_ready(true) && peer.mcu_interrupt() == 2 && peer.acknowledge_mcu(1) &&
              peer.mcu_interrupt() == 2 && peer.acknowledge_mcu(2) && peer.mcu_interrupt() == 2 &&
              peer.dsp_ready(false) && !peer.mcu_interrupt(),
          "Native interrupt acceptance cannot clear an asserted XIRQ source; release clears it");
}

void concurrent_exchange() {
  PeerPort peer;
  const auto payload = [](const std::uint8_t value) {
    return PeerPort::Sample{.mask = static_cast<std::uint8_t>(value ^ drive_tag),
                            .known = static_cast<std::uint8_t>(value ^ knowledge_tag),
                            .value = value};
  };
  peer.mcu_data(payload(0));
  std::atomic<bool> done{false};
  constexpr std::uint64_t pulses = 4096;
  bool posted = true;
  std::jthread producer([&] {
    for (std::uint64_t count = 0; count < pulses; ++count) {
      peer.mcu_data(payload(static_cast<std::uint8_t>(count & byte_mask)));
      posted = posted && peer.mcu_strobe(true) && peer.mcu_strobe(false);
    }
    done.store(true, std::memory_order_release);
  });
  bool coherent = true;
  bool accepted = true;
  std::uint64_t last = 0;
  while (!done.load(std::memory_order_acquire)) {
    const auto sample = peer.mcu_data();
    coherent = coherent && sample == payload(sample.value);
    if (const auto request = peer.dsp_interrupt(true)) {
      accepted = accepted && peer.acknowledge_dsp(*request);
      last = *request;
    }
  }
  producer.join();
  if (const auto request = peer.dsp_interrupt(true)) {
    accepted = accepted && peer.acknowledge_dsp(*request);
    last = *request;
  }
  require(posted && coherent && accepted && last == pulses && !peer.dsp_interrupt(true),
          "Concurrent snapshots are coherent and final posted/accepted generations agree");
}

void begin_reply(PeerPort &peer) {
  constexpr PeerPort::Sample report{.mask = 0xff, .known = 0xff, .value = 0x0d};
  peer.dsp_data(report);
  require(peer.dsp_ready(true) && peer.dsp_ready(false) && peer.mcu_strobe(true) &&
              peer.mcu_strobe(false),
          "A real report announcement and MCU request establish bounded response ownership");
}

PeerPort::Sample reply_byte(const unsigned index) {
  return {.mask = static_cast<std::uint8_t>(byte_mask),
          .known = static_cast<std::uint8_t>(byte_mask),
          .value = static_cast<std::uint8_t>(index / 2)};
}

void fixed_reply() {
  PeerPort ordinary;
  constexpr PeerPort::Sample raw{.mask = 0xff, .known = 0xff, .value = 0x0d};
  ordinary.dsp_data(raw);
  require(
      ordinary.mcu_strobe(true) && ordinary.mcu_strobe(false) &&
          ordinary.read_dsp_data(byte_mask) == raw && ordinary.read_dsp_data(byte_mask) == raw,
      "A command-looking latch without an actual announcement remains an ordinary retained read");
  PeerPort peer;
  begin_reply(peer);
  constexpr unsigned bytes = 16;
  for (unsigned index = 0; index < bytes; ++index) {
    const auto value = reply_byte(index);
    peer.dsp_data(value);
    require(peer.dsp_write(value), "Capture every actual data write, including equal neighbors");
  }
  peer.dsp_data({});
  require(peer.read_dsp_data(1) == PeerPort::Sample{} && peer.dsp_data() == PeerPort::Sample{},
          "Masked and raw GPIO reads retain the real released input and consume no response");
  for (unsigned index = 0; index < bytes; ++index) {
    require(peer.read_dsp_data(byte_mask) == reply_byte(index),
            "A complete MCU block read receives the exact original DSP write sequence");
  }
  require(peer.read_dsp_data(byte_mask) == PeerPort::Sample{} && peer.error() == nullptr &&
              peer.dsp_interrupt(true) == 1 && !peer.mcu_interrupt(),
          "The block ends at its fixed length without manufacturing data or interrupt acceptance");
}

void reply_errors() {
  PeerPort early;
  begin_reply(early);
  require(!early.read_dsp_data(byte_mask) && early.error() != nullptr,
          "A missing producer byte is an explicit underrun, not a repeated or invented sample");
  early.reset_dsp();
  require(early.error() == nullptr && early.read_dsp_data(byte_mask) == PeerPort::Sample{},
          "Reset cancels an incomplete response before a new epoch");
  PeerPort excess;
  begin_reply(excess);
  constexpr unsigned bytes = 16;
  for (unsigned index = 0; index < bytes; ++index) {
    require(excess.dsp_write(reply_byte(index)), "Fill precisely one bounded response");
  }
  require(!excess.dsp_write(reply_byte(bytes)) && excess.error() != nullptr,
          "A seventeenth data write is a visible overflow");
  PeerPort unknown;
  begin_reply(unknown);
  constexpr PeerPort::Sample partial{.mask = 0xff, .known = 0x0f, .value = 0xa5};
  require(unknown.dsp_write(partial) && unknown.read_dsp_data(byte_mask) == partial,
          "Response ownership cannot turn unknown producer bits into valid data");
  unknown.dsp_data({});
  require(unknown.error() != nullptr, "Premature producer release reports an incomplete response");
}

void concurrent_reply() {
  PeerPort peer;
  begin_reply(peer);
  std::atomic<unsigned> published{0};
  bool valid = true;
  constexpr unsigned bytes = 16;
  std::jthread producer([&] {
    for (unsigned index = 0; index < bytes; ++index) {
      const auto value = reply_byte(index);
      peer.dsp_data(value);
      valid = peer.dsp_write(value) && valid;
      published.store(index + 1, std::memory_order_release);
      published.notify_one();
    }
    peer.dsp_data({});
  });
  for (unsigned index = 0; index < bytes; ++index) {
    auto ready = published.load(std::memory_order_acquire);
    while (ready <= index) {
      published.wait(ready, std::memory_order_acquire);
      ready = published.load(std::memory_order_acquire);
    }
    require(peer.read_dsp_data(byte_mask) == reply_byte(index),
            "Concurrent response consumption preserves every produced word and its ownership");
  }
  producer.join();
  require(valid && peer.error() == nullptr,
          "Independent producer and consumer finish one bounded block");
}
}

int main() try {
  data_views();
  requests();
  concurrent_exchange();
  fixed_reply();
  reply_errors();
  concurrent_reply();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
