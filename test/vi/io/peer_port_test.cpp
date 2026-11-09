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
              peer.mcu_interrupt() == 1,
          "A completed DSP-ready pulse remains available to its MCU owner");
  peer.reset_dsp();
  require(!peer.mcu_interrupt() && !peer.dsp_interrupt(true) && peer.dsp_interrupt(false) == 2,
          "Reset cancels stale edge requests without changing the MCU's input level");
  require(peer.dsp_ready(true) && peer.mcu_interrupt() == 2 && peer.acknowledge_mcu(1) &&
              peer.mcu_interrupt() == 2 && peer.acknowledge_mcu(2) && !peer.mcu_interrupt(),
          "The next DSP epoch retains monotonic, individually accepted notifications");
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
}

int main() try {
  data_views();
  requests();
  concurrent_exchange();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
