// SPDX-License-Identifier: GPL-3.0-only
#include "io/peer_port.hpp"

#include <array>
#include <atomic>
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
  require(peer.mcu_data(mcu, 0), "Publish the actual MCU data and direction");
  peer.dsp_data(dsp);
  require(peer.mcu_data() == mcu && peer.mcu_data() == mcu && peer.dsp_data() == dsp,
          "Each endpoint retains its genuine drive mask, unknown bits and output latch");
  peer.reset_dsp();
  require(peer.mcu_data() == mcu && peer.dsp_data() == PeerPort::Sample{},
          "DSP reset releases only its output; the MCU's actual driver is retained");
}

void timed_data() {
  PeerPort peer;
  constexpr std::uint64_t first_tick = 10;
  constexpr std::uint64_t next_tick = 20;
  constexpr std::uint64_t release_tick = 30;
  constexpr std::uint64_t same_tick = 40;
  constexpr PeerPort::Sample first{.mask = 0xff, .known = 0xff, .value = 0x52};
  constexpr PeerPort::Sample unknown{.mask = 0xff, .known = 0x7f, .value = 0xa5};
  constexpr PeerPort::Sample released{.mask = 0, .known = 0xff, .value = 0x33};
  require(peer.mcu_data(first, first_tick) && peer.mcu_data(unknown, next_tick) &&
              peer.mcu_data(released, release_tick),
          "The MCU publishes actual successive data and direction changes without a DSP wait");
  require(peer.mcu_data() == released && peer.sample_mcu_data(first_tick - 1) == PeerPort::Sample{},
          "The producer's latest latch cannot leak into an earlier DSP input sample");
  require(peer.sample_mcu_data(first_tick) == first && peer.sample_mcu_data(next_tick - 1) == first,
          "An exact-time input change is visible and repeated earlier reads retain its value");
  peer.reset_dsp();
  require(peer.sample_mcu_data(next_tick - 1) == first &&
              peer.sample_mcu_data(next_tick) == unknown &&
              peer.sample_mcu_data(release_tick) == released,
          "DSP reset retains the MCU-owned timeline, unknown bits and later direction release");
  require(peer.mcu_data(first, same_tick) && peer.mcu_data(unknown, same_tick) &&
              peer.sample_mcu_data(same_tick) == unknown &&
              peer.sample_mcu_data(same_tick) == unknown && !peer.dsp_interrupt(true),
          "Same-time writes commit in order; data samples neither pop payload nor fabricate IRQs");
  require(!peer.mcu_data(first, same_tick - 1) && peer.error() != nullptr &&
              peer.mcu_data() == unknown,
          "Backwards producer time fails without publishing a replacement sample");
}

void concurrent_timed_data() {
  PeerPort peer;
  constexpr std::uint64_t count = 10000;
  const auto payload = [](const std::uint64_t index) {
    const auto value = static_cast<std::uint8_t>(index & byte_mask);
    return PeerPort::Sample{.mask = value,
                            .known = static_cast<std::uint8_t>(value ^ knowledge_tag),
                            .value = static_cast<std::uint8_t>(value ^ drive_tag)};
  };
  std::atomic<std::uint64_t> committed{0};
  bool published = true;
  std::jthread producer([&](const std::stop_token &stop) {
    for (std::uint64_t index = 1; index <= count; ++index) {
      if (!peer.mcu_data(payload(index), index, stop)) {
        published = false;
        return;
      }
      committed.store(index, std::memory_order_release);
    }
  });
  bool coherent = true;
  std::uint64_t observed = 0;
  while (observed != count) {
    observed = committed.load(std::memory_order_acquire);
    if (observed == 0) { continue; }
    coherent = peer.sample_mcu_data(observed) == payload(observed) && coherent;
  }
  producer.join();
  require(published && coherent && peer.sample_mcu_data(count) == payload(count) &&
              peer.error() == nullptr,
          "Wrapping producer/consumer ownership preserves every sampled frontier without races");
}

void cancelled_data_producer() {
  PeerPort peer;
  constexpr std::uint64_t count = 10000;
  std::atomic<std::uint64_t> committed{0};
  std::promise<bool> outcome;
  auto completion = outcome.get_future();
  std::jthread producer([&](const std::stop_token &stop) {
    for (std::uint64_t index = 1; index <= count; ++index) {
      const PeerPort::Sample data{
          .mask = 0xff, .known = 0xff, .value = static_cast<std::uint8_t>(index & byte_mask)};
      if (!peer.mcu_data(data, index, stop)) {
        outcome.set_value(false);
        return;
      }
      committed.store(index, std::memory_order_release);
    }
    outcome.set_value(true);
  });
  const auto ready = completion.wait_for(std::chrono::milliseconds(20));
  producer.request_stop();
  producer.join();
  require(ready == std::future_status::timeout && !completion.get() &&
              committed.load(std::memory_order_acquire) != 0 && peer.error() == nullptr,
          "A full transport applies bounded backpressure and stop wakes its producer");
  const auto last = committed.load(std::memory_order_acquire);
  require(peer.sample_mcu_data(count).value == static_cast<std::uint8_t>(last & byte_mask),
          "Cancellation leaves precisely the committed input prefix available to the DSP");
}

void reset_data_retirement(const bool retire_before_release) {
  PeerPort peer;
  constexpr std::uint64_t updates = 10000;
  constexpr std::uint8_t driven = 0x7f;
  constexpr std::uint8_t known = 0x3f;
  constexpr PeerPort::Sample before{.mask = 0xff, .known = 0xff, .value = 0x52};
  require(peer.mcu_data(before, 0), "An old unconsumed input precedes reset assertion");
  peer.hold_mcu_data();
  if (retire_before_release) { peer.retire_mcu_data(); }
  std::promise<bool> outcome;
  auto completion = outcome.get_future();
  std::jthread producer([&](const std::stop_token &stop) {
    for (std::uint64_t index = 1; index <= updates; ++index) {
      const PeerPort::Sample data{
          .mask = driven, .known = known, .value = static_cast<std::uint8_t>(index & byte_mask)};
      if (!peer.mcu_data(data, index, stop)) {
        outcome.set_value(false);
        return;
      }
    }
    outcome.set_value(peer.release_mcu_data(updates + 1, stop));
  });
  const auto ready = completion.wait_for(std::chrono::seconds(1));
  producer.request_stop();
  producer.join();
  require(ready == std::future_status::ready && completion.get(),
          "Changes while the DSP is held coalesce without filling a transport it cannot drain");
  if (!retire_before_release) { peer.retire_mcu_data(); }
  require(peer.sample_mcu_data(updates) == PeerPort::Sample{} &&
              peer.sample_mcu_data(updates + 1) ==
                  PeerPort::Sample{.mask = driven,
                                   .known = known,
                                   .value = static_cast<std::uint8_t>(updates & byte_mask)},
          "Old-prefix retirement cannot discard a newer release value or publish it early");
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

void eligible_notifications() {
  PeerPort peer;
  require(peer.mcu_interrupt_ready(true), "Publish the MCU's actual native interrupt eligibility");
  require(peer.dsp_ready(true) && peer.dsp_ready(false) && peer.mcu_interrupt() == 1,
          "An eligible producer notification survives a complete host execution batch");
  require(peer.acknowledge_mcu(1) && !peer.mcu_interrupt(),
          "Only actual interrupt acceptance retires the observed eligible generation");
  require(peer.dsp_ready(true) && peer.dsp_ready(false) && peer.dsp_ready(true) &&
              peer.dsp_ready(false) && peer.acknowledge_mcu(2) && peer.mcu_interrupt() == 3,
          "Acknowledging an older notification cannot lose a newer producer event");
  require(peer.mcu_interrupt_ready(false) && !peer.mcu_interrupt() &&
              peer.mcu_interrupt_ready(true) && !peer.mcu_interrupt(),
          "A mask transition revokes transport notifications without reviving them on unmask");
  require(peer.mcu_interrupt_ready(false) && peer.dsp_ready(true) && peer.dsp_ready(false) &&
              !peer.mcu_interrupt() && peer.mcu_interrupt_ready(true) && !peer.mcu_interrupt(),
          "A pulse wholly inside a masked window is never queued for later execution");
  require(peer.dsp_ready(true) && peer.dsp_ready(false) && peer.mcu_interrupt() == 5,
          "A fresh eligible window receives only a new actual producer event");
  peer.reset_dsp();
  require(!peer.mcu_interrupt() && peer.dsp_ready(true) && peer.dsp_ready(false) &&
              peer.mcu_interrupt() == 6,
          "DSP reset revokes old events while preserving the running MCU's admission window");
}

void concurrent_admission() {
  PeerPort peer;
  std::atomic<bool> done{false};
  constexpr unsigned pulses = 10000;
  bool posted = true;
  std::jthread producer([&] {
    for (unsigned index = 0; index < pulses; ++index) {
      posted = peer.dsp_ready(true) && peer.dsp_ready(false) && posted;
    }
    done.store(true, std::memory_order_release);
  });
  bool accepted = true;
  do {
    accepted = peer.mcu_interrupt_ready(true) && accepted;
    if (const auto request = peer.mcu_interrupt()) {
      accepted = peer.acknowledge_mcu(*request) && accepted;
    }
    accepted = peer.mcu_interrupt_ready(false) && accepted;
  } while (!done.load(std::memory_order_acquire));
  producer.join();
  require(posted && accepted && !peer.mcu_interrupt(),
          "Concurrent eligibility changes leave no ended notification behind the mask");
  require(peer.mcu_interrupt_ready(true) && !peer.mcu_interrupt(),
          "An old DSP eligibility observation cannot cross a mask/unmask generation");
  require(peer.dsp_ready(true) && peer.dsp_ready(false) && peer.mcu_interrupt() == pulses + 1 &&
              peer.acknowledge_mcu(pulses + 1) && !peer.mcu_interrupt(),
          "The final quiescent unmasked producer event is delivered and retired exactly once");
}

void concurrent_exchange() {
  PeerPort peer;
  const auto payload = [](const std::uint8_t value) {
    return PeerPort::Sample{.mask = static_cast<std::uint8_t>(value ^ drive_tag),
                            .known = static_cast<std::uint8_t>(value ^ knowledge_tag),
                            .value = value};
  };
  require(peer.mcu_data(payload(0), 0), "Publish the initial coherent MCU sample");
  std::atomic<bool> done{false};
  constexpr std::uint64_t pulses = 4096;
  bool posted = true;
  std::jthread producer([&] {
    for (std::uint64_t count = 0; count < pulses; ++count) {
      posted =
          peer.mcu_data(payload(static_cast<std::uint8_t>(count & byte_mask)), count) && posted;
      posted = posted && peer.mcu_strobe(true) && peer.mcu_strobe(false);
    }
    done.store(true, std::memory_order_release);
  });
  bool coherent = true;
  bool accepted = true;
  std::uint64_t last = 0;
  while (!done.load(std::memory_order_acquire)) {
    static_cast<void>(peer.sample_mcu_data(pulses));
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

void report_notifications() {
  constexpr std::array<std::uint8_t, 2> commands{0x0d, 0x0e};
  for (const auto command : commands) {
    PeerPort peer;
    const PeerPort::Sample driven{.mask = 0xff, .known = 0xff, .value = command};
    peer.dsp_data(driven);
    require(peer.dsp_ready(true) && peer.dsp_ready(false) && peer.mcu_interrupt() == 1,
            "An unchanged announced report remains pending after its short readiness pulse");
    require(peer.acknowledge_mcu(1) && peer.mcu_interrupt() == 1,
            "Interrupt admission cannot consume an unaccepted report command");
    peer.dsp_data(driven);
    require(peer.mcu_interrupt() == 1 && peer.read_dsp_data(byte_mask) == driven,
            "Unchanged GPIO publication retains notification without inventing report bytes");
    require(peer.mcu_strobe(true) && peer.mcu_strobe(false) && !peer.mcu_interrupt() &&
                peer.dsp_interrupt(true) == 1,
            "The actual MCU strobe accepts the report and posts its native DSP request");
  }
}

void withdrawn_reports() {
  constexpr PeerPort::Sample command{.mask = 0xff, .known = 0xff, .value = 0x0d};
  constexpr std::array<PeerPort::Sample, 4> withdrawals{
      PeerPort::Sample{},
      PeerPort::Sample{.mask = 0x7f, .known = 0xff, .value = 0x0d},
      PeerPort::Sample{.mask = 0xff, .known = 0x7f, .value = 0x0d},
      PeerPort::Sample{.mask = 0xff, .known = 0xff, .value = 0x0e}};
  for (const auto withdrawal : withdrawals) {
    PeerPort peer;
    peer.dsp_data(command);
    require(peer.dsp_ready(true) && peer.dsp_ready(false) && peer.mcu_interrupt() == 1,
            "A genuine report announcement precedes withdrawal");
    peer.dsp_data(withdrawal);
    require(!peer.mcu_interrupt() && peer.dsp_data() == withdrawal,
            "Changed command, direction or knowledge cancels notification without holding data");
    peer.dsp_data(command);
    require(!peer.mcu_interrupt(), "Reusing an old command latch cannot recreate its announcement");
    require(peer.dsp_ready(true) && peer.dsp_ready(false) && peer.mcu_interrupt() == 2,
            "A fresh announcement belongs to a new request generation");
    peer.reset_dsp();
    require(!peer.mcu_interrupt() && peer.dsp_data() == PeerPort::Sample{},
            "Reset cancels an unaccepted report and releases its real data");
  }
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
  timed_data();
  concurrent_timed_data();
  cancelled_data_producer();
  reset_data_retirement(false);
  reset_data_retirement(true);
  requests();
  eligible_notifications();
  concurrent_admission();
  concurrent_exchange();
  report_notifications();
  withdrawn_reports();
  fixed_reply();
  reply_errors();
  concurrent_reply();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
