// SPDX-License-Identifier: GPL-3.0-only
#include "io/midi_port.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
using blofeld::vi::MidiPort;
constexpr std::uint8_t note_on = 0x90, note = 60, velocity = 127;
constexpr std::array message{note_on, note, velocity};

void require(const bool condition, const char *message_text) {
  if (!condition) { throw std::runtime_error(message_text); }
}

void wait_for(auto ready, const char *message_text) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!ready()) {
    require(std::chrono::steady_clock::now() < deadline, message_text);
    std::this_thread::yield();
  }
}

void stream_frontiers() {
  MidiPort live;
  require(!live.input_limit() && live.await_input(std::numeric_limits<std::uint64_t>::max(), {}),
          "Live input retains its unbounded execution contract");
  MidiPort stream(MidiPort::InputMode::stream);
  constexpr std::uint64_t first = 100;
  require(stream.input_limit() == 0 && stream.advance_input(first) &&
              stream.input_limit() == first && stream.await_input(first - 1, {}) &&
              !stream.peek_input() && stream.time() == 0,
          "A known silent interval publishes no bytes and advances no device time");
  require(stream.write_input(message, first) &&
              stream.input_limit() == first + (message.size() * MidiPort::byte_ticks) &&
              stream.await_input(first, {}),
          "Accepted actual bytes extend knowledge through their serialized character intervals");
  require(stream.finish_input() && stream.finish_input() && !stream.input_limit() &&
              stream.await_input(std::numeric_limits<std::uint64_t>::max(), {}) &&
              stream.peek_input() == MidiPort::Event{.ticks = first, .byte = note_on},
          "EOF removes the future-input barrier without discarding queued bytes");
  require(!stream.write_input(message, first) && stream.error() != nullptr,
          "A finished source cannot add more bytes");
  MidiPort backwards(MidiPort::InputMode::stream);
  require(backwards.advance_input(first) && !backwards.advance_input(first - 1) &&
              backwards.input_limit() == first,
          "A committed input frontier cannot retreat");
  MidiPort late(MidiPort::InputMode::stream);
  require(late.advance_input(first) && !late.write_input(message, first - 1) && !late.peek_input(),
          "A late producer cannot insert data inside a committed silent interval");
  MidiPort serialized(MidiPort::InputMode::stream);
  require(serialized.write_input(message, 0) && serialized.write_input(message, 0) &&
              serialized.input_limit() == message.size() * 2 * MidiPort::byte_ticks,
          "Same-time packets retain DIN serialization rather than violating the frontier");
}

void frontier_waits() {
  for (unsigned action = 0; action < 3; ++action) {
    MidiPort stream(MidiPort::InputMode::stream);
    std::atomic<bool> entered{false};
    bool admitted = false;
    std::jthread mcu([&](const std::stop_token &stop) {
      entered.store(true, std::memory_order_release);
      admitted = stream.await_input(0, stop);
    });
    wait_for([&] { return entered.load(std::memory_order_acquire); },
             "Start the blocked MCU owner");
    if (action == 0) {
      require(stream.advance_input(1), "Publish new future-input knowledge");
    } else if (action == 1) {
      require(stream.finish_input(), "Publish actual EOF");
    } else {
      mcu.request_stop();
    }
    mcu.join();
    require(admitted == (action != 2) && stream.error() == nullptr && stream.time() == 0,
            "Knowledge, EOF and cancellation wake the owner without inventing elapsed time");
  }
}

void blocking_admission() {
  constexpr std::array<std::uint8_t, MidiPort::capacity> bytes{};
  for (const bool cancel : {false, true}) {
    MidiPort stream(MidiPort::InputMode::stream);
    require(stream.write_input(bytes, 0), "Fill the bounded source queue");
    std::atomic<bool> entered{false};
    bool admitted = false;
    std::jthread producer([&](const std::stop_token &stop) {
      entered.store(true, std::memory_order_release);
      admitted = stream.write_input(message, 0, stop);
    });
    wait_for([&] { return entered.load(std::memory_order_acquire); },
             "Start admission under backpressure");
    if (cancel) {
      producer.request_stop();
    } else {
      for (unsigned index = 0; index < message.size(); ++index) { stream.pop_input(); }
      require(stream.peek_input(MidiPort::Receiver::dsp) == MidiPort::Event{} &&
                  stream.input_limit() == bytes.size() * MidiPort::byte_ticks,
              "MCU retirement alone cannot grant reuse of the DSP observer's immutable slots");
      for (unsigned index = 0; index < message.size(); ++index) {
        stream.pop_input(MidiPort::Receiver::dsp);
      }
    }
    producer.join();
    require(admitted != cancel && stream.error() == nullptr &&
                stream.input_limit() ==
                    (bytes.size() + (cancel ? 0 : message.size())) * MidiPort::byte_ticks,
            "Backpressure waits for whole-packet credit and cancellation commits nothing");
    const auto retained = bytes.size() - (cancel ? 0 : message.size());
    for (std::size_t index = 0; index < retained; ++index) {
      const auto event = stream.peek_input();
      require(event && event->byte == 0 && stream.peek_input(MidiPort::Receiver::dsp) == event,
              "Admission never overwrites old accepted bytes for either observer");
      stream.pop_input();
      stream.pop_input(MidiPort::Receiver::dsp);
    }
    if (!cancel) {
      for (const auto byte : message) {
        const auto event = stream.peek_input();
        require(event && event->byte == byte && stream.peek_input(MidiPort::Receiver::dsp) == event,
                "A released packet enters once and in order for each observer");
        stream.pop_input();
        stream.pop_input(MidiPort::Receiver::dsp);
      }
    }
    require(!stream.peek_input(), "Cancelled admission contributes no partial data");
  }
}

void frontier_registration() {
  MidiPort stream(MidiPort::InputMode::stream);
  constexpr std::uint64_t rounds = 1024;
  std::atomic<std::uint64_t> acknowledged{0};
  bool coherent = true;
  std::jthread mcu([&](const std::stop_token &stop) {
    for (std::uint64_t tick = 0; tick < rounds; ++tick) {
      if (!stream.await_input(tick, stop)) {
        coherent = false;
        return;
      }
      acknowledged.store(tick + 1, std::memory_order_release);
    }
  });
  for (std::uint64_t tick = 1; tick <= rounds; ++tick) {
    require(stream.advance_input(tick), "Publish a strictly ordered finite frontier");
    wait_for([&] { return acknowledged.load(std::memory_order_acquire) == tick; },
             "Publication and wait registration cannot lose a wakeup");
  }
  mcu.join();
  require(coherent && stream.input_limit() == rounds && !stream.peek_input() && stream.time() == 0,
          "Repeated handoff transfers knowledge without synthetic traffic or device-time steps");
}

void echo_input(MidiPort &port,
                const std::span<const std::uint8_t> packet,
                const std::size_t count,
                std::atomic<std::size_t> &acknowledged,
                bool &coherent,
                const std::stop_token &stop) {
  for (std::size_t index = 0; index < count; ++index) {
    if (!port.await_input(index * MidiPort::byte_ticks, stop)) {
      coherent = false;
      return;
    }
    const auto event = port.peek_input();
    if (!event || event->ticks != index * MidiPort::byte_ticks ||
        event->byte != packet[index % packet.size()]) {
      coherent = false;
      port.abort_input();
      return;
    }
    port.pop_input();
    if (!port.write_output(*event)) {
      coherent = false;
      return;
    }
    while (acknowledged.load(std::memory_order_acquire) != index + 1) {
      if (stop.stop_requested()) { return; }
      std::this_thread::yield();
    }
  }
}

bool observe_input(MidiPort &port,
                   const std::span<const std::uint8_t> packet,
                   const std::size_t count,
                   const std::stop_token &stop) {
  for (std::size_t index = 0; index < count; ++index) {
    std::optional<MidiPort::Event> event;
    while (!(event = port.peek_input(MidiPort::Receiver::dsp))) {
      if (stop.stop_requested() || port.error() != nullptr) { return false; }
      std::this_thread::yield();
    }
    if (event->ticks != index * MidiPort::byte_ticks ||
        event->byte != packet[index % packet.size()]) {
      port.abort_input();
      return false;
    }
    port.pop_input(MidiPort::Receiver::dsp);
  }
  return true;
}

void separate_output_owner() {
  MidiPort port(MidiPort::InputMode::stream);
  constexpr std::size_t packets = 64;
  constexpr std::size_t packet_bytes = 64;
  constexpr std::size_t count = packets * packet_bytes;
  const auto packet = [] {
    std::array<std::uint8_t, packet_bytes> bytes{};
    for (std::size_t index = 0; index < bytes.size(); ++index) {
      bytes[index] = static_cast<std::uint8_t>(index);
    }
    return bytes;
  }();
  std::atomic<std::size_t> acknowledged{0};
  bool sent = true;
  bool coherent = true;
  bool dsp_coherent = false;
  std::jthread dsp([&](const std::stop_token &stop) {
    dsp_coherent = observe_input(port, packet, count, stop);
  });
  std::jthread producer([&](const std::stop_token &stop) {
    for (std::size_t index = 0; index < packets; ++index) {
      if (!port.write_input(packet, 0, stop)) {
        sent = false;
        return;
      }
    }
    sent = port.finish_input();
  });
  std::jthread mcu([&](const std::stop_token &stop) {
    echo_input(port, packet, count, acknowledged, coherent, stop);
  });
  for (std::size_t index = 0; index < count; ++index) {
    require(port.await_output(mcu.get_stop_token()),
            "An independent output owner waits for actual replies");
    const auto event = port.read_output();
    require(event && event->ticks == index * MidiPort::byte_ticks &&
                event->byte == packet[index % packet.size()],
            "Replies drain while the independent input producer is backpressured");
    acknowledged.store(index + 1, std::memory_order_release);
  }
  mcu.join();
  producer.join();
  dsp.join();
  require(sent && coherent && dsp_coherent && !port.peek_input() &&
              !port.peek_input(MidiPort::Receiver::dsp) && !port.read_output() &&
              port.error() == nullptr,
          "All four owners retain exact ordering across source and reply queue reuse");
  std::atomic<bool> entered{false};
  bool replied = true;
  std::jthread waiting([&](const std::stop_token &stop) {
    entered.store(true, std::memory_order_release);
    replied = port.await_output(stop);
  });
  wait_for([&] { return entered.load(std::memory_order_acquire); }, "Start an idle output owner");
  waiting.request_stop();
  waiting.join();
  require(!replied && port.error() == nullptr, "Cancelling output wait creates no reply or fault");
}

void ordered_packets() {
  MidiPort port;
  constexpr std::uint64_t first = 100;
  require(!port.peek_input() && !port.read_output() && port.time() == 0 &&
              port.write_input({}, first) && port.write_input(message, first) &&
              port.write_input(message, first),
          "Empty ports manufacture no bytes and packets can share a musical timestamp");
  for (unsigned index = 0; index < message.size() * 2; ++index) {
    const MidiPort::Event wanted{.ticks = first + (index * MidiPort::byte_ticks),
                                 .byte = message[index % message.size()]};
    require(port.peek_input() == wanted && port.peek_input() == wanted,
            "Packet bytes retain order and DIN spacing; lookahead does not consume them");
    port.pop_input();
    require(port.peek_input(MidiPort::Receiver::dsp) == wanted,
            "MCU consumption cannot erase the DSP's observation of the same input");
    port.pop_input(MidiPort::Receiver::dsp);
    require(port.write_output(wanted) && port.read_output() == wanted && !port.read_output(),
            "Actual output is transferred exactly once with its completion timestamp");
  }
  require(!port.peek_input() && port.error() == nullptr, "Packets end without invented padding");
  port.publish_time(first);
  require(port.time() == first, "The host sees published device time rather than host time");
}

void wrap_and_overload() {
  MidiPort port;
  std::array<std::uint8_t, MidiPort::capacity> bytes{};
  constexpr auto byte_mask = std::numeric_limits<std::uint8_t>::max();
  for (unsigned index = 0; index < bytes.size(); ++index) {
    bytes[index] = static_cast<std::uint8_t>(index & byte_mask);
  }
  require(port.write_input(bytes, 0), "Fill the entire bounded input");
  for (unsigned index = 0; index < bytes.size() / 2; ++index) {
    port.pop_input();
    port.pop_input(MidiPort::Receiver::dsp);
  }
  require(port.write_input(std::span(bytes).first(bytes.size() / 2), 0),
          "Released slots can be reused across the ring boundary");
  for (unsigned index = 0; index < bytes.size(); ++index) {
    const auto sequence = index + (bytes.size() / 2);
    require(port.peek_input() ==
                MidiPort::Event{.ticks = sequence * MidiPort::byte_ticks,
                                .byte = static_cast<std::uint8_t>(sequence & byte_mask)},
            "Wrapped input retains exact data and timestamps");
    port.pop_input();
    port.pop_input(MidiPort::Receiver::dsp);
  }
  require(!port.peek_input(), "All wrapped input was consumed");
  MidiPort full;
  require(full.write_input(bytes, 0) && !full.write_input(message, 0) && full.error() != nullptr &&
              full.peek_input() == MidiPort::Event{},
          "Input overflow is visible and cannot overwrite previously accepted data");
  MidiPort slow;
  require(slow.write_input(bytes, 0), "Fill both observers' shared immutable input");
  for (unsigned index = 0; index < bytes.size(); ++index) { slow.pop_input(); }
  require(!slow.write_input(message, 0) &&
              slow.peek_input(MidiPort::Receiver::dsp) == MidiPort::Event{},
          "A slow DSP observer prevents slot reuse even after the MCU has consumed every byte");
  MidiPort output;
  for (unsigned index = 0; index < MidiPort::capacity; ++index) {
    require(output.write_output({.ticks = index, .byte = note_on}), "Fill bounded output");
  }
  require(!output.write_output({.ticks = MidiPort::capacity, .byte = note}) &&
              output.error() != nullptr &&
              output.read_output() == MidiPort::Event{.ticks = 0, .byte = note_on},
          "Output overflow preserves the oldest real firmware byte");
}

void invalid_times() {
  MidiPort late;
  constexpr std::uint64_t published = 100;
  late.publish_time(published);
  require(!late.write_input(message, 0) && !late.peek_input() &&
              !late.peek_input(MidiPort::Receiver::dsp) && late.error() != nullptr,
          "New input cannot be admitted behind either executing owner's published frontier");
  MidiPort backwards;
  require(backwards.write_input(message, 1) && !backwards.write_input(message, 0) &&
              backwards.error() != nullptr,
          "Out-of-order host timestamps fail explicitly");
  MidiPort overflow;
  require(!overflow.write_input(message, std::numeric_limits<std::uint64_t>::max()) &&
              !overflow.peek_input() && overflow.error() != nullptr,
          "An unrepresentable character interval never partially enters the queue");
  MidiPort output;
  require(output.write_output({.ticks = 1, .byte = note_on}) &&
              !output.write_output({.ticks = 0, .byte = note}) && output.error() != nullptr,
          "Firmware output cannot reverse the common device timeline");
}

bool observe_dsp(MidiPort &port,
                 const unsigned bytes,
                 std::atomic<unsigned> &observed,
                 const std::stop_token &stop) {
  bool coherent = true;
  for (unsigned index = 0; index < bytes; ++index) {
    std::optional<MidiPort::Event> event;
    while (!(event = port.peek_input(MidiPort::Receiver::dsp))) {
      if (stop.stop_requested()) { return false; }
      std::this_thread::yield();
    }
    coherent = coherent && event == MidiPort::Event{.ticks = index * MidiPort::byte_ticks,
                                                    .byte = message[index % message.size()]};
    port.pop_input(MidiPort::Receiver::dsp);
    observed.store(index + 1, std::memory_order_release);
  }
  return coherent;
}

void concurrent_exchange() {
  MidiPort port;
  constexpr unsigned rounds = 1024;
  constexpr unsigned bytes = rounds * message.size();
  bool coherent = true;
  bool dsp_coherent = true;
  std::atomic<unsigned> observed{0};
  std::jthread dsp([&](const std::stop_token &stop) {
    dsp_coherent = observe_dsp(port, bytes, observed, stop);
  });
  std::jthread mcu([&](const std::stop_token &stop) {
    for (unsigned index = 0; index < bytes; ++index) {
      while (!port.peek_input()) {
        if (stop.stop_requested()) { return; }
        std::this_thread::yield();
      }
      const auto event = *port.peek_input();
      coherent = coherent && event == MidiPort::Event{.ticks = index * MidiPort::byte_ticks,
                                                      .byte = message[index % message.size()]};
      port.pop_input();
      const bool posted = port.write_output(event);
      coherent = coherent && posted;
      if (!posted) { return; }
      port.publish_time(event.ticks);
    }
  });
  for (unsigned round = 0; round < rounds; ++round) {
    require(port.write_input(message, 0), "One host owner submits ordered packets");
    for (unsigned index = 0; index < message.size(); ++index) {
      std::optional<MidiPort::Event> event;
      while (!(event = port.read_output())) {
        require(port.error() == nullptr, "Concurrent output cannot silently stop on overload");
        std::this_thread::yield();
      }
      require(event == MidiPort::Event{.ticks = ((round * message.size()) + index) *
                                                MidiPort::byte_ticks,
                                       .byte = message[index]},
              "The host observes coherent output slots through concurrent ring reuse");
    }
    while (observed.load(std::memory_order_acquire) < (round + 1) * message.size()) {
      std::this_thread::yield();
    }
  }
  mcu.join();
  dsp.join();
  require(coherent && dsp_coherent && !port.peek_input() &&
              !port.peek_input(MidiPort::Receiver::dsp) && !port.read_output() &&
              port.error() == nullptr && port.time() == (bytes - 1) * MidiPort::byte_ticks,
          "Both owners finish with exact data, device time and empty queues");
}
}

int main() try {
  ordered_packets();
  wrap_and_overload();
  invalid_times();
  concurrent_exchange();
  stream_frontiers();
  frontier_waits();
  blocking_admission();
  frontier_registration();
  separate_output_owner();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
