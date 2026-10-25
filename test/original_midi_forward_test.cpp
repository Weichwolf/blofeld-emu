// SPDX-License-Identifier: GPL-3.0-only
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "original_midi_forward_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <utility>

namespace blofeld::test::midi_forward {
using namespace usb_host;

namespace {
struct Message {
  Event event;
  unsigned length;
};

constexpr std::array<Message, 7> messages{{
    {.event = {8, 0x80, 60, 100}, .length = 3},
    {.event = {9, 0x91, 61, 99}, .length = 3},
    {.event = {10, 0xa2, 62, 98}, .length = 3},
    {.event = {11, 0xb3, 7, 97}, .length = 3},
    {.event = {12, 0xc4, 5, canary}, .length = 2},
    {.event = {13, 0xd5, 96, canary}, .length = 2},
    {.event = {14, 0xe6, 1, 95}, .length = 3},
}};
constexpr std::array<unsigned, 3> cursors{0, 62, 63};
constexpr std::array<std::uint8_t, 6> discarded{0, 0x7f, 0xf0, 0xf7, 0xf8, 0xff};
constexpr unsigned mask_bits = 0x50;
constexpr unsigned masked = 0x50;
constexpr unsigned idle = 0x1000;
constexpr unsigned tick_address = 0x182e;
constexpr unsigned boundary_sentinel = canary;
constexpr unsigned expected_cases = 178;
constexpr unsigned uart_enabled = 0xac;

struct Expected {
  unsigned consumed = 0;
  Bytes tx;
  bool failure = false;
  bool wait_ack = false;
};

Bytes bytes(const Message &message) {
  return {message.event.begin() + 1, message.event.begin() + 1 + message.length};
}

void check_peer(Fixture &fixture, const Input &input, const Expected &expected) {
  if (!input.peer) { return; }
  auto &bus = fixture.board().bus();
  require(bus.read(peer_reader) == ((peer_cursor + input.bytes.size()) & ring_mask) &&
              bus.read(tx_writer) == ((input.cursor + expected.tx.size()) & ring_mask) &&
              bus.read(tx_reader) == ((input.cursor + (expected.tx.empty() ? 0 : 1)) & ring_mask) &&
              fixture.counters().uart_calls == input.bytes.size() &&
              fixture.counters().sci_irqs == static_cast<unsigned>(!expected.tx.empty()),
          "Real UART queue calls drain peer bytes, skip FF and preserve the separate SCI reader");
  std::array<std::uint8_t, 64> expected_ring{};
  expected_ring.fill(canary);
  for (unsigned index = 0; index < expected.tx.size(); ++index) {
    expected_ring[(input.cursor + index) & ring_mask] = expected.tx[index];
  }
  for (unsigned index = 0; index < expected_ring.size(); ++index) {
    require(bus.read(static_cast<std::uint16_t>(tx_ring + index)) == expected_ring[index],
            "Every UART TX ring byte matches independent peer-forwarding expectations");
  }
  Bytes transmitted;
  unsigned enables = 0;
  for (const auto &[address, value] : fixture.counters().writes) {
    if (address == hc12::sci_data_low) { transmitted.push_back(static_cast<std::uint8_t>(value)); }
    if (address == hc12::sci_control2) {
      require(value == uart_enabled, "Actual UART queue writes retain the whole SCI enable preset");
      ++enables;
    }
  }
  const Bytes first = expected.tx.empty() ? Bytes{} : Bytes{expected.tx.front()};
  require(
      transmitted == first && enables == expected.tx.size(),
      "The actual SCI ISR writes the independent first byte; FF never enables another queue slot");
}

void complete(Fixture &fixture, const Input &input, const Expected &expected) {
  if (expected.wait_ack) {
    require(fixture.board().bus().read(tick_address) == boundary_sentinel,
            "Full/busy waiting does not invent elapsed timer ticks");
    fixture.acknowledge();
    fixture.service_usb();
  }
  fixture.run();
  fixture.check_return(expected.consumed == 0 ? input.cursor
                                              : (input.cursor + expected.consumed) & ring_mask);
  check_peer(fixture, input, expected);
  auto &bus = fixture.board().bus();
  const auto next = (input.usb_cursor + 4) & usb_mask;
  if (input.event) {
    for (unsigned index = 0; index < input.event->size(); ++index) {
      require(bus.read(static_cast<std::uint16_t>(usb_ring + input.usb_cursor + index)) ==
                  (*input.event)[index],
              "The genuine queue stores the complete original caller packet even on failure");
    }
  }
  if (!input.event || expected.failure) {
    const auto read = (input.usb_cursor + (input.full ? 4 : 0)) & usb_mask;
    require(bus.read(usb_reader) == read && bus.read(usb_writer) == input.usb_cursor &&
                word(bus, input_control) == idle &&
                bus.request_usb_in(UsbBuffer::endpoint2).response == UsbResponse::nak,
            "Incomplete/discarded messages or actual queue failure create no USB completion");
  } else {
    require(bus.read(usb_writer) == next, "The actual USB queue publishes one four-byte event");
    if (input.busy && !expected.wait_ack) {
      fixture.acknowledge();
      fixture.service_usb();
    }
    auto stream = fixture.old_events();
    stream.insert(stream.end(), input.event->begin(), input.event->end());
    fixture.drain(stream);
    require(bus.read(usb_reader) == next,
            "The native sender and IRQs consume every expected event");
  }
  const auto expected_masks = input.event && !expected.failure ? 0 : input.ccr & mask_bits;
  unsigned expected_usb = 0;
  if (input.event && !expected.failure) {
    expected_usb = input.busy || input.full ? 2 : 1;
    if (expected.wait_ack) { expected_usb = 3; }
  }
  require((fixture.board().cpu().state().ccr & mask_bits) == expected_masks &&
              fixture.counters().usb_irqs == expected_usb &&
              fixture.counters().publications == static_cast<unsigned>(expected.consumed != 0),
          "Actual IRQ masks, host ACK counts and one-message mirror publication match");
  fixture.guard();
}

unsigned
execute(const Firmware &firmware, const unsigned id, const Input &input, const Expected &expected) {
  Fixture fixture(firmware, input);
  if (input.event) { fixture.until(queue_entry); }
  if (expected.wait_ack) {
    fixture.until(wait_pc);
    for (unsigned iteration = 0; iteration < 3; ++iteration) { fixture.until(wait_pc, true); }
  }
  const auto saved = fixture.save();
  std::cout << "PREFIX " << id << ' ' << saved.stats.instructions << ' ' << saved.stats.usb_irqs
            << ' ' << saved.stats.sci_irqs << '\n';
  std::optional<Outcome> first;
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    if (attempt != 0) { fixture.restore(saved); }
    complete(fixture, input, expected);
    const auto actual = fixture.outcome();
    if (!first) { first = actual; }
    require(actual == *first,
            "Two joint restores retain CPU, all RAM, native USB/SCI, DATA PID and MMIO history");
    const auto &stats = fixture.counters();
    std::cout << "CASE " << id << ' ' << attempt << ' ' << stats.instructions << ' '
              << stats.usb_irqs << ' ' << stats.sci_irqs << ' ' << stats.usb_calls << ' '
              << stats.uart_calls << ' ' << stats.publications << '\n';
  }
  return id + 1;
}

unsigned ordinary(const Firmware &firmware) {
  unsigned id = 0;
  for (const auto &message : messages) {
    for (const auto cursor : cursors) {
      for (unsigned available = 0; available <= message.length; ++available) {
        const auto source = bytes(message);
        const bool complete_message = available == message.length;
        id = execute(firmware,
                     id,
                     {.bytes = {source.begin(), source.begin() + available},
                      .cursor = cursor,
                      .event = complete_message ? std::make_optional(message.event) : std::nullopt},
                     {.consumed = complete_message ? message.length : 0, .tx = {}});
      }
    }
  }
  for (const auto value : discarded) {
    for (const auto cursor : cursors) {
      id = execute(firmware, id, {.bytes = {value}, .cursor = cursor}, {.consumed = 1, .tx = {}});
    }
  }
  for (const auto cursor : cursors) {
    auto source = bytes(messages[0]);
    const auto second = bytes(messages[1]);
    source.insert(source.end(), second.begin(), second.end());
    id = execute(firmware,
                 id,
                 {.bytes = std::move(source), .cursor = cursor, .event = messages[0].event},
                 {.consumed = messages[0].length, .tx = {}});
  }
  return id;
}

unsigned backpressure(const Firmware &firmware, unsigned id) {
  for (const auto &message : messages) {
    for (const auto cursor : cursors) {
      id =
          execute(firmware,
                  id,
                  {.bytes = bytes(message), .cursor = cursor, .event = message.event, .busy = true},
                  {.consumed = message.length, .tx = {}});
    }
  }
  for (const bool failure : {true, false}) {
    for (const auto &message : messages) {
      for (const auto cursor : {0U, usb_mask}) {
        id = execute(firmware,
                     id,
                     {.bytes = bytes(message),
                      .cursor = ring_mask,
                      .usb_cursor = cursor,
                      .ccr = failure ? masked : 0x40,
                      .event = message.event,
                      .full = true},
                     {.consumed = message.length, .tx = {}, .failure = failure});
      }
    }
  }
  for (const auto cursor : {0U, usb_mask}) {
    id = execute(firmware,
                 id,
                 {.bytes = bytes(messages[0]),
                  .cursor = ring_mask,
                  .usb_cursor = cursor,
                  .event = messages[0].event,
                  .busy = true,
                  .full = true},
                 {.consumed = 3, .tx = {}, .wait_ack = true});
  }
  return execute(firmware,
                 id,
                 {.bytes = bytes(messages[0]),
                  .cursor = ring_mask + 1,
                  .event = messages[0].event,
                  .raw_alias = true},
                 {.consumed = 3, .tx = {}});
}

unsigned peer_forwarding(const Firmware &firmware, unsigned id) {
  for (const auto &message : messages) {
    for (const auto cursor : cursors) {
      id =
          execute(firmware,
                  id,
                  {.bytes = bytes(message), .cursor = cursor, .event = message.event, .peer = true},
                  {.consumed = message.length, .tx = bytes(message)});
    }
  }
  for (const auto cursor : {0U, ring_mask}) {
    auto source = bytes(messages[0]);
    source.insert(source.begin() + 1, discarded.back());
    id = execute(
        firmware,
        id,
        {.bytes = std::move(source), .cursor = cursor, .event = messages[0].event, .peer = true},
        {.consumed = 3, .tx = bytes(messages[0])});
  }
  for (const auto cursor : cursors) {
    auto source = bytes(messages[0]);
    source.insert(source.begin(), discarded[4]);
    id = execute(firmware,
                 id,
                 {.bytes = source, .cursor = cursor, .peer = true},
                 {.consumed = 1, .tx = source});
  }
  return execute(firmware, id, {.bytes = {}, .peer = true}, {.tx = {}});
}
}
}

int main(const int count, char **arguments) {
  using namespace blofeld::test::midi_forward;
  try {
    blofeld::test::usb_host::require(count == 2, "Usage: original-midi-forward original.zip");
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    const auto completed = peer_forwarding(firmware, backpressure(firmware, ordinary(firmware)));
    blofeld::test::usb_host::require(completed == expected_cases,
                                     "Complete fixed forwarding matrix");
    std::cout << "SUMMARY cases=178 restores=356 own-peripheral-inputs=1 original-boot=0\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
