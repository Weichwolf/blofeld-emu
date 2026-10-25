// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_shi.hpp"
#include "hcs08_spi_fixture.hpp"
#include "keyboard/hcs08_spi.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08_spi_test;
constexpr unsigned format_base = 0x50, receiver_control = 0xd4, last_bit = 7;
constexpr unsigned baud_mask = 0x77, unsupported_control2 = 0x13;
constexpr unsigned data_a = 0xa5, data_b = 0x3c, xor_mask = 0x55, full_status = 0xa0;
constexpr unsigned spi_reserved = 0x2c, pc = 0x1234, flags = 0x79;

void registers() {
  require(rejected([] { const Hcs08Spi missing; }).contains("explicit candidate"),
          "No default target or clock hypothesis can select a successful SPI");
  Hcs08Spi device(contract);
  require(device.state() == Hcs08SpiState{} && device.read(spi::data) == 0 &&
              device.read(spi::status) == spi::transmit_empty && !device.pins().driven &&
              !device.interrupt_requested() && device.clock_divisor() == 2,
          "Independent reset registers/data/flags/ownership/divisor match GT chapter12");
  constexpr std::array<unsigned, 8> divisors{2, 4, 8, 16, 32, 64, 128, 256};
  for (unsigned value = 0; value <= byte_mask; ++value) {
    device.write({.address = spi::baud, .value = static_cast<std::uint8_t>(value)});
    require(device.read(spi::baud) == (value & baud_mask) &&
                device.clock_divisor() == (1 + (value / 16) % 8) * divisors[value % 8],
            "All256 baud values use independent primary prescaler/divisor tables");
    Hcs08Spi c1(contract);
    if ((value & spi::enable) != 0 && (value & spi::master) == 0) {
      require(rejected([&] {
                c1.write({.address = spi::control1, .value = static_cast<std::uint8_t>(value)});
              }).contains("slave") &&
                  c1.state() == Hcs08SpiState{},
              "Every enabled unsupported slave role rejects before register mutation");
    } else {
      c1.write({.address = spi::control1, .value = static_cast<std::uint8_t>(value)});
      require(c1.read(spi::control1) == value && c1.read(spi::status) == spi::transmit_empty,
              "Disabled configurations and all supported master formats retain defined fields");
    }
    Hcs08Spi c2(contract);
    const auto before = c2.state();
    if ((value & unsupported_control2) != 0) {
      require(rejected([&] {
                c2.write({.address = spi::control2, .value = static_cast<std::uint8_t>(value)});
              }).contains("unqualified") &&
                  c2.state() == before && c2.faulted(),
              "Every unsupported control2 feature refuses before mutation");
    } else {
      c2.write({.address = spi::control2, .value = static_cast<std::uint8_t>(value)});
      require(c2.read(spi::control2) == (value & 8U),
              "Unimplemented control2 bits read zero; BIDIROE has no effect in full duplex");
    }
  }
}

std::uint8_t format(const unsigned mode) {
  return static_cast<std::uint8_t>(format_base | ((mode & 1U) != 0 ? 4U : 0U) |
                                   ((mode & 2U) != 0 ? 8U : 0U) | (mode >> 2U));
}

bool bit(const unsigned value, const unsigned position, const bool lsb) {
  return ((value >> (lsb ? position : last_bit - position)) & 1U) != 0;
}

struct ByteCase {
  unsigned mode, tx, rx;
};

void edge_byte(Hcs08Spi &device, const ByteCase input) {
  const auto mode = input.mode;
  const auto tx = input.tx;
  const auto rx = input.rx;
  const bool phase = (mode & 1U) != 0;
  const bool polarity = (mode & 2U) != 0;
  const bool lsb = (mode & 4U) != 0;
  const auto from = device.state().edges;
  for (unsigned edge = from; edge < edges; ++edge) {
    const bool sample = (edge % 2 == 0) != phase;
    const auto observed = sample ? std::optional<bool>(bit(rx, edge / 2, lsb)) : std::nullopt;
    device.edge(observed);
    const auto pads = device.pins();
    const unsigned position = phase ? edge / 2 : std::min((edge + 1) / 2, last_bit);
    require(pads.driven && pads.clock == (polarity != (edge % 2 == 0)) &&
                pads.mosi == bit(tx, position, lsb) && device.state().edges == edge + 1 &&
                device.state().samples == (edge + (phase ? 1U : 2U)) / 2,
            "Independent CPOL/CPHA/MSB/LSB pin and sample chronology at every edge");
  }
  require(!device.state().receive_full && device.state().samples == samples,
          "All sampled bits precede the separate untimed completion event");
  device.complete_transfer();
  require(device.state().receive == rx && device.state().receive_full && !device.state().active,
          "Only sixteen actual edges/eight known inputs create the received byte");
}

void bytes_and_copies() {
  for (unsigned mode = 0; mode < 8; ++mode) {
    for (unsigned value = 0; value <= byte_mask; ++value) {
      Hcs08Spi device(contract);
      configure(device, format(mode));
      device.write({.address = spi::data, .value = static_cast<std::uint8_t>(value)});
      require(!device.state().transmit, "SPID without the required status read is ignored");
      queue(device, static_cast<std::uint8_t>(value));
      require(device.read(spi::status) == 0, "A genuine queued byte clears SPTEF");
      device.transfer_to_shifter();
      const unsigned incoming = (value ^ xor_mask) & byte_mask;
      edge_byte(device, {.mode = mode, .tx = value, .rx = incoming});
      require(device.read(spi::data) == incoming && device.state().receive_full,
              "An unarmed data read cannot clear SPRF");
      require(device.read(spi::status) == full_status && device.read(spi::data) == incoming &&
                  device.read(spi::status) == spi::transmit_empty,
              "Actual status/data sequence acknowledges only the old received byte");
    }
    for (unsigned cut = 0; cut <= edges; ++cut) {
      Hcs08Spi original(contract);
      configure(original, format(mode));
      queue(original, data_a);
      original.transfer_to_shifter();
      const bool phase = (mode & 1U) != 0;
      const bool lsb = (mode & 4U) != 0;
      for (unsigned n = 0; n < cut; ++n) {
        original.edge((n % 2 == 0) != phase ? std::optional<bool>(bit(data_b, n / 2, lsb))
                                            : std::nullopt);
      }
      for (unsigned replay = 0; replay < 2; ++replay) {
        auto restored = original;
        edge_byte(restored, {.mode = mode, .tx = data_a, .rx = data_b});
        require(restored.read(spi::status) == full_status && restored.read(spi::data) == data_b,
                "Fresh copied shifters preserve partial bits, pads and status-ack history");
      }
    }
  }
}

void pipeline() {
  Hcs08Spi device(contract);
  configure(device);
  require(device.interrupt_requested(), "Original74 control requests actual SPTEF interrupt");
  queue(device, data_a);
  require(!device.interrupt_requested(), "Occupied transmit buffer removes its IRQ request");
  device.transfer_to_shifter();
  require(device.interrupt_requested(), "Real buffer-to-shifter transfer reasserts SPTEF");
  queue(device, data_b);
  require(!device.interrupt_requested(), "Second genuine queued byte occupies the separate buffer");
  edge_byte(device, {.mode = 1, .tx = data_a, .rx = xor_mask});
  require(device.read(spi::status) == spi::receive_full && device.state().transmit == data_b,
          "Completion cannot overwrite the queued byte or invent a buffer transfer");
  device.transfer_to_shifter();
  for (unsigned n = 0; n < edges; ++n) { device.edge(false); }
  device.complete_transfer();
  require(device.read(spi::status) == full_status && device.read(spi::data) == xor_mask,
          "Receive overrun preserves the unread first byte and adds no invented overrun flag");
  device.write({.address = spi::control1, .value = receiver_control});
  require(!device.interrupt_requested(), "Only actual enabled flag sources request an IRQ");
  queue(device, data_b);
  device.transfer_to_shifter();
  edge_byte(device, {.mode = 1, .tx = data_b, .rx = data_a});
  require(device.interrupt_requested(), "SPRF requests IRQ through SPIE independently of SPTIE");
  static_cast<void>(device.read(spi::status));
  static_cast<void>(device.read(spi::data));
  require(!device.interrupt_requested(), "Acknowledging actual receive data clears its IRQ");
}

void failures_and_disable() {
  Hcs08Spi device(contract);
  device.context({.pc = pc, .ccr = flags});
  configure(device);
  queue(device, data_a);
  device.transfer_to_shifter();
  device.edge(std::nullopt); // CPHA1 shift needs no MISO observation.
  const auto before = device.state();
  const auto failure = rejected([&] { device.edge(std::nullopt); });
  require(failure.contains("MISO sample address=0x002d pc=0x1234 ccr=0x0079") && device.faulted() &&
              device.state() == before,
          "Unknown input fails visibly before sampling, changing a pad or shifting a bit");
  require(rejected([&] { device.edge(false); }).contains("coordinated recovery"),
          "A failed wire edge cannot be silently retried");
  device.reset();
  configure(device);
  queue(device, data_a);
  device.transfer_to_shifter();
  const auto active = device.state();
  require(rejected([&] { device.complete_transfer(); }).contains("all16 actual") &&
              device.state() == active,
          "No host completion shortcut bypasses real clock edges");
  device.reset();
  configure(device);
  queue(device, data_a);
  device.transfer_to_shifter();
  device.edge(std::nullopt);
  device.write({.address = spi::control1, .value = spi::reset_control});
  require(
      !device.state().active && !device.state().transmit && !device.state().receive_full &&
          device.read(spi::status) == spi::transmit_empty && !device.pins().driven,
      "Documented SPE clear aborts shifters and initializes flags without a fake received byte");
  device.reset();
  const auto reset = device.state();
  device.write({.address = spi::status, .value = byte_mask});
  require(device.state() == reset, "SPIS writes have no effect, including all ones");
  require(rejected([&] { static_cast<void>(device.read(spi_reserved)); }).contains("reserved"),
          "The hole in the SPI map remains visibly unsupported");
  device.reset();
  configure(device);
  queue(device, data_a);
  device.transfer_to_shifter();
  const auto running = device.state();
  require(rejected([&] {
            device.write({.address = spi::control1, .value = control ^ spi::lsb_first});
          }).contains("active format") &&
              device.state() == running,
          "An unsupported active shifter-format change cannot silently reinterpret in-flight bits");
}

struct Pair {
  Hcs08Spi device{contract};
  DspShi peer;
  unsigned byte = 0, transitions = 0;
  std::array<std::uint8_t, 3> received{};

  Pair() {
    configure(device);
    select(peer);
    queue(device, event_bytes[0]);
    device.transfer_to_shifter();
  }

  void step() {
    require(byte < event_bytes.size(), "Only three actual SPI byte transfers form the own record");
    coupled_edge(device, peer);
    ++transitions;
    if (device.state().edges != edges) { return; }
    device.complete_transfer();
    require(device.read(spi::status) == full_status,
            "Real paired completion creates actual RX data");
    received[byte] = device.read(spi::data);
    ++byte;
    if (byte < event_bytes.size()) {
      queue(device, event_bytes[byte]);
      device.transfer_to_shifter();
    }
  }

  void finish() {
    while (byte < event_bytes.size()) { step(); }
    require(transitions == edges * event_bytes.size() && received == reply_bytes &&
                peer.read(hrx) == event,
            "Three real master bytes form the exact DSP24-bit word and return all peer bits");
  }
};

void coupled_copies() {
  constexpr std::array<unsigned, 6> cuts{0, 7, 16, 23, 32, 47};
  for (const auto cut : cuts) {
    Pair original;
    while (original.transitions < cut) { original.step(); }
    for (unsigned replay = 0; replay < 2; ++replay) {
      auto restored = original;
      restored.finish();
    }
  }
}
}

int main() {
  try {
    registers();
    bytes_and_copies();
    pipeline();
    failures_and_disable();
    coupled_copies();
    std::cout << "HCS08 SPI registers, real master edges and paired DSP24-bit transfers pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
