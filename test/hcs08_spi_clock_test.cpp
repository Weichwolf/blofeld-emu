// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_shi.hpp"
#include "hcs08_spi_fixture.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <tuple>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08_spi_test;
constexpr auto clock_contract = Hcs08SpiClockContract::reference_half_period_and_final_edge;
constexpr ExecutionContext context{.pc = 0xe57f, .ccr = 0x61};
constexpr unsigned last_bit = 7, tx_pattern = 0x5c, rx_pattern = 0xa6;
constexpr unsigned frame_periods = 384, byte_periods = 128;
constexpr unsigned completed_status = 0xa0, changed_baud = 0x13;
constexpr std::array<std::uint8_t, 8> formats{0x50, 0x54, 0x58, 0x5c, 0x51, 0x55, 0x59, 0x5d};

unsigned primary_half_period(const unsigned raw) {
  constexpr std::array<unsigned, 8> prescaler{1, 2, 3, 4, 5, 6, 7, 8};
  constexpr std::array<unsigned, 8> half_divider{1, 2, 4, 8, 16, 32, 64, 128};
  return prescaler[(raw / 16) % 8] * half_divider[raw % 8];
}

bool data_bit(const unsigned data, const unsigned position, const bool lsb) {
  return ((data >> (lsb ? position : last_bit - position)) & 1U) != 0;
}

struct ClockCase {
  unsigned raw, mode;
};

void clock_byte(const ClockCase input) {
  const unsigned raw = input.raw;
  const unsigned mode = input.mode;
  Hcs08Spi spi(contract);
  configure(spi, formats[mode]);
  spi.write({.address = hcs08_spi::baud, .value = static_cast<std::uint8_t>(raw)});
  const unsigned outgoing = raw ^ tx_pattern;
  const unsigned incoming = raw ^ rx_pattern;
  queue(spi, static_cast<std::uint8_t>(outgoing));
  Hcs08SpiClock clock(spi, clock_contract);
  constexpr unsigned idle = 3;
  const auto queued = spi.state();
  for (unsigned n = 0; n < idle; ++n) {
    require(!clock.advance(context, std::nullopt) && spi.state() == queued,
            "Elapsed idle periods cannot choose an unqualified shifter-promotion phase");
  }
  clock.begin_transfer(context);
  const unsigned half = primary_half_period(raw);
  const bool phase = (mode % 2) != 0;
  const bool polarity = ((mode / 2) % 2) != 0;
  const bool lsb = (mode / 4) != 0;
  for (unsigned period = 1; period <= edges * half; ++period) {
    const unsigned edge_count = period / half;
    const bool edge_due = (period % half) == 0;
    const bool sample = edge_due && ((edge_count % 2) != 0) != phase;
    const auto miso =
        sample ? std::optional<bool>(data_bit(incoming, (edge_count - 1) / 2, lsb)) : std::nullopt;
    if (sample) {
      require(spi.pins().mosi == data_bit(outgoing, (edge_count - 1) / 2, lsb),
              "Literal outgoing bit is present before every actual receiver sampling edge");
    }
    const auto event = clock.advance(context, miso);
    const auto s = spi.state();
    const auto t = clock.state();
    const bool finished = period == edges * half;
    require(event.has_value() == edge_due && s.edges == edge_count &&
                s.samples == (edge_count + (phase ? 0U : 1U)) / 2 && s.active == !finished &&
                s.receive_full == finished &&
                spi.pins().clock == (polarity != ((edge_count % 2) != 0)) &&
                t.offered_periods == idle + period && t.active_periods == period &&
                t.generated_edges == edge_count && t.completed_transfers == (finished ? 1U : 0U) &&
                t.half_period == (finished ? 0U : half) &&
                t.remaining_periods == (finished ? 0U : half - (period % half)),
            "Independent divider-table oracle checks every BUSCLK, edge, sample and flag boundary");
    if (event && sample) {
      require(event->mosi == data_bit(outgoing, (edge_count - 1) / 2, lsb),
              "The final edge retains sampled MOSI even after the peripheral releases idle data");
    }
  }
  require(spi.read(hcs08_spi::status) == completed_status &&
              spi.read(hcs08_spi::data) == incoming && !spi.state().receive_full &&
              !spi.pins().mosi,
          "Only completed actual samples produce the byte; real status/data reads acknowledge it");
}

struct Link {
  Hcs08Spi spi{contract};
  DspShi peer;
  Hcs08SpiClock clock{spi, clock_contract};
  std::vector<Hcs08SpiPins> history;
  std::vector<unsigned> received;

  Link() {
    configure(spi);
    select(peer);
    queue(spi, event_bytes[0]);
    clock.begin_transfer(context);
    queue(spi, event_bytes[1]);
  }

  void tick() {
    const auto before = clock.state().completed_transfers;
    if (!clock.state().active && before < event_bytes.size()) {
      clock.begin_transfer(context); // Explicit next-byte promotion reference boundary.
      if (before == 1) { queue(spi, event_bytes[2]); }
    }
    const auto input = peer.outputs().miso;
    const auto pins = clock.advance(context, input.driven ? input.level : std::nullopt);
    if (pins) {
      history.push_back(*pins);
      peer.observe_spi({.selected = true, .clock = pins->clock, .mosi = pins->mosi});
    }
    if (clock.state().completed_transfers != before) {
      require((spi.read(hcs08_spi::status) & hcs08_spi::receive_full) != 0,
              "Clock completion supplies the genuine receive flag");
      received.push_back(spi.read(hcs08_spi::data));
    }
  }
};

struct Snapshot {
  Hcs08Spi spi;
  DspShi peer;
  Hcs08SpiClockCheckpoint clock;
  std::vector<Hcs08SpiPins> history;
  std::vector<unsigned> received;
};

Snapshot capture_link(const unsigned cut) {
  Link link;
  for (unsigned n = 0; n < cut; ++n) { link.tick(); }
  return {.spi = link.spi,
          .peer = link.peer,
          .clock = link.clock.checkpoint(context),
          .history = link.history,
          .received = link.received};
}

auto outcome(Link &link) {
  const auto word = link.peer.read(hrx);
  const auto output = link.peer.outputs();
  return std::tuple(link.spi.state(),
                    link.clock.state(),
                    link.history,
                    link.received,
                    word,
                    output.miso.driven,
                    output.miso.level,
                    output.request.driven,
                    output.request.level,
                    link.peer.interrupt_vectors());
}

void paired_continuations() {
  Link baseline;
  for (unsigned n = 0; n < frame_periods; ++n) { baseline.tick(); }
  require(baseline.received == std::vector<unsigned>(reply_bytes.begin(), reply_bytes.end()) &&
              baseline.history.size() == edges * event_bytes.size() &&
              baseline.clock.state().active_periods == frame_periods &&
              baseline.clock.state().completed_transfers == event_bytes.size(),
          "Three real divider-driven bytes consume384 BUSCLK and48 actual DSP-SHI pin events");
  const auto expected = outcome(baseline);
  require(std::get<4>(expected) == event,
          "The peer assembled the literal24-bit word solely from MOSI edges");
  for (unsigned cut = 0; cut <= frame_periods; ++cut) {
    const auto saved = capture_link(cut); // Original owners are destroyed before either replay.
    for (unsigned replay = 0; replay < 2; ++replay) {
      Link fresh;
      fresh.spi = saved.spi;
      fresh.peer = saved.peer;
      fresh.clock.restore_checkpoint(saved.clock, context);
      fresh.history = saved.history;
      fresh.received = saved.received;
      for (unsigned n = cut; n < frame_periods; ++n) { fresh.tick(); }
      require(
          outcome(fresh) == expected,
          "Every pre/mid/post period fresh graph reproduces pin history, bytes and peer outputs");
    }
  }
}

void ownership_failures() {
  for (unsigned mutation = 0; mutation < 3; ++mutation) {
    Hcs08Spi spi(contract);
    configure(spi);
    queue(spi, tx_pattern);
    Hcs08SpiClock clock(spi, clock_contract);
    clock.begin_transfer(context);
    if (mutation == 0) {
      spi.write({.address = hcs08_spi::baud, .value = changed_baud});
    } else {
      const unsigned transitions = mutation == 1 ? 1 : edges;
      for (unsigned n = 0; n < transitions; ++n) { spi.edge(false); }
      if (mutation == 2) { spi.complete_transfer(); }
    }
    const auto before = spi.state();
    const auto error = rejected([&] { (void)clock.advance(context, false); });
    require(error.contains("phase") && error.contains("pc=0xe57f") &&
                error.contains(mutation == 0 ? "address=0x002a" : "address=0x002d") &&
                clock.faulted() && spi.state() == before && clock.state().offered_periods == 0,
            "External active baud, edge and completion ownership changes refuse before more time");
    require(rejected([&] { (void)clock.checkpoint(context); }).contains("coordinated recovery"),
            "An incomplete clock phase cannot become a trusted snapshot");
  }
}

void unknown_input_recovery() {
  for (unsigned phase = 0; phase < 2; ++phase) {
    Hcs08Spi spi(contract);
    configure(spi, formats[phase]);
    queue(spi, tx_pattern);
    Hcs08SpiClock clock(spi, clock_contract);
    clock.begin_transfer(context);
    const auto saved_spi = spi;
    const auto saved = clock.checkpoint(context);
    const unsigned first_sample = (phase + 1) * (byte_periods / edges);
    for (unsigned n = 1; n < first_sample; ++n) { (void)clock.advance(context, std::nullopt); }
    require(
        rejected([&] { (void)clock.advance(context, std::nullopt); }).contains("pc=0xe57f") &&
            clock.faulted() && spi.faulted() && clock.state().offered_periods == first_sample &&
            spi.state().samples == 0 && clock.state().generated_edges == phase,
        "Unknown sampling input retains elapsed time and prior shift events, with real context");
    spi = saved_spi;
    clock.restore_checkpoint(saved, context);
    for (unsigned n = 0; n < byte_periods; ++n) { (void)clock.advance(context, false); }
    require(spi.state().receive == 0 && spi.state().receive_full && !clock.faulted(),
            "A coordinated peripheral/clock restore genuinely samples known zero bits");
  }
}

void reset_and_contracts() {
  Hcs08Spi spi(contract);
  require(rejected([&] { const Hcs08SpiClock missing(spi); }).contains("explicit") &&
              spi.state() == Hcs08SpiState{} && !spi.faulted(),
          "No implicit successful transfer phase or target clock is selected");
  configure(spi);
  queue(spi, tx_pattern);
  Hcs08SpiClock clock(spi, clock_contract);
  clock.begin_transfer(context);
  for (unsigned n = 0; n < 9; ++n) { (void)clock.advance(context, false); }
  const auto saved_spi = spi;
  const auto saved = clock.checkpoint(context);
  const auto before = clock.state();
  require(rejected([&] { clock.reset(context); }).contains("separately reset") &&
              clock.state() == before && spi.state() == saved_spi.state(),
          "Clock reset cannot silently discard a live peripheral transfer");
  spi = saved_spi;
  clock.restore_checkpoint(saved, context);
  spi.write({.address = hcs08_spi::control1, .value = hcs08_spi::reset_control});
  require(!clock.advance(context, std::nullopt) && !clock.state().active &&
              clock.state().aborted_transfers == 1 && clock.state().completed_transfers == 0 &&
              clock.state().active_periods == before.active_periods && !spi.state().receive_full,
          "Actual SPE disable aborts without inventing another edge or received byte");
  spi.reset();
  clock.reset(context);
  require(clock.state() == Hcs08SpiClockState{} && !clock.faulted(),
          "Separate real peripheral reset permits coordinated clock recovery");
  configure(spi);
  queue(spi, tx_pattern);
  clock.begin_transfer(context);
  const auto active = clock.checkpoint(context);
  const auto retained = clock.state();
  require(spi.read(hcs08_spi::status) == hcs08_spi::transmit_empty,
          "Real status read changes acknowledgement history without consuming clock time");
  require(
      rejected([&] { clock.restore_checkpoint(active, context); }).contains("same separately") &&
          clock.state() == retained,
      "Joint restore rejects mismatched peripheral acknowledgement history before mutation");
}
}

int main() {
  try {
    for (unsigned raw = 0; raw <= byte_mask; ++raw) {
      for (unsigned mode = 0; mode < formats.size(); ++mode) {
        clock_byte({.raw = raw, .mode = mode});
      }
    }
    paired_continuations();
    ownership_failures();
    unknown_input_recovery();
    reset_and_contracts();
    std::cout << "HCS08 SPI reference clock:2048 divider/format cases,385 cuts/770 fresh paired "
                 "continuations, phase ownership and genuine input/reset failures pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
