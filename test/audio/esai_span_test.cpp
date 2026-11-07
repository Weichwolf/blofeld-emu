// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_esai_registers.hpp"
#include "serial_pcm_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace {
using namespace blofeld;
using blofeld::test::pcm::require;
constexpr unsigned clock_base = 0x600101, first_word = 0xa5c39f;
constexpr unsigned maximum_iterations = 10000;

void same(DspEsai &fast, DspEsai &edges) {
  require(fast.outputs() == edges.outputs() && fast.clock_outputs() == edges.clock_outputs(),
          "Quiet spans preserve independently stepped bit/clock outputs");
  require(fast.read(esai::saisr) == edges.read(esai::saisr) &&
              fast.transmit_request() == edges.transmit_request() &&
              fast.interrupt_vector() == edges.interrupt_vector(),
          "Quiet spans preserve actual status, DMA requests and IRQ priorities");
  require(fast.frames_started() == edges.frames_started() &&
              fast.slots_started() == edges.slots_started() &&
              fast.high_clock_edges_until_event() == edges.high_clock_edges_until_event(),
          "Quiet spans retain frame/slot counts and the exact next event time");
  const auto a = fast.transmitted_slot();
  const auto b = edges.transmitted_slot();
  require(a.has_value() == b.has_value(), "Latch availability matches edge execution");
  if (!a || !b) { return; }
  require(a->sequence == b->sequence && a->index == b->index && a->driven == b->driven &&
              a->known == b->known && std::ranges::equal(a->words, b->words),
          "Word latches match the original edge path on every split span");
}

void span_case(const unsigned format, const unsigned options) {
  DspEsai fast(DspEsaiInstance::secondary);
  constexpr auto timing = DspEsaiClockTiming::reference_reset_edges;
  require(!fast.high_clock_edges_until_event(), "Cold inactive hardware has no event");
  fast.observe({.high_clock = false, .timing = timing});
  const auto polarity = ((options & 1U) != 0 ? esai::clock_polarity : 0U) |
                        ((options & 2U) != 0 ? esai::frame_polarity : 0U) |
                        ((options & 4U) != 0 ? esai::high_clock_polarity : 0U);
  const auto divider = (options & 8U) != 0 ? 0x4000U : 0U;
  const auto slots = (options & 16U) != 0 ? 0x400U : 0x200U;
  fast.write(esai::tccr, clock_base | polarity | divider | slots);
  const auto control = (format << 10U) | esai::network | esai::data_interrupt |
                       esai::exception_interrupt | ((options & 32U) != 0 ? esai::bit_sync : 0U) |
                       ((options & 64U) != 0 ? esai::shift_lsb : 0U) | 1U;
  fast.write(esai::tcr, control);
  fast.write(0, first_word);
  require(!fast.high_clock_edges_until_event(), "Configuration alone does not connect the clock");
  fast.set_active(true);
  auto edges = fast;
  bool level = false;
  unsigned iteration = 0;
  constexpr std::array cuts{1U, 3U, 17U, 1024U};
  while (fast.frames_started() < 4) {
    require(iteration < maximum_iterations, "Directed finite span case reaches four real frames");
    const auto horizon = fast.high_clock_edges_until_event();
    if (!horizon) { throw Fault("Configured transmitter must supply an event horizon"); }
    const auto count = std::min(*horizon, std::uint64_t{cuts.at(iteration % cuts.size())});
    fast.advance_high_clock(count);
    for (std::uint64_t edge = 0; edge < count; ++edge) {
      level = !level;
      edges.observe({.high_clock = level, .timing = timing});
    }
    same(fast, edges);
    if (iteration % 7 == 0) {
      const auto value = iteration % 14 == 0 ? 0x123456U : 0xa5c39fU;
      fast.write(0, value);
      edges.write(0, value);
    }
    ++iteration;
  }
}

void configuration_horizons() {
  DspEsai serial(DspEsaiInstance::secondary);
  constexpr auto timing = DspEsaiClockTiming::reference_reset_edges;
  constexpr unsigned two_slots = 0x200;
  constexpr unsigned two_bit_divider = 0x4000;
  const auto control = esai::network | esai::bit_sync | 1U;
  require(!serial.high_clock_edges_until_event(),
          "Inactive event absence can be queried repeatedly");
  serial.observe({.high_clock = false, .timing = timing});
  serial.write(esai::tccr, clock_base | two_slots);
  serial.write(esai::tcr, control);
  require(!serial.high_clock_edges_until_event(), "Inactive configuration retains no deadline");
  serial.set_active(true);
  // HCKT /2 needs two rising edges: rises 1 and 3 launch the first generated bit.
  require(serial.high_clock_edges_until_event() == 3,
          "Activation replaces an absent deadline with the literal first launch edge");
  bool rejected = false;
  try {
    serial.write(esai::saicr, 1); // Unsupported common-control bit.
  } catch (const Fault &) { rejected = true; }
  require(rejected && serial.high_clock_edges_until_event() == 3,
          "A rejected configuration preserves the previous actual deadline");
  auto copy = serial;
  copy.advance_high_clock(1);
  require(copy.high_clock_edges_until_event() == 2 && serial.high_clock_edges_until_event() == 3,
          "A copied quiet countdown advances independently without moving the original event");
  serial.observe({.high_clock = true, .timing = timing});
  require(serial.high_clock_edges_until_event() == 2,
          "An actual input observation replaces the previous countdown");
  serial.observe({.high_clock = false, .timing = timing});
  require(serial.high_clock_edges_until_event() == 1,
          "A falling edge retains the next rising edge's literal position");
  serial.set_active(false);
  require(!serial.high_clock_edges_until_event(), "Disconnection withdraws the previous deadline");
  serial.set_active(true);
  require(serial.high_clock_edges_until_event() == 3,
          "Reconnection starts from reset divider state and the actual current clock level");
  serial.write(esai::tcr, control | esai::personal_reset);
  require(!serial.high_clock_edges_until_event(), "Personal reset withdraws a running event");
  serial.write(esai::tcr, control);
  require(serial.high_clock_edges_until_event() == 3,
          "Reset release restores the first real event");
  serial.set_active(false);
  serial.reset();
  require(!serial.high_clock_edges_until_event(), "Hardware reset discards a retained countdown");
  serial.write(esai::tccr, clock_base | two_slots | two_bit_divider);
  serial.write(esai::tcr, control);
  serial.set_active(true);
  // Four rises (1,3,5,7) are now required for the first generated bit.
  require(serial.high_clock_edges_until_event() == 7,
          "A new clock divider cannot reuse the earlier first-launch deadline");
}

}

int main() {
  try {
    configuration_horizons();
    for (const unsigned format : {0U, 2U, 30U, 31U}) {
      for (unsigned options = 0; options < 128; ++options) { span_case(format, options); }
    }
    std::cout << "512 directed quiet-span cases match the stepped transmitter.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
