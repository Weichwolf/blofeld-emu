// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_words.hpp"
#include "audio/serial_pcm.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_gpio.hpp"
#include "serial_pcm_fixture.hpp"
#include "support/serial_pcm.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::pcm::rejects;

constexpr std::uint64_t first_tick = 100, second_tick = 868, frame_end = 1612;
constexpr std::uint64_t duplicate_tick = 111, late_duplicate_tick = 900;
constexpr std::uint64_t attachment_tick = 500, late_tick = 600;
constexpr std::uint64_t next_frame_tick = 1636, missing_tick = 2404;
constexpr unsigned word_bits = 24, positive_word = 0x7fffff, all_bits = 0xffffff;
constexpr unsigned other_word = 0x123456, first_word = 0xa5c39f;
constexpr unsigned mux = 0x878, both_lanes = 33;
constexpr unsigned clock_register = 22, control_register = 21, status_register = 19;
constexpr unsigned clock_value = 0x600301, format_value = 0x7d00;
constexpr std::int32_t positive_maximum = 8388607, signed_first = -5913697, signed_other = 1193046;
using blofeld::test::pcm::require;

struct Source {
  std::array<std::uint32_t, 6> words{0x800000, positive_word, all_bits, 0, other_word, first_word};
  DspEsaiSlot slot{.words = words,
                   .sequence = 1,
                   .index = 0,
                   .channels = 2,
                   .slot_bits = 32,
                   .word_bits = word_bits,
                   .shift = 0,
                   .driven = 63,
                   .known = 63,
                   .high_edges_per_bit = 8,
                   .msb_first = true};
};

void signed_words_and_ownership() {
  Source source;
  PcmWords output({});
  require(!output.observe(source.slot, 0, {.ticks = first_tick, .high_half_period = 3}),
          "One actual word cannot complete stereo");
  source.words[0] = other_word;
  require(!output.observe(source.slot, 0, {.ticks = duplicate_tick, .high_half_period = 3}),
          "Repeated observations do not reread a latch");
  ++source.slot.sequence;
  source.slot.index = 1;
  source.words[0] = positive_word;
  const auto frame = output.observe(source.slot, 0, {.ticks = second_tick, .high_half_period = 3});
  if (!frame) { throw Fault("Expected a complete stereo frame"); }
  require(frame->samples[0] == -8388608 && frame->samples[1] == positive_maximum,
          "Signed extrema and copied first-channel ownership are independent of later writes");
  require(frame->first_tick == first_tick && frame->last_tick == frame_end &&
              frame->bit_ticks == word_bits,
          "Presentation interval derives exactly from the actual divider and device time");
  require(output.completed_frames() == 1 && output.locked() && !output.faulted(),
          "Each complete transmitter frame is counted once");
  require(!output.observe(source.slot, 0, {.ticks = late_duplicate_tick, .high_half_period = 3}),
          "Completed-frame observations cannot duplicate PCM");
}

void alignment_and_rejections() {
  Source source;
  PcmWords late({}, attachment_tick);
  require(!late.observe(source.slot, 0, {.ticks = first_tick, .high_half_period = 3}),
          "Attachment threshold retains the seen sequence");
  require(!late.observe(source.slot, 0, {.ticks = late_tick, .high_half_period = 3}),
          "A late callback cannot timestamp an old word anew");
  source.slot.index = 1;
  ++source.slot.sequence;
  require(!late.observe(source.slot, 0, {.ticks = second_tick, .high_half_period = 3}),
          "Joining mid-frame never fabricates a first channel");
  ++source.slot.sequence;
  source.slot.index = 0;
  require(!late.observe(source.slot, 5, {.ticks = next_frame_tick, .high_half_period = 3}),
          "Next complete frame starts on actual channel zero");
  source.slot.sequence += 2;
  source.slot.index = 1;
  require(rejects([&] {
            static_cast<void>(
                late.observe(source.slot, 5, {.ticks = missing_tick, .high_half_period = 3}));
          }).contains("lost"),
          "Missing words fail visibly rather than dropping output");
  require(late.faulted(), "Failure requires an explicit reset");
  late.reset();
  require(!late.locked() && !late.faulted(), "Coordinated reset removes stale alignment");
  for (unsigned invalid = 0; invalid < 7; ++invalid) {
    Source input;
    PcmWords target({});
    if (invalid == 0) { input.slot.known = 0; }
    if (invalid == 1) { input.slot.driven = 0; }
    if (invalid == 2) { input.slot.word_bits = 16; }
    if (invalid == 3) { input.slot.shift = 1; }
    if (invalid == 4) { input.slot.msb_first = false; }
    if (invalid == 5) { input.slot.high_edges_per_bit = 0; }
    const auto half = invalid == 6 ? UINT64_MAX : 3;
    static_cast<void>(rejects([&] {
      static_cast<void>(
          target.observe(input.slot, 0, {.ticks = first_tick, .high_half_period = half}));
    }));
  }
}

void narrow_words() {
  for (const unsigned bits : {8U, 12U, 16U, 20U, 24U}) {
    Source input;
    input.slot.word_bits = bits;
    input.slot.shift = word_bits - bits;
    input.words[0] = all_bits;
    PcmWords output({.word_bits = bits});
    require(!output.observe(input.slot, 0, {.ticks = first_tick, .high_half_period = 1}),
            "First narrow sample starts the frame");
    ++input.slot.sequence;
    input.slot.index = 1;
    input.words[0] = positive_word;
    const auto frame = output.observe(input.slot, 0, {.ticks = 356, .high_half_period = 1});
    require(frame && frame->samples[0] == -1 && frame->samples[1] == (1 << (bits - 1)) - 1,
            "Each supported word width extracts signed payload without padding or rounding");
  }
}

void decoded_latches() {
  DspEsai serial(DspEsaiInstance::secondary);
  constexpr auto timing = DspEsaiClockTiming::reference_reset_edges;
  serial.observe({.high_clock = false, .timing = timing});
  serial.write(clock_register, clock_value);
  serial.write(control_register, format_value | 1U);
  serial.write(0, first_word);
  serial.set_active(true);
  SerialPcmReceiver reference({.sample_edge = SerialPcmEdge::falling});
  require(!reference.observe(0, {.clock = false, .frame_sync = false}),
          "An inactive baseline emits no reference samples");
  PcmWords output({});
  std::vector<SerialPcmFrame> frames;
  std::uint64_t sequence = 0;
  constexpr std::uint64_t maximum_edges = 20000;
  for (std::uint64_t tick = 1; reference.completed_frames() < 2; ++tick) {
    require(tick < maximum_edges, "Independent bit decoder completes its finite input");
    serial.observe({.high_clock = tick % 2 != 0, .timing = timing});
    const auto slot = serial.transmitted_slot();
    if (slot) {
      const auto frame = output.observe(*slot, 0, {.ticks = tick, .high_half_period = 1});
      if (frame) { frames.push_back(*frame); }
      if (slot->sequence != sequence) {
        sequence = slot->sequence;
        serial.write(0, slot->index == 0 ? other_word : first_word);
      }
    }
    const auto clocks = serial.clock_outputs();
    const auto data = serial.outputs();
    const auto frame = reference.observe(tick,
                                         {.clock = (clocks.levels & 1U) != 0,
                                          .frame_sync = (clocks.levels & 2U) != 0,
                                          .data = (data.levels & 1U) != 0});
    if (!frame) { continue; }
    // Both paths start from the observed inactive baseline; only their actual
    // latch and sampling-edge availability times differ.
    const auto &direct = frames.at(reference.completed_frames() - 1);
    require(direct.samples[0] == signed_first && direct.samples[1] == signed_other &&
                direct.samples == frame->samples,
            "Actual negative/positive word pairs match an independent bit decoder");
    constexpr unsigned bit_ticks = 8;
    require(direct.bit_ticks == bit_ticks && frame->bit_ticks == bit_ticks &&
                frame->first_tick == direct.first_tick + bit_ticks / 2,
            "Latch availability and opposite-edge sampling retain their actual half-bit phase");
  }
}

void bus_latches(const DspEsaiInstance instance) {
  DspBus bus;
  const auto space = instance == DspEsaiInstance::primary ? DspSpace::x : DspSpace::y;
  const auto base = instance == DspEsaiInstance::primary ? 0xffffa0U : 0xffff80U;
  const auto port = instance == DspEsaiInstance::primary ? DspPort::c : DspPort::e;
  const auto write = [&](const unsigned offset, const unsigned value) {
    bus.write({.space = space, .address = base + offset, .value = value});
  };
  write(clock_register, clock_value);
  write(control_register, format_value);
  write(0, first_word);
  write(5, other_word);
  bus.gpio(port).write({.reg = DspGpioRegister::control, .value = mux});
  bus.gpio(port).write({.reg = DspGpioRegister::direction, .value = mux});
  bus.observe_esai_transmit(
      instance, {.high_clock = false, .timing = DspEsaiClockTiming::reference_reset_edges});
  write(control_register, (format_value | both_lanes));
  PcmWords output({});
  std::uint64_t at = 0;
  while (output.completed_frames() < 2) {
    const auto horizon = bus.esai_high_clock_edges_until_event(instance);
    if (!horizon) { throw Fault("Expected a running transmitter horizon"); }
    require(*horizon != 0, "A running output has an actual next transmitter event");
    bus.advance_esai_high_clock(instance, *horizon);
    at += *horizon;
    const auto slot = bus.esai_transmitted_slot(instance);
    if (!slot) { continue; }
    const auto frame = output.observe(*slot, 0, {.ticks = at, .high_half_period = 1});
    if (frame) {
      require(
          frame->samples[0] == signed_first && frame->samples[1] == signed_first,
          "An actual hardware underrun repeats its TX value, without invented replacement audio");
    }
    const auto second = bus.esai_transmitted_slot(instance);
    require(second && second->words[5] == other_word && second->driven == both_lanes,
            "Both selected lanes expose their actual borrowed latch words");
  }
  require((bus.read({.space = space, .address = base + status_register}) & 0x4000U) != 0,
          "Word output preserves firmware-visible underrun status");
  constexpr unsigned output_clocks = 0x18;
  bus.gpio(port).write({.reg = DspGpioRegister::control, .value = mux & ~output_clocks});
  const auto disconnected = bus.esai_transmitted_slot(instance);
  require(disconnected && disconnected->driven == 0 && disconnected->known == 0,
          "Absent output clocks cannot expose a qualified DAC word stream");
}
}

int main() {
  try {
    signed_words_and_ownership();
    alignment_and_rejections();
    narrow_words();
    decoded_latches();
    bus_latches(DspEsaiInstance::primary);
    bus_latches(DspEsaiInstance::secondary);
    std::cout << "Actual PCM word ownership, timing, precision and failure checks passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
