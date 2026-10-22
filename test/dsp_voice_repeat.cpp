// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_voice_repeat.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_dma.hpp"
#include "blofeld/dsp_esai.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "blofeld/serial_pcm.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <ios>
#include <optional>
#include <ostream>

namespace blofeld::test {
namespace {
constexpr unsigned source = 0xffffef;
constexpr unsigned destination = 0xffffee;
constexpr unsigned counter = 0xffffed;
constexpr unsigned control = 0xffffec;
constexpr unsigned offset0 = 0xfffff3;
constexpr unsigned offset1 = 0xfffff2;
constexpr unsigned transmit = 0xffff80;
constexpr unsigned serial_status = 0xffff93;
constexpr unsigned underrun = 0x4000;
constexpr unsigned ring_start = 0x660;
constexpr unsigned half_words = 32;
constexpr unsigned half_frames = 16;
constexpr unsigned ring_count = 0xfc0;
constexpr unsigned half_count = 0x7c0;
constexpr unsigned ring_rewind = 0xffffc1;
// DSP56300FM DTM5, DRS22, mode C, 3D source, fixed Y destination.
constexpr unsigned repeating = 0xa8b604;
constexpr unsigned maximum_effects = 64;
constexpr std::uint64_t maximum_edges = 1000000;
constexpr auto serial = DspEsaiInstance::secondary;

unsigned read(DspBus &bus, const unsigned at) {
  return bus.read({.space = DspSpace::x, .address = at});
}
}

void VoiceRepeat::append(DspProgram &program) {
  entry = program.pc();
  for (const auto store : {DspStore{.address = offset0, .value = 1},
                           DspStore{.address = offset1, .value = ring_rewind},
                           DspStore{.address = source, .value = ring_start},
                           DspStore{.address = destination, .value = transmit},
                           DspStore{.address = counter, .value = ring_count},
                           DspStore{.address = control, .value = repeating}}) {
    program.emit("movep #>" + number(store.value) + ",x:<<" + number(store.address));
  }
  program.emit("jmp $198"); // Enter the original foreground; it never returns to this caller.
}

void VoiceRepeat::instruction(DspMachine &machine) {
  auto progress = machine.begin_reference_instruction(DspReferenceExecution::access_boundaries);
  unsigned effects = 0;
  while (!progress.retired) {
    if (!progress.pending) { throw Fault("Original stream instruction lost its admission ticket"); }
    require(effects++ < maximum_effects, "Original stream instruction effect bound");
    progress = machine.advance_reference_access(progress.pending->ticket);
  }
  require(!progress.pending, "Original stream instruction must retire before peripheral phases");
}

void VoiceRepeat::begin(DspBus &bus) {
  half = words = frames = 0;
  edge = cycle = 0;
  receiver.reset();
  bus.observe_esai_transmit(
      serial, {.high_clock = false, .timing = DspEsaiClockTiming::reference_reset_edges});
  static_cast<void>(grant(bus));
  const auto clocks = bus.esai_clock_outputs(serial);
  require(
      clocks.driven == 3 && clocks.known == 3 &&
          !receiver.observe(
              edge, {.clock = (clocks.levels & 1U) != 0, .frame_sync = (clocks.levels & 2U) != 0}),
      "Repeating serial receiver begins at the actual known primed baseline");
}

DspDmaBusResult VoiceRepeat::grant(DspBus &bus) {
  // Hold the NEXT real request once this half is complete. It must survive the
  // foreground work; this is an explicit laboratory grant, not a silicon stall.
  const auto result = bus.advance_dma(
      {.cycle = cycle++,
       .timing = DspDmaTiming::reference_sampled_cycles,
       .grant = words == half_words ? DspDmaGrant::wait : DspDmaGrant::available},
      {.accesses = {}, .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy});
  require(!result.core_stall, "The core is actually paused during serial/DMA phases");
  if (result.word) {
    require(words < transfers.size(), "No repeating DMA word may be dropped");
    transfers.at(words++) = *result.word;
  }
  return result;
}

DspDmaBusResult VoiceRepeat::step(DspBus &bus) {
  require(edge < maximum_edges, "Repeating serial edge bound");
  ++edge;
  bus.observe_esai_transmit(
      serial, {.high_clock = edge % 2 != 0, .timing = DspEsaiClockTiming::reference_reset_edges});
  const auto clocks = bus.esai_clock_outputs(serial);
  const auto data = bus.esai_transmit_outputs(serial);
  require(clocks.driven == 3 && clocks.known == 3,
          "Repeating serial clock/frame outputs must remain known");
  const auto frame = receiver.observe(edge,
                                      {.clock = (clocks.levels & 1U) != 0,
                                       .frame_sync = (clocks.levels & 2U) != 0,
                                       .data = (data.driven & data.known & 1U) != 0
                                                   ? std::optional{(data.levels & 1U) != 0}
                                                   : std::nullopt});
  if (frame) {
    require(frames < decoded.size(), "No repeating serial frame may be dropped");
    decoded.at(frames++) = *frame;
  }
  return grant(bus);
}

void VoiceRepeat::verify(DspBus &bus) const {
  const auto start = ring_start + ((half % 2) * half_words);
  const auto next = ring_start + (((half + 1) % 2) * half_words);
  require(words == half_words && frames == half_frames && read(bus, source) == next &&
              read(bus, counter) == (half % 2 == 0 ? half_count : ring_count) &&
              read(bus, control) == repeating && read(bus, destination) == transmit,
          "Original stream retains enabled repeating DMA with exact half/ring reload");
  require((bus.read({.space = DspSpace::y, .address = serial_status}) & underrun) == 0,
          "No repeating serial underrun may be hidden");
  for (unsigned i = 0; i < words; ++i) {
    const auto &word = transfers.at(i);
    require(word.channel == 0 && word.source == DspDmaAddress{.address = start + i} &&
                word.destination == DspDmaAddress{.space = DspDmaSpace::y, .address = transmit},
            "Repeating DMA supplies consecutive original ring words to actual ESAI TX0");
  }
}

void VoiceRepeat::finish(DspBus &bus, const std::uint64_t target) {
  while (receiver.completed_frames() < target) { static_cast<void>(step(bus)); }
  verify(bus);
}

void VoiceRepeat::drain(DspBus &bus, const DspCore &cpu, std::ostream &trace) {
  const auto before = cpu.debug_state();
  words = frames = 0;
  const auto target = receiver.completed_frames() + half_frames;
  static_cast<void>(grant(bus));
  static_cast<void>(grant(bus));
  if (half == 0) {
    while (true) {
      const auto result = step(bus);
      if (result.phase && result.phase->kind == DspDmaPhaseKind::source_read) { break; }
    }
    const auto saved_bus = bus;
    const auto saved_wire = *this;
    finish(bus, target);
    const auto completed = *this;
    bus = saved_bus;
    *this = saved_wire;
    finish(bus, target);
    require(*this == completed, "Repeating held-word restore reproduces serial/DMA output");
    trace << "REPEAT_RESTORE 1\n";
  }
  finish(bus, target);
  require(cpu.debug_state() == before, "Serial phases cannot execute a hidden DSP instruction");
  emit(trace);
  ++half;
}

void VoiceRepeat::emit(std::ostream &trace) const {
  trace << std::hex;
  for (unsigned i = 0; i < words; ++i) {
    const auto &word = transfers.at(i);
    trace << "REPEAT_DMA " << i << ' ' << word.source.address << ' ' << word.destination.address
          << ' ' << word.value << '\n';
  }
  for (unsigned i = 0; i < frames; ++i) {
    const auto &frame = decoded.at(i);
    if (!frame.bit_ticks) { throw Fault("Repeating frame has no bit period"); }
    require(frame.channels == 2 && frame.uniform_clock,
            "Repeating PCM retains known uniform two-channel timing");
    trace << "REPEAT_FRAME " << i << ' ' << frame.first_tick << ' ' << frame.last_tick << ' '
          << *frame.bit_ticks;
    for (unsigned lane = 0; lane < 2; ++lane) {
      trace << ' ' << (static_cast<unsigned>(frame.samples.at(lane)) & word_mask);
    }
    trace << '\n';
  }
  trace << std::dec;
}
}
