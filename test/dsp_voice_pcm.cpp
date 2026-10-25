// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_voice_pcm.hpp"

#include "audio/serial_pcm.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <ios>
#include <optional>
#include <ostream>

namespace blofeld::test {
namespace {
constexpr auto serial = DspEsaiInstance::secondary;
constexpr auto timing = DspEsaiClockTiming::reference_reset_edges;
constexpr unsigned transmit = 0xffff80;
constexpr unsigned status = 0xffff93;
constexpr unsigned control = 0xffff95;
constexpr unsigned clock = 0xffff96;
constexpr unsigned port_direction = 0xffff9e;
constexpr unsigned port_control = 0xffff9f;
constexpr unsigned source = 0xffffef;
constexpr unsigned destination = 0xffffee;
constexpr unsigned counter = 0xffffed;
constexpr unsigned dma_control = 0xffffec;
constexpr unsigned dma_status = 0xfffff4;
constexpr unsigned enable = 0x800000;
// DSP56300FM DTM1, priority1, DRS22, linear X source/fixed Y destination.
constexpr unsigned single_block = 0x8ab254;
constexpr unsigned phase_slot = 0x802;
constexpr unsigned first_half = 0x660;
constexpr unsigned second_half = 0x680;
constexpr unsigned word_count = 32;
constexpr unsigned frame_count = word_count / 2;
constexpr unsigned maximum_edges = 100000;
constexpr unsigned underrun = 0x4000;
constexpr unsigned all_done = 0x3f;

unsigned read(DspBus &bus, const unsigned address) {
  return bus.read({.space = DspSpace::x, .address = address});
}

void put(DspProgram &program, const DspSpace space, const unsigned address, const unsigned value) {
  program.emit("movep #>" + number(value) + (space == DspSpace::x ? ",x:<<" : ",y:<<") +
               number(address));
}
}

void VoicePcm::append(DspProgram &program) {
  setup = program.pc();
  constexpr unsigned word32 = 0x7d00;
  constexpr unsigned generated_clock = 0x640301;
  constexpr unsigned mux = 0x838;
  constexpr unsigned mix_signal = 0x200;
  put(program, DspSpace::y, control, word32);
  put(program, DspSpace::y, clock, generated_clock);
  put(program, DspSpace::y, port_control, mux);
  put(program, DspSpace::y, port_direction, mux | mix_signal);
  put(program, DspSpace::y, transmit, 0);
  put(program, DspSpace::y, control, word32 | 1U);
  program.emit("rts");
  transfer = program.pc();
  // Master's native caller has already toggled its next-half selector.
  program.immediate("a", second_half);
  program.emit("move x:>" + number(phase_slot) + ",x0");
  program.emit("sub x0,a");
  program.emit("movep a1,x:<<" + number(source));
  put(program, DspSpace::x, destination, transmit);
  put(program, DspSpace::x, counter, word_count - 1);
  put(program, DspSpace::x, dma_control, single_block);
  enabled = program.pc();
  program.emit("rts");
}

void VoicePcm::begin(DspBus &bus) {
  edge = cycle = 0;
  block = words = frames = 0;
  receiver.reset();
  bus.observe_esai_transmit(serial, {.high_clock = false, .timing = timing});
  static_cast<void>(grant(bus));
}

void VoicePcm::baseline(DspBus &bus) {
  const auto clocks = bus.esai_clock_outputs(serial);
  require(clocks.driven == 3 && clocks.known == 3,
          "Own native setup must drive known clock/frame outputs");
  require(!receiver.observe(
              edge, {.clock = (clocks.levels & 1U) != 0, .frame_sync = (clocks.levels & 2U) != 0}),
          "A baseline cannot supply a decoded frame");
}

DspDmaBusResult VoicePcm::grant(DspBus &bus) {
  const auto result = bus.advance_dma(
      {.cycle = cycle++,
       .timing = DspDmaTiming::reference_sampled_cycles,
       .grant = DspDmaGrant::available},
      {.accesses = {}, .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy});
  require(!result.core_stall, "Paused core has no simultaneous memory occupancy");
  if (result.word) {
    require(words < transfers.size(), "No DMA word may escape the declared complete block");
    transfers.at(words++) = *result.word;
  }
  return result;
}

DspDmaBusResult VoicePcm::step(DspBus &bus) {
  require(edge < maximum_edges, "Explicit serial edge bound");
  ++edge;
  bus.observe_esai_transmit(serial, {.high_clock = edge % 2 != 0, .timing = timing});
  const auto clocks = bus.esai_clock_outputs(serial);
  const auto data = bus.esai_transmit_outputs(serial);
  require(clocks.driven == 3 && clocks.known == 3, "Actual ESAI clock outputs stay known");
  const auto frame = receiver.observe(edge,
                                      {.clock = (clocks.levels & 1U) != 0,
                                       .frame_sync = (clocks.levels & 2U) != 0,
                                       .data = (data.driven & data.known & 1U) != 0
                                                   ? std::optional{(data.levels & 1U) != 0}
                                                   : std::nullopt});
  if (frame) {
    require(frames < decoded.size(), "No extra serial frame may be silently dropped");
    decoded.at(frames++) = *frame;
  }
  return grant(bus);
}

void VoicePcm::finish(DspBus &bus, const std::uint64_t target) {
  while (receiver.completed_frames() < target) { static_cast<void>(step(bus)); }
  verify(bus);
}

void VoicePcm::verify(DspBus &bus) const {
  const auto start = block % 2 == 0 ? first_half : second_half;
  require(words == word_count && frames == frame_count && read(bus, source) == start + word_count &&
              read(bus, destination) == transmit && read(bus, counter) == word_count - 1 &&
              read(bus, dma_control) == (single_block & ~enable) &&
              read(bus, dma_status) == all_done,
          "Actual finite DMA must complete, reload its count and stop at the next half");
  require((bus.read({.space = DspSpace::y, .address = status}) & underrun) == 0,
          "Actual serial refills cannot hide an underrun");
  for (unsigned i = 0; i < word_count; ++i) {
    const auto &word = transfers.at(i);
    require(word.channel == 0 && word.source == DspDmaAddress{.address = start + i} &&
                word.destination == DspDmaAddress{.space = DspDmaSpace::y, .address = transmit},
            "Every native DMA transfer uses its actual master-output address and ESAI TX0");
  }
}

void VoicePcm::drain(DspBus &bus, const DspCore &cpu, std::ostream &trace) {
  const auto before = cpu.debug_state();
  words = frames = 0;
  const auto target = receiver.completed_frames() + frame_count;
  // Between blocks TDE is held high; service its genuine request before clocking
  // the next slot. First-block TDE stays low until the real priming-word launch.
  static_cast<void>(grant(bus));
  static_cast<void>(grant(bus));
  if (block == 0) {
    while (true) {
      const auto result = step(bus);
      if (result.phase && result.phase->kind == DspDmaPhaseKind::source_read) { break; }
    }
    require(words == 0, "Selected checkpoint retains the first held source word before TX write");
    const auto saved_bus = bus;
    const auto saved_wire = *this;
    finish(bus, target);
    const auto completed = *this;
    bus = saved_bus;
    *this = saved_wire;
    finish(bus, target);
    require(*this == completed, "Held-word restore reproduces every DMA word, edge and PCM frame");
    trace << "PCM_RESTORE 1\n";
  }
  finish(bus, target);
  require(cpu.debug_state() == before,
          "Standalone DMA/ESAI progression must leave the core paused");
  emit(trace);
  ++block;
}

void VoicePcm::emit(std::ostream &trace) const {
  trace << std::hex;
  for (unsigned i = 0; i < words; ++i) {
    const auto &word = transfers.at(i);
    trace << "PCM_DMA " << i << ' ' << word.source.address << ' ' << word.destination.address << ' '
          << word.value << '\n';
  }
  for (unsigned i = 0; i < frames; ++i) {
    const auto &frame = decoded.at(i);
    if (!frame.bit_ticks) { throw Fault("Decoded frame has no known bit period"); }
    require(frame.channels == 2 && frame.uniform_clock,
            "Every actual decoded frame retains both channels and a known uniform clock");
    trace << "PCM_FRAME " << i << ' ' << frame.first_tick << ' ' << frame.last_tick << ' '
          << *frame.bit_ticks;
    for (unsigned lane = 0; lane < 2; ++lane) {
      trace << ' ' << (static_cast<unsigned>(frame.samples.at(lane)) & word_mask);
    }
    trace << '\n';
  }
  trace << std::dec;
}
}
