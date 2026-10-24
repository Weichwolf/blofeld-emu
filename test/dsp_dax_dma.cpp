// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_dax.hpp"
#include "blofeld/dsp_dma.hpp"
#include "blofeld/dsp_machine.hpp"
#include "dax_fixture.hpp"
#include "dma_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test::dax_test {
namespace {
namespace transfer = blofeld::test::dma_test;
constexpr unsigned source = 0x700;
constexpr unsigned initial_non_audio = 0x8400;
constexpr unsigned first_non_audio = 0x5400;
constexpr unsigned second_non_audio = 0xc800;
constexpr unsigned a1 = 0x010203;
constexpr unsigned b1 = 0x0a0b0c;
constexpr unsigned a2 = 0xffffff;
constexpr unsigned b2 = 0;
constexpr unsigned line_control = 0x905050; // DTM2,DRS10,source linear,destination DOR0 2-D.
constexpr unsigned pair_count = 0x1001;
constexpr unsigned triple_count = 0x1002;
constexpr unsigned back_pair = 0xffffff;
constexpr unsigned back_triple = 0xfffffe;

unsigned program(DspProgram &code, const bool with_non_audio) {
  unsigned count = 0;
  const auto put = [&](const DspStore access) {
    code.emit("movep #>" + number(access.value) + ",x:<<" + number(access.address));
    ++count;
  };
  put({.address = non_audio, .value = initial_non_audio});
  put({.address = audio_a, .value = first_sample});
  put({.address = audio_b, .value = other_sample});
  put({.address = control, .value = 8});
  put({.address = port_direction, .value = 3});
  put({.address = port_control, .value = 3});
  put({.address = transfer::offset0, .value = with_non_audio ? back_triple : back_pair});
  put({.address = transfer::source0, .value = source});
  put({.address = transfer::destination0, .value = with_non_audio ? non_audio : audio_a});
  put({.address = transfer::counter0, .value = with_non_audio ? triple_count : pair_count});
  put({.address = transfer::control0, .value = line_control});
  code.emit("nop");
  code.emit("movep x:<<$ffffef,x0");
  code.emit("movep x:<<$ffffed,y0");
  code.emit("jset #0,x:<<$ffffd4," + number(code.pc()));
  return count;
}

struct Stream {
  std::uint64_t edge = 1;
  std::string wire;
  std::vector<DspDmaWord> writes;

  void tick(DspMachine &machine) {
    machine.observe_dax(inputs(edge % 2 != 0, 1));
    if (edge % 4 == 3) {
      const auto pin = machine.bus().dax_output();
      require(pin.driven, "Native Port D mux connects the real DAX output");
      wire.push_back(pin.level ? '1' : '0');
    }
    const std::array declared_fetch{
        DspMemoryAccess{.target = {.address = machine.cpu().state().pc}},
        DspMemoryAccess{.target = {.space = DspSpace::x, .address = transfer::source0}}};
    const auto progress = machine.advance_dma(
        {.cycle = edge,
         .timing = DspDmaTiming::reference_sampled_cycles,
         .grant = DspDmaGrant::available},
        {.accesses = declared_fetch, .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy});
    require(!progress.core_stall,
            "Declared P-ROM occupancy cannot block an independent DAX data-register write");
    if (progress.word) { writes.push_back(*progress.word); }
    ++edge;
  }
};

void verify_wire(const std::string_view wire, const bool with_non_audio) {
  constexpr std::array samples{Decoded{.audio = first_sample, .flags = 1},
                               Decoded{.audio = other_sample, .flags = 4},
                               Decoded{.audio = a1, .flags = 5},
                               Decoded{.audio = b1, .flags = 2},
                               Decoded{.audio = a2, .flags = 2},
                               Decoded{.audio = b2, .flags = 6}};
  for (std::size_t channel = 0; channel < samples.size(); ++channel) {
    char preamble = channel % 2 != 0 ? 'Y' : 'X';
    if (channel == 0) { preamble = 'Z'; }
    auto expected = samples.at(channel);
    if (!with_non_audio) { expected.flags = channel % 2 == 0 ? 1U : 4U; }
    require(decode(wire.substr(channel * subframe_units, subframe_units), preamble) == expected,
            "The independent receiver checks six literal native/DMA payloads and per-frame VUC");
  }
}

void native_line_stream(const bool with_non_audio) {
  constexpr std::array triples{first_non_audio, a1, b1, second_non_audio, a2, b2};
  constexpr std::array pairs{a1, b1, a2, b2};
  const auto values =
      with_non_audio ? std::span<const unsigned>{triples} : std::span<const unsigned>{pairs};
  DspMachine machine;
  DspProgram code;
  const auto instructions = program(code, with_non_audio);
  code.install(machine.bus());
  for (std::size_t i = 0; i < values.size(); ++i) {
    machine.bus().write(
        {.space = DspSpace::x, .address = source + static_cast<unsigned>(i), .value = values[i]});
  }
  machine.cpu().reset_spi_boot();
  machine.observe_dax(inputs(false, 1));
  static_cast<void>(machine.advance_dma({.cycle = 0,
                                         .timing = DspDmaTiming::reference_sampled_cycles,
                                         .grant = DspDmaGrant::available}));
  for (unsigned i = 0; i < instructions; ++i) { machine.cpu().step(); }
  Stream stream;
  while (stream.edge < 4) { stream.tick(machine); }
  require(stream.wire == "0" && stream.writes.empty() &&
              machine.bus().read({.space = DspSpace::x, .address = transfer::status}) ==
                  transfer::zero_active,
          "The first real XADE captured a DMA source but has not stored its destination");
  machine.cpu().step();
  machine.bus().write({.space = DspSpace::x, .address = source, .value = 0});
  const auto frozen = machine.checkpoint();
  const auto saved = stream;
  const auto cpu = machine.cpu().debug_state();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(frozen);
    stream = saved;
    constexpr std::size_t expected_units = std::size_t{3} * frame_units;
    constexpr unsigned edge_limit = 2000;
    while (stream.wire.size() < expected_units && stream.edge < edge_limit) {
      stream.tick(machine);
    }
    require(stream.wire.size() == expected_units && stream.writes.size() == values.size(),
            "Two real DMA lines refill the actual DAX buffers within a fixed independent bound");
    verify_wire(stream.wire, with_non_audio);
    const unsigned registers = with_non_audio ? 3U : 2U;
    const auto base = with_non_audio ? non_audio : audio_a;
    for (unsigned i = 0; i < stream.writes.size(); ++i) {
      const auto &word = stream.writes.at(i);
      require(word.channel == 0 && word.source == DspDmaAddress{.address = source + i} &&
                  word.destination == DspDmaAddress{.address = base + (i % registers)} &&
                  word.value == values[i],
              "Every actual source/write matches the manual2/3-register line and held read value");
    }
    constexpr unsigned native_prefix = 23;
    require(machine.cpu().debug_state() == cpu && cpu.native_cycles == native_prefix,
            "Clock observations and DMA phases execute no hidden native instructions");
    machine.cpu().step();
    machine.cpu().step();
    require(machine.cpu().state().x == source + values.size() &&
                machine.cpu().state().y == (with_non_audio ? triple_count : pair_count) &&
                machine.bus().read({.space = DspSpace::x, .address = transfer::status}) ==
                    transfer::all_done,
            "Native pointer/count reads see the completed block and its exact reload");
    const auto polling = machine.cpu().state().pc;
    machine.cpu().step();
    require(machine.cpu().state().pc == polling,
            "Native masked XADE polling excludes the slave XAUR erratum bit");
  }
}

void line_boundaries() {
  constexpr unsigned external_line = 0x900054; // IRQ A,linearX->two-dimensionalY.
  transfer::Harness test;
  static_cast<void>(test.tick());
  test.dma.write(transfer::offset0, back_pair);
  transfer::configure(test.dma, external_line, pair_count);
  constexpr std::array destinations{0x900U, 0x901U, 0x900U, 0x901U};
  for (unsigned word = 0; word < destinations.size(); ++word) {
    if (word == 2) {
      static_cast<void>(test.tick(1));
      require(test.reads.size() == 2, "A held request cannot start another line after DCOL reload");
      static_cast<void>(test.tick());
    }
    static_cast<void>(test.tick(1));
    const auto result = test.tick(1);
    require(result && result->source.address == transfer::ring_start + word &&
                result->destination.address == destinations.at(word),
            "Each new edge triggers exactly one two-word line, including DOR destination rewind");
  }
  require(test.dma.read(transfer::counter0) == pair_count &&
              test.dma.read(transfer::control0) == (external_line & ~transfer::enable),
          "The second line completes the actual block and clears DE");
  for (const bool started : {false, true}) {
    transfer::Harness stopped;
    static_cast<void>(stopped.tick());
    stopped.dma.write(transfer::offset0, back_pair);
    transfer::configure(stopped.dma, external_line, pair_count);
    static_cast<void>(stopped.tick(1, !started));
    stopped.dma.write(transfer::control0, external_line & ~transfer::enable);
    const auto result = stopped.tick();
    require(started ? result.has_value() : !result,
            "APR23 DE clear finishes only an already-started line word, not an unstarted line");
    static_cast<void>(stopped.tick());
    require(stopped.reads.size() == (started ? 1U : 0U) && !stopped.dma.active(),
            "A cancelled line cannot quietly continue its remaining words");
  }
}
}

void dma_checks() {
  line_boundaries();
  native_line_stream(false);
  native_line_stream(true);
}
}
