// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_fixture.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "esai_clock_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace blofeld::test::dma_test {
namespace {
constexpr unsigned primary_base = 0xffffa0;
constexpr unsigned secondary_base = 0xffff80;
constexpr unsigned mux = 0x838;
constexpr unsigned dax = 0xffffd2;
constexpr unsigned first_value = 0x10000;
constexpr unsigned total_words = 66;
constexpr unsigned port_control = 31;
constexpr unsigned port_direction = 30;

struct StatusBitCase {
  unsigned channel;
  unsigned bit;
};

void native_masked_status_case(const StatusBitCase input) {
  constexpr unsigned channels = 6;
  const auto [channel, bit] = input;
  const auto register_offset = channel * 4;
  OwnDspMachine machine(native_clocks);
  NativeProgram code;
  for (const auto [address, value] : {std::pair{source0, ring_start},
                                      std::pair{destination0, destination},
                                      std::pair{counter0, 0U},
                                      std::pair{control0, word_copy}}) {
    code.write({.space = DspSpace::x, .address = address - register_offset, .value = value});
  }
  // FM MOVEC SR seeds the opposite Carry; high-pp BTST selects DSTR's real bit.
  constexpr unsigned move_sr = 0x05f439;
  constexpr unsigned masked_sr = 0xc00300;
  constexpr unsigned btst_status = 0x0bb420;
  code.word(move_sr);
  code.word(masked_sr | (bit < channels ? 0U : 1U));
  code.word(btst_status | bit);
  reset_native(machine, code);
  for (unsigned step = 0; step < 5; ++step) { step_native(machine); }
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const bool unknown = bit == channel || (bit >= 8 && bit <= 11);
  if (unknown) {
    const auto error = rejects([&] { step_native(machine); });
    require(error.contains("cycle timing") && error.contains("pc=" + hex(before.pc, 6)) &&
                error.contains("omr=" + hex(before.omr, 6)) &&
                error.contains("mask=" + hex(1U << bit, 6)) &&
                machine.cpu().registers() == before &&
                machine.cpu().progress().instructions == progress.instructions,
            "Native selected unknown DMA fields fail with exact CPU/register/mask context");
    const auto stopped = machine.cpu().registers();
    const auto stopped_progress = machine.cpu().progress();
    require(!rejects([&] { step_native(machine); }).empty() && machine.io().failure() &&
                machine.cpu().registers() == stopped &&
                machine.cpu().progress() == stopped_progress,
            "Rejected unknown status cannot resume or change native state without recovery");
  } else {
    step_native(machine);
    auto expected = before;
    ++expected.pc;
    expected.sr = (expected.sr & ~1U) | (bit < channels ? 1U : 0U);
    require(machine.cpu().registers() == expected &&
                machine.cpu().progress().instructions == progress.instructions + 1,
            "FM BTST changes only Carry and PC for an independent known DTD or reserved bit");
  }
  require(machine.bus().read({.space = DspSpace::x, .address = control0 - register_offset}) ==
                  word_copy &&
              machine.bus().read({.space = DspSpace::x, .address = counter0 - register_offset}) ==
                  0,
          "Status bit access never alters the real enabled channel or counter");
}

void native_masked_status_without_time() {
  constexpr unsigned bits = 24;
  for (unsigned channel = 0; channel < 6; ++channel) {
    for (unsigned bit = 0; bit < bits; ++bit) {
      native_masked_status_case({.channel = channel, .bit = bit});
    }
  }
}

unsigned program(NativeProgram &code, const bool second, const bool with_dax) {
  unsigned count = 0;
  const auto put = [&](const DspSpace space, const unsigned address, const unsigned value) {
    code.write({.space = space, .address = address, .value = value});
    ++count;
  };
  const auto base = second ? secondary_base : primary_base;
  const auto space = second ? DspSpace::y : DspSpace::x;
  put(space, base + esai::control, esai_clock::word32);
  put(space, base + esai::clock, esai_clock::base_clock | esai_clock::clock_polarity);
  put(space, base + port_control, mux);
  put(space, base + port_direction, mux);
  put(space, base, esai::other);
  put(space, base + esai::control, esai_clock::word32 | 1U);
  put(DspSpace::x, offset0, 1);
  put(DspSpace::x, offset1, ring_back);
  put(DspSpace::x, source0, ring_start);
  put(DspSpace::x, destination0, base);
  put(DspSpace::x, counter0, ring_count);
  constexpr unsigned primary_ring = 0xa86600;
  constexpr unsigned priority1 = 0x20000;
  put(DspSpace::x, control0, (second ? repeat_ring : primary_ring) | priority1);
  if (with_dax) {
    put(DspSpace::x, source0 - 12, ring_start);
    put(DspSpace::x, destination0 - 12, dax);
    put(DspSpace::x, counter0 - 12, 0);
    constexpr unsigned secondary_one_word = 0x88b240;
    put(DspSpace::x, control0 - 12, secondary_one_word);
  }
  // FM high-pp MOVEP loads DSAR0->X0/DCO0->Y0, then JCLR DACT back to itself.
  constexpr unsigned source_x0 = 0x08442f;
  constexpr unsigned count_y0 = 0x08462d;
  constexpr unsigned poll_dact = 0x0ab488;
  code.word(0);
  code.word(source_x0);
  code.word(count_y0);
  const auto poll = code.pc();
  code.word(poll_dact);
  code.word(poll);
  return count;
}

void initialize(OwnDspMachine &machine,
                const DspEsaiInstance instance,
                const NativeProgram &code,
                const unsigned instructions) {
  reset_native(machine, code);
  for (unsigned address = ring_start; address <= ring_last; ++address) {
    machine.bus().write(
        {.space = DspSpace::x, .address = address, .value = first_value + address - ring_start});
  }
  machine.bus().observe_esai_transmit(instance,
                                      {.high_clock = false, .timing = esai_clock::timing});
  static_cast<void>(dma(machine, {.cycle = 0, .timing = timing, .grant = grant}));
  for (unsigned i = 0; i < instructions; ++i) { step_native(machine); }
}

struct Stream {
  std::uint64_t edge = 1;
  std::string bits;
  std::vector<DspDmaWord> transfers;

  void step(OwnDspMachine &machine, const DspEsaiInstance instance) {
    const auto old = machine.bus().esai_clock_outputs(instance);
    machine.bus().observe_esai_transmit(
        instance, {.high_clock = edge % 2 != 0, .timing = esai_clock::timing});
    const auto clocks = machine.bus().esai_clock_outputs(instance);
    if ((old.levels & 1U) == 0 && (clocks.levels & 1U) != 0) {
      const auto data = machine.bus().esai_transmit_outputs(instance);
      if (data.driven != 0) { bits.push_back(esai::sample(data, 0)); }
    }
    const std::array declared_fetch{
        DspMemoryAccess{.target = {.address = machine.cpu().registers().pc}},
        DspMemoryAccess{.target = {.space = DspSpace::x, .address = source0}}};
    const auto progress = dma(
        machine,
        {.cycle = edge, .timing = timing, .grant = grant},
        {.accesses = declared_fetch, .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy});
    require(!progress.core_stall,
            "Declared P-ROM occupancy is independent of the actual ESAI/DAX destination");
    if (progress.word) { transfers.push_back(*progress.word); }
    ++edge;
  }
};

std::string expected_stream() {
  std::string result = "00010010001101000101011000000000";
  constexpr unsigned payload_bits = 24;
  constexpr unsigned padding = 8;
  constexpr unsigned ring_words = 64;
  for (unsigned word = 0; word < total_words - 1; ++word) {
    const auto value = first_value + (word % ring_words);
    for (unsigned bit = payload_bits; bit != 0; --bit) {
      result.push_back((value & (1U << (bit - 1))) != 0 ? '1' : '0');
    }
    result.append(padding, (value & 1U) != 0 ? '1' : '0');
  }
  return result;
}

void serial_refill(const bool second) {
  const auto instance = second ? DspEsaiInstance::secondary : DspEsaiInstance::primary;
  OwnDspMachine machine(native_clocks);
  NativeProgram code;
  const auto count = program(code, second, false);
  initialize(machine, instance, code, count);
  Stream stream;
  // TCKP1 launches at HCKT half-edge7; DMA reads there and writes at the next edge.
  constexpr unsigned after_source_read = 8;
  while (stream.edge < after_source_read) { stream.step(machine, instance); }
  require(stream.transfers.empty() && stream.bits.empty() &&
              machine.bus().read({.space = DspSpace::x, .address = status}) == zero_active,
          "The first real TDE edge has read DMA0 but has not fabricated a destination write");
  step_native(machine); // A real native NOP does not clock the standalone DMA model.
  const auto checkpoint = machine.checkpoint();
  const auto saved = stream;
  const auto expected = expected_stream();
  const auto base = second ? secondary_base : primary_base;
  const auto space = second ? DspDmaSpace::y : DspDmaSpace::x;
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(checkpoint);
    stream = saved;
    constexpr std::size_t width = 32;
    constexpr unsigned edge_limit = 20000;
    while (stream.bits.size() < total_words * width && stream.edge < edge_limit) {
      stream.step(machine, instance);
    }
    require(stream.bits == expected && stream.transfers.size() == total_words,
            "Both full native-owner restores reproduce independently expected66 serial words");
    constexpr unsigned ring_words = 64;
    for (unsigned i = 0; i < stream.transfers.size(); ++i) {
      const auto &word = stream.transfers.at(i);
      require(word.channel == 0 &&
                  word.source == DspDmaAddress{.address = ring_start + (i % ring_words)} &&
                  word.destination == DspDmaAddress{.space = space, .address = base} &&
                  word.value == first_value + (i % ring_words),
              "Every actual RAM read/TX write follows the64-word source circle and correct map");
    }
    require(machine.bus().read(
                {.space = second ? DspSpace::y : DspSpace::x, .address = base + esai::status}) == 0,
            "Timely real DMA refills prevent both underrun and fabricated empty flags");
    step_native(machine);
    step_native(machine);
    constexpr unsigned remaining_counter = 0xf40;
    require(machine.cpu().registers().x == ring_start + 2 &&
                machine.cpu().registers().y == remaining_counter,
            "Actual native MOVEP reads see the literal updated pointer and counter");
    const auto poll = machine.cpu().registers().pc;
    step_native(machine);
    require(machine.cpu().registers().pc == poll,
            "Actual native masked DACT polling does not invent or read unknown repeating DTD");
  }
}

void dax_fifo_failure() {
  OwnDspMachine machine(native_clocks);
  NativeProgram code;
  const auto count = program(code, true, true);
  initialize(machine, DspEsaiInstance::secondary, code, count);
  // Actual disabled DAX storage: A, B, then the single pending XADR word.
  for (unsigned value = 0; value < 3; ++value) {
    machine.bus().write({.space = DspSpace::x, .address = dax, .value = first_value + value});
  }
  Stream stream;
  constexpr unsigned before_dax_write = 10;
  while (stream.edge < before_dax_write) { stream.step(machine, DspEsaiInstance::secondary); }
  require(stream.transfers.size() == 1 && stream.transfers[0].channel == 0,
          "The higher-priority real ESAI refill completes before DMA3");
  const auto error = rejects([&] {
    static_cast<void>(dma(machine, {.cycle = stream.edge, .timing = timing, .grant = grant}));
  });
  require(error.contains("channel=3") && error.contains("0xffffd2") &&
              error.contains("pc=" + hex(machine.cpu().registers().pc, 6)),
          "A latched same-edge DMA3 request survives TDE clear and visibly faults on full XADR");
  const auto again = rejects([&] {
    static_cast<void>(dma(machine, {.cycle = stream.edge, .timing = timing, .grant = grant}));
  });
  require(error == again &&
              machine.bus().read({.space = DspSpace::x, .address = source0 - 12}) == ring_start,
          "The rejected DAX destination retains the held word and exact same failing cycle");
}

void automatic_dma_arbitration() {
  OwnDspMachine machine(native_clocks);
  NativeProgram code;
  constexpr unsigned words = 3;
  constexpr unsigned payload = 0x123450;
  constexpr unsigned sentinel = 0x654321;
  for (const auto [address, value] : {std::pair{source0, ring_start},
                                      std::pair{destination0, destination},
                                      std::pair{counter0, words - 1},
                                      std::pair{control0, software_copy}}) {
    code.write({.space = DspSpace::x, .address = address, .value = value});
  }
  constexpr unsigned foreground_words = 64;
  for (unsigned i = 0; i < foreground_words; ++i) { code.word(0); }
  reset_native(machine, code);
  for (unsigned i = 0; i < words; ++i) {
    machine.bus().write({.space = DspSpace::x, .address = ring_start + i, .value = payload + i});
    machine.bus().write({.space = DspSpace::y, .address = destination + i, .value = sentinel});
  }
  for (unsigned i = 0; i < 4; ++i) { step_native(machine); }
  const auto before = machine.cpu().registers();
  const auto progress = machine.cpu().progress();
  const auto ticks = machine.clock().elapsed_ticks();
  const auto error = rejects([&] { static_cast<void>(machine.advance_reference_cycle()); });
  require(error.contains("DMA arbitration") && error.contains("X:0xfffff4") &&
              error.contains("pc=" + hex(before.pc, 6)) && machine.cpu().registers() == before &&
              machine.cpu().progress() == progress && machine.clock().elapsed_ticks() == ticks &&
              !machine.bus().last_dma_cycle(),
          "A bare reference clock cannot bypass actual DMA arbitration or consume a period");
  OwnDspFetch fetch(machine);
  std::vector<DspDmaWord> transfers;
  constexpr unsigned bound = 128;
  for (unsigned cycle = 0; cycle < bound; ++cycle) {
    const auto actual = fetch.advance_dma_automatic(
        {.cycle = cycle, .known_requests = 15, .timing = timing, .grant = grant},
        OwnDspAutomaticTiming::one_effect_per_cycle);
    require(!actual.native || actual.native->fault.kind == dsp56300::FaultKind::none,
            "Actual native foreground remains healthy during automatic DMA arbitration");
    if (actual.dma.word) { transfers.push_back(*actual.dma.word); }
    if (transfers.size() == words &&
        machine.cpu().progress().instructions > progress.instructions) {
      for (unsigned i = 0; i < words; ++i) {
        const auto &word = transfers.at(i);
        require(
            word.channel == 0 && word.source == DspDmaAddress{.address = ring_start + i} &&
                word.destination ==
                    DspDmaAddress{.space = DspDmaSpace::y, .address = destination + i} &&
                word.value == payload + i &&
                machine.bus().read({.space = DspSpace::y, .address = destination + i}) ==
                    payload + i,
            "Every automatic DMA read/write and native RAM endpoint matches literal expectations");
      }
      require(machine.bus().read({.space = DspSpace::x, .address = control0}) ==
                      (software_copy & ~enable) &&
                  machine.clock().elapsed_ticks() > ticks,
              "Actual arbitration clears DE after three words and advances real reference periods");
      return;
    }
  }
  throw Fault("Automatic native DMA copy exceeded its bounded reference periods");
}

void memory_failures() {
  constexpr unsigned source_address = 0x200;
  constexpr unsigned value = 0x123456;
  constexpr unsigned sentinel = 0x654321;
  constexpr unsigned protected_rom = 0x4000;
  constexpr unsigned gpio_data = 0xffff9d;
  for (const auto target : {destination, protected_rom, gpio_data}) {
    OwnDspMachine machine(native_clocks);
    NativeProgram code;
    code.word(0);
    reset_native(machine, code);
    static_cast<void>(
        dma(machine, {.cycle = 0, .known_requests = 1, .timing = timing, .grant = grant}));
    for (const auto [address, data] : {std::pair{source0, source_address},
                                       std::pair{destination0, target},
                                       std::pair{counter0, 0U},
                                       std::pair{control0, word_copy}}) {
      machine.bus().write({.space = DspSpace::x, .address = address, .value = data});
    }
    const DspDmaInputs read_cycle{
        .cycle = 1, .known_requests = 1, .requests = 1, .timing = timing, .grant = grant};
    require(rejects([&] {
              static_cast<void>(dma(machine, read_cycle));
            }).contains("Unknown DSP memory read at X:0x000200"),
            "Unknown real RAM never becomes a zero DMA source or consumes the request");
    machine.bus().write({.space = DspSpace::x, .address = source_address, .value = value});
    static_cast<void>(dma(machine, read_cycle));
    machine.bus().write({.space = DspSpace::x, .address = source_address, .value = sentinel});
    const DspDmaInputs write_cycle{
        .cycle = 2, .known_requests = 1, .requests = 1, .timing = timing, .grant = grant};
    if (target == destination) {
      const auto word = dma(machine, write_cycle);
      require(word && word->value == value &&
                  machine.bus().read({.space = DspSpace::y, .address = target}) == value,
              "The real destination receives the captured value despite later source mutation");
    } else {
      const auto error = rejects([&] { static_cast<void>(dma(machine, write_cycle)); });
      require(error.contains(target == protected_rom ? "Protected DSP ROM" : "peripheral write") &&
                  error.contains("pc=0xff0000") && error.contains("cycle=2") &&
                  machine.bus().read({.space = DspSpace::x, .address = control0}) == word_copy &&
                  machine.bus().read({.space = DspSpace::x, .address = counter0}) == 0,
              "Protected ROM and forbidden GPIO fail with native context before committing a word");
    }
  }
}

void dax_data_map() {
  // DSP56371UM table3-7 and10.5.1/10.5.4: three data destinations, no DMA control.
  constexpr unsigned control = 0xffffd0;
  constexpr unsigned non_audio = 0xffffd1;
  constexpr unsigned audio_a = 0xffffd2;
  constexpr unsigned audio_b = 0xffffd3;
  constexpr unsigned dax_status = 0xffffd4;
  constexpr unsigned x_word_copy = 0x880240;
  for (const auto target : {control, non_audio, audio_a, audio_b, dax_status}) {
    OwnDspMachine machine(native_clocks);
    NativeProgram code;
    code.word(0);
    reset_native(machine, code);
    static_cast<void>(
        dma(machine, {.cycle = 0, .known_requests = 1, .timing = timing, .grant = grant}));
    constexpr unsigned payload = 0x400; // Valid XNADR fields and a known audio word.
    machine.bus().write({.space = DspSpace::x, .address = ring_start, .value = payload});
    for (const auto [address, value] : {std::pair{source0, ring_start},
                                        std::pair{destination0, target},
                                        std::pair{counter0, 0U},
                                        std::pair{control0, x_word_copy}}) {
      machine.bus().write({.space = DspSpace::x, .address = address, .value = value});
    }
    static_cast<void>(
        dma(machine,
            {.cycle = 1, .known_requests = 1, .requests = 1, .timing = timing, .grant = grant}));
    const bool data_register = target >= non_audio && target <= audio_b;
    const DspDmaInputs write_cycle{
        .cycle = 2, .known_requests = 1, .requests = 1, .timing = timing, .grant = grant};
    if (data_register) {
      const auto word = dma(machine, write_cycle);
      require(word && word->value == payload &&
                  machine.bus().read({.space = DspSpace::x, .address = control0}) ==
                      (x_word_copy & ~enable),
              "Both XADR aliases and XNADR accept their actual data-register transfer");
    } else {
      const auto error = rejects([&] { static_cast<void>(dma(machine, write_cycle)); });
      require(error.contains("Unsupported DSP DMA peripheral write") &&
                  error.contains(hex(target, 6)) &&
                  machine.bus().read({.space = DspSpace::x, .address = control0}) == x_word_copy,
              "DMA cannot write DAX status or configure its control register");
    }
  }
}
}

void native_checks() {
  native_masked_status_without_time();
  serial_refill(false);
  serial_refill(true);
  dax_fifo_failure();
  automatic_dma_arbitration();
  memory_failures();
  dax_data_map();
}
}
