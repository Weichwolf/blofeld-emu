// SPDX-License-Identifier: GPL-3.0-only
#include "dax_fixture.hpp"
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_bit_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::DspProgram;
using blofeld::test::number;
using blofeld::test::require;
namespace dma = blofeld::test::dma_test;
namespace dax = blofeld::test::dax_test;
namespace esai = blofeld::test::esai;
constexpr unsigned ring = 0x660, ring_words = 64, count = 63U << 6U;
constexpr unsigned row_rewind = 0xffffc1, first_sample = 0x010203, sample_step = 0x010101;
constexpr unsigned secondary = 0xffff80, transmit_clock = 0x640301, transmit_format = 0x7d00;
constexpr unsigned mux_e = 0x838, personal_reset = 0x80000;
constexpr unsigned repeat_control = 0xa8b600, esai_control = 0xaab604;
constexpr unsigned frame_edges = 512, slot_bits = 32;
constexpr unsigned bcr = 0xfffffb, port_bcr = 0x01ffff;
constexpr unsigned port_control = 0xfffff7, port_direction = 0xfffff6;
constexpr unsigned port_pins = 0x7ff;
constexpr std::uint64_t core_hz = 12000000, source_hz = 500000;
constexpr auto which = DspEsaiInstance::secondary;

unsigned payload(const unsigned index) {
  return first_sample + ((index % ring_words) * sample_step);
}

void observe(DspBus &bus, const bool level, const bool shared) {
  constexpr unsigned hckt_pad = 0x20;
  bus.gpio(DspPort::e).update_drive(hckt_pad, {.mask = hckt_pad, .levels = level ? hckt_pad : 0U});
  bus.observe_esai_transmit(
      which, {.high_clock = level, .timing = DspEsaiClockTiming::reference_reset_edges});
  if (shared) {
    bus.gpio(DspPort::d).update_drive(1, {.mask = 1, .levels = level ? 1U : 0U});
    bus.observe_dax({.aci = level,
                     .rates = {.core_hz = core_hz, .aci_hz = source_hz},
                     .timing = DspDaxTiming::reference_rising_edges});
  }
}

struct Input {
  unsigned alignment = 0, value = 0;
  bool direction = false;
};

struct Stream {
  std::uint64_t edge = 0, cycle = 0;
  bool sck = false, dax_level = true;
  std::string pcm, bmc;
  std::array<unsigned, 2> stores{};
  std::vector<DspDmaWord> writes;
};

void capture_esai(Stream &stream, DspMachine &machine) {
  const auto clocks = machine.bus().esai_clock_outputs(which);
  const bool sck = (clocks.levels & 1U) != 0;
  if (sck != stream.sck && !sck) {
    stream.pcm.push_back(esai::sample(machine.bus().esai_transmit_outputs(which), 0));
  }
  stream.sck = sck;
  require(stream.pcm.size() == (stream.edge + 1) / 8,
          "Actual generated falling SCKT edges follow the independent HCKT/4 divider");
}

unsigned install(DspMachine &machine, const Input input) {
  DspProgram code;
  unsigned instructions = 0;
  const auto put = [&](const DspStore store) {
    code.emit("movep #>" + number(store.value) + ',' +
              std::string(store.space == DspSpace::y ? "y" : "x") + ":<<" + number(store.address));
    ++instructions;
  };
  put({.space = DspSpace::y, .address = secondary + esai::control, .value = personal_reset});
  put({.space = DspSpace::y, .address = secondary + esai::clock, .value = transmit_clock});
  put({.space = DspSpace::y, .address = secondary + esai::control, .value = transmit_format});
  put({.space = DspSpace::y, .address = secondary});
  put({.space = DspSpace::y, .address = secondary + esai::control, .value = transmit_format | 1U});
  constexpr unsigned e_control = 31;
  constexpr unsigned e_direction = 30;
  constexpr unsigned e_data = 29;
  put({.space = DspSpace::y, .address = secondary + e_control, .value = mux_e});
  put({.space = DspSpace::y, .address = secondary + e_direction, .value = mux_e | 4U});
  put({.space = DspSpace::y, .address = secondary + e_data});
  put({.address = bcr, .value = port_bcr});
  put({.space = DspSpace::y, .address = port_control});
  put({.space = DspSpace::y, .address = port_direction});
  put({.address = dax::non_audio});
  put({.address = dax::audio_a});
  put({.address = dax::audio_a});
  put({.address = dax::control, .value = 8});
  put({.address = dax::port_control, .value = 3});
  put({.address = dax::port_direction, .value = 3});
  put({.address = dma::offset0, .value = 1});
  put({.address = dma::offset1, .value = row_rewind});
  for (const unsigned channel : {0U, 3U}) {
    const unsigned offset = channel * 4;
    put({.address = dma::counter0 - offset, .value = count});
    put({.address = dma::source0 - offset, .value = ring});
    put({.address = dma::destination0 - offset, .value = channel == 0 ? secondary : dax::audio_a});
  }
  put({.address = dma::control0, .value = esai_control});
  put({.address = dma::control0 - 12, .value = repeat_control});
  code.immediate("sr", test::peripheral_bit::status);
  ++instructions;
  constexpr unsigned first_source_cycle = 84;
  constexpr unsigned store_latency = 6;
  const auto nops = first_source_cycle + input.alignment - store_latency;
  for (unsigned index = 0; index < nops; ++index) { code.word(0); }
  // FM13 MOVEP immediate-to-Y:pp; address bits are relative to $ffffc0.
  constexpr unsigned immediate_y_pp = 0x09f480;
  constexpr unsigned pp_base = 0xffffc0;
  const unsigned address = input.direction ? port_direction : port_control;
  code.word(immediate_y_pp | (address - pp_base));
  code.word(input.value);
  constexpr unsigned jump_word = 0x0af080;
  const auto jump = test::boot + (2 * instructions) + nops + 2;
  code.word(jump_word);
  code.word(jump);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  for (unsigned index = 0; index < ring_words; ++index) {
    machine.bus().write({.space = DspSpace::x, .address = ring + index, .value = payload(index)});
  }
  machine.bus().observe_dax({.core_clock = false,
                             .aci = false,
                             .rates = {.core_hz = core_hz, .aci_hz = source_hz},
                             .timing = DspDaxTiming::reference_rising_edges});
  observe(machine.bus(), false, true);
  static_cast<void>(machine.advance_dma({.cycle = 0,
                                         .timing = DspDmaTiming::reference_sampled_cycles,
                                         .grant = DspDmaGrant::available}));
  for (unsigned index = 0; index < instructions; ++index) { machine.cpu().step(); }
  require(machine.cpu().debug_state().native_instructions == instructions &&
              machine.cpu().debug_state().native_cycles == std::uint64_t{2} * instructions,
          "Only genuine native MOVEP/SR seeds configure the two peripherals,Port F and DMA");
  for (const unsigned channel : {0U, 3U}) {
    const unsigned offset = channel * 4;
    require(machine.bus().read({.space = DspSpace::x, .address = dma::source0 - offset}) == ring &&
                machine.bus().read({.space = DspSpace::x, .address = dma::counter0 - offset}) ==
                    count &&
                machine.bus().read({.space = DspSpace::x, .address = dma::control0 - offset}) ==
                    (channel == 0 ? esai_control : repeat_control),
            "Actual native controller registers retain both independently encoded DRS22/ModeC "
            "configurations");
  }
  return jump - 2;
}

void serial(Stream &stream, DspMachine &machine) {
  capture_esai(stream, machine);
  const auto output = machine.bus().dax_output();
  require(output.driven, "Actual Port D mux connects DAX");
  if (stream.edge >= 3 && (stream.edge - 3) % 4 == 0) {
    stream.bmc.push_back(output.level ? '1' : '0');
  } else {
    require(output.level == stream.dax_level, "DAX retains its real biphase unit boundaries");
  }
  stream.dax_level = output.level;
}

void check_word(Stream &stream, DspMachine &machine, const DspDmaWord word) {
  const unsigned index = word.channel == 0 ? 0U : 1U;
  require(word.channel == (stream.writes.size() % 2 == 0 ? 0U : 3U),
          "Real priority and captured shared TDE requests alternate DMA0 and DMA3 writes");
  const auto count_before = stream.stores.at(index);
  require(
      word.source == DspDmaAddress{.address = ring + (count_before % ring_words)} &&
          word.destination ==
              DspDmaAddress{.space = word.channel == 0 ? DspDmaSpace::y : DspDmaSpace::x,
                            .address = word.channel == 0 ? secondary : dax::audio_a} &&
          word.value == payload(count_before),
      "Each real DMA read/write uses the independently specified source ring, space and sample");
  ++stream.stores.at(index);
  require(
      machine.bus().read({.space = DspSpace::x, .address = dma::source0 - (word.channel * 4)}) ==
          ring + (stream.stores.at(index) % ring_words),
      "ModeC advances and wraps the actual native source pointer independently for both channels");
  stream.writes.push_back(word);
}

void verify_wire(const Stream &stream) {
  require(!stream.pcm.empty() && !stream.bmc.empty(),
          "Actual clocks produce both serial streams before payload qualification");
  for (unsigned slot = 0; slot < stream.pcm.size() / slot_bits; ++slot) {
    const unsigned value = slot == 0 ? 0U : payload(slot - 1);
    const auto expected =
        std::bitset<24>(value).to_string() + std::string(8, (value & 1U) != 0 ? '1' : '0');
    require(
        stream.pcm.substr(std::size_t{slot} * slot_bits, slot_bits) == expected,
        "Independent ESAI receiver checks source words plus all eight repeated LSB padding bits");
  }
  for (unsigned slot = 0; slot < stream.bmc.size() / dax::subframe_units; ++slot) {
    char preamble = slot % 2 != 0 ? 'Y' : 'X';
    if (slot == 0) { preamble = 'Z'; }
    const unsigned value = slot < 2 ? 0U : payload(slot - 2);
    require(dax::decode(std::string_view(stream.bmc)
                            .substr(std::size_t{slot} * dax::subframe_units, dax::subframe_units),
                        preamble) == dax::Decoded{.audio = value, .flags = 0},
            "Independent BMC receiver checks payload, preamble, VUC and even parity including "
            "source-ring wraps");
  }
}

constexpr unsigned source_half_period = 12, first_request_cycle = 84;
constexpr unsigned native_seed_steps = 28;
constexpr unsigned program = test::boot + (native_seed_steps * 2);
constexpr unsigned target_frames = 4;
// FMRev5 tableA-1: JMP ea T3 plus one long-absolute addressing cycle.
constexpr unsigned long_absolute_jump_cost = 4;
constexpr unsigned finish_cycle = target_frames * frame_edges * source_half_period;

struct Oracle {
  DspDebugState cpu;
  unsigned store_address = 0, writes = 0, stores_retired = 0;
};

void retire(const DspFetchCycleResult &cycle, const Input input, Oracle &oracle) {
  const auto &phase = test::pipeline::stages(cycle);
  const auto expected_store = first_request_cycle + input.alignment;
  if (phase.retirement_published) {
    require(phase.execute2 && phase.execute2->address == oracle.cpu.cpu.pc,
            "Only the independent own NOP/MOVEP/self-JMP control flow retires");
    const auto pc = oracle.cpu.cpu.pc;
    const bool store = pc == oracle.store_address;
    const bool jump = pc == oracle.store_address + 2;
    require(jump || (pc >= program && pc <= oracle.store_address),
            "The core cannot escape the installed own program");
    if (!jump) { oracle.cpu.cpu.pc += store ? 2U : 1U; }
    if (store) {
      require(cycle.completed_cycles == expected_store + 1,
              "The native MOVEP retires at E2, after its real E1 write");
      ++oracle.stores_retired;
    }
    ++oracle.cpu.native_instructions;
    const unsigned normal_cost = store ? 2U : 1U;
    oracle.cpu.native_cycles += jump ? long_absolute_jump_cost : normal_cost;
  }
}

void core(const DspFetchCycleResult &cycle,
          DspMachine &machine,
          const Input input,
          Oracle &oracle) {
  const auto &phase = test::pipeline::stages(cycle);
  const auto expected_store = first_request_cycle + input.alignment;
  const unsigned address = input.direction ? port_direction : port_control;
  if (phase.memory) {
    require(cycle.completed_cycles == expected_store &&
                phase.memory->data.kind == DspNativeAccessKind::write &&
                phase.memory->data.pc == oracle.store_address &&
                phase.memory->data.target.space == DspSpace::y &&
                phase.memory->data.target.address == address &&
                phase.memory->data.value == input.value,
            "Actual Port-F E1 lands on the independently selected live DMA phase");
    ++oracle.writes;
  }
  require(!cycle.bus.bit_write, "Whole configuration words create no bit transaction");
  retire(cycle, input, oracle);
  require(machine.cpu().debug_state() == oracle.cpu &&
              machine.bus().read({.space = DspSpace::y, .address = address}) ==
                  (cycle.completed_cycles >= expected_store ? input.value : 0U),
          "Every period preserves full CPU/CCR/stack and the independent Port-F latch boundary");
  if (cycle.completed_cycles >= first_request_cycle &&
      cycle.completed_cycles < first_request_cycle + 4) {
    const auto offset = cycle.completed_cycles - first_request_cycle;
    require(cycle.bus.dma.phase && cycle.bus.dma.phase->channel == (offset < 2 ? 0U : 3U) &&
                cycle.bus.dma.phase->kind == (offset % 2 == 0 ? DspDmaPhaseKind::source_read
                                                              : DspDmaPhaseKind::destination_write),
            "Genuine shared TDE requests select source/write phases of DMA0 then DMA3");
  }
}

void tick(
    DspFetchRun &run, DspMachine &machine, Stream &stream, const Input input, Oracle &oracle) {
  ++stream.cycle;
  if (stream.cycle % source_half_period == 0) {
    ++stream.edge;
    observe(machine.bus(), stream.edge % 2 != 0, true);
    serial(stream, machine);
  }
  const auto cycle =
      run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
  require(cycle.completed_cycles == stream.cycle && !cycle.bus.dma.core_stall &&
              run.elapsed_ticks() == stream.cycle * test::pipeline::period,
          "The native12MHz clock and actual500kHz source share one exact integer reference grid");
  if (cycle.bus.dma.word) { check_word(stream, machine, *cycle.bus.dma.word); }
  core(cycle, machine, input, oracle);
}

void finish(
    DspFetchRun &run, DspMachine &machine, Stream &stream, const Input input, Oracle &oracle) {
  while (run.completed_cycles() < finish_cycle) { tick(run, machine, stream, input, oracle); }
  verify_wire(stream);
  constexpr unsigned expected_writes = target_frames * 4;
  require(stream.writes.size() == expected_writes && oracle.stores_retired == 1 &&
              oracle.writes == 1,
          "One genuine Port-F configuration write coexists with all real DRS22 ESAI/DAX transfers");
}

Oracle prepare(DspMachine &machine, const Input input) {
  const auto address = install(machine, input);
  static_cast<void>(machine.begin_reference_instruction(test::pipeline::execution));
  return {.cpu = machine.cpu().debug_state(), .store_address = address};
}

unsigned windows = 0, suffixes = 0, clones = 0;

void window(const Input input) {
  DspMachine machine;
  auto oracle = prepare(machine, input);
  Stream stream;
  DspFetchRun run(test::peripheral_bit::contract(), machine);
  finish(run, machine, stream, input, oracle);
  ++windows;
}

void snapshots(const Input input) {
  for (unsigned offset = 0; offset < 5; ++offset) {
    DspMachine machine;
    auto oracle = prepare(machine, input);
    Stream stream;
    DspFetchRun run(test::peripheral_bit::contract(), machine);
    const auto cut = first_request_cycle + offset - 1;
    while (run.completed_cycles() < cut) { tick(run, machine, stream, input, oracle); }
    const auto saved = run.checkpoint(machine);
    const auto prefix = stream;
    const auto state = oracle;
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      stream = prefix;
      oracle = state;
      finish(run, machine, stream, input, oracle);
      ++suffixes;
    }
    DspMachine clone;
    DspFetchRun cloned(saved, clone);
    stream = prefix;
    oracle = state;
    finish(cloned, clone, stream, input, oracle);
    ++clones;
  }
}

void checks(const unsigned alignment) {
  for (const bool direction : {false, true}) {
    for (const unsigned value : {0U, port_pins}) {
      window({.alignment = alignment, .value = value, .direction = direction});
    }
    if (alignment == 1) {
      snapshots({.alignment = alignment, .value = port_pins, .direction = direction});
    }
  }
}
}

int main() {
  try {
    for (unsigned alignment = 0; alignment < 4; ++alignment) { checks(alignment); }
    std::cout << "Native Port-F/DRS22 serial windows=" << windows << " suffixes=" << suffixes
              << " clones=" << clones << ";exact500kHz reference,independent ESAI/BMC\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
