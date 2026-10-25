// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_audio_dma_fixture.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::audio_dma {
namespace {
const DspRecord &component(const Firmware &firmware) {
  for (const auto &record : firmware.dsp) {
    if (record.space == 0 && record.address == component_address) { return record; }
  }
  throw Fault("Missing complete original extended-services program record");
}

void put(DspProgram &program, const DspSpace space, const unsigned address, const unsigned value) {
  program.emit("movep #>" + number(value) + (space == DspSpace::x ? ",x:<<" : ",y:<<") +
               number(address));
}
}

unsigned sample(const unsigned index) {
  require(index < ring_size, "Own ring-oracle index must remain within its64 declared words");
  constexpr unsigned corner_count = 8;
  constexpr unsigned stride = 0x10203;
  constexpr std::array corners{
      0x800001U, 0xffffffU, 0U, 0x7fffffU, 0x800000U, 1U, 0x654321U, 0xabcdefU};
  return corners.at(index % corner_count) ^ ((index / corner_count) * stride);
}

unsigned expected_word(const unsigned channel, const unsigned ordinal, const bool changed) {
  const auto index = ordinal % ring_size;
  if (changed && index == 0 && (channel == 3 || ordinal != 0)) { return replacement; }
  return sample(index);
}

std::int32_t signed_word(const unsigned word) {
  constexpr std::int64_t sign = 0x800000;
  constexpr std::int64_t modulus = 0x1000000;
  const auto value = static_cast<std::int64_t>(word);
  return static_cast<std::int32_t>(value < sign ? value : value - modulus);
}

Experiment::Experiment(const Firmware &firmware) : record_(component(firmware)) {
  auto &bus = machine.bus();
  constexpr unsigned omr = 0x600389;
  bus.observe_omr(omr);
  bus.advance_instruction_cycles(3);
  for (unsigned i = 0; i < record_.words.size(); ++i) {
    bus.write({.address = record_.address + i, .value = record_.words[i]});
  }
  for (unsigned i = 0; i < ring_size; ++i) {
    bus.write({.space = DspSpace::x, .address = ring_start + i, .value = sample(i)});
  }
  for (const auto address : {ring_start - 1, ring_start + ring_size}) {
    bus.write({.space = DspSpace::x, .address = address, .value = guard});
  }
  configure();
  paused_ = machine.cpu().debug_state();
  grant(); // Explicit known-low TDE baseline before any supplied serial edge.
  const auto clock = bus.esai_clock_outputs(DspEsaiInstance::secondary);
  static_cast<void>(driver.pcm.observe(
      0, {.clock = (clock.levels & 1U) != 0, .frame_sync = (clock.levels & 2U) != 0}));
}

void Experiment::configure() {
  struct Register {
    DspSpace space;
    unsigned address;
    unsigned value;
  };
  using enum DspSpace;
  // Own preconditions match the published register layout; no reserved write is supplied.
  constexpr std::array preloads{Register{.space = y, .address = 0xffff95, .value = 0x80000},
                                Register{.space = y, .address = 0xffff9d, .value = 0x202},
                                Register{.space = y, .address = 0xffff9e, .value = 0xebc},
                                Register{.space = y, .address = 0xffff9f, .value = 0x83b},
                                Register{.space = x, .address = 0xfffff3, .value = 1},
                                Register{.space = x, .address = 0xfffff2, .value = 0xffffc1},
                                Register{.space = x, .address = 0xfffff1, .value = 1},
                                Register{.space = x, .address = 0xfffff0, .value = 0xffffc1},
                                Register{.space = x, .address = source0, .value = ring_start},
                                Register{.space = x, .address = 0xffffee, .value = esai_tx},
                                Register{.space = x, .address = 0xffffed, .value = 0xfc0},
                                Register{.space = x, .address = source3, .value = ring_start},
                                Register{.space = x, .address = 0xffffe2, .value = dax_tx},
                                Register{.space = x, .address = 0xffffe1, .value = 0xfc0},
                                Register{.space = y, .address = 0xffff96, .value = 0x640301},
                                Register{.space = y, .address = 0xffff95, .value = 0x7d00},
                                Register{.space = y, .address = esai_tx, .value = 0},
                                Register{.space = y, .address = 0xffff95, .value = 0x7d01},
                                Register{.space = x, .address = 0xffffd1, .value = 0},
                                Register{.space = x, .address = dax_tx, .value = 0},
                                Register{.space = x, .address = dax_tx, .value = 0},
                                Register{.space = x, .address = 0xffffd0, .value = 8},
                                Register{.space = x, .address = 0xffffd7, .value = 3},
                                Register{.space = x, .address = 0xffffd6, .value = 3}};
  constexpr unsigned omr = 0x600389;
  DspProgram caller;
  caller.immediate("omr", omr);
  for (unsigned n = 0; n < 3; ++n) { caller.emit("nop"); }
  caller.immediate("sr", masked_status);
  for (const auto reg : preloads) { put(caller, reg.space, reg.address, reg.value); }
  caller.immediate("r6", enable_entry);
  caller.emit("jsr (r6)");
  caller.install(machine.bus());
  machine.cpu().reset_spi_boot();
  machine.bus().observe_esai_transmit(
      DspEsaiInstance::secondary,
      {.high_clock = false, .timing = DspEsaiClockTiming::reference_reset_edges});
  machine.observe_dax({.core_clock = false,
                       .aci = false,
                       .rates = {.core_hz = reference_core_hz, .aci_hz = reference_aci_hz},
                       .timing = DspDaxTiming::reference_rising_edges});
  unsigned steps = 0;
  while (machine.cpu().state().pc != enable_entry) {
    require(steps++ < 64, "Own native configuration caller exceeded its instruction bound");
    machine.cpu().step();
    machine.bus().advance_instruction_cycles(1);
  }
  const auto count = machine.cpu().debug_state().native_instructions;
  machine.cpu().step();
  require(machine.cpu().state().pc == enable_entry + 2 &&
              machine.bus().read({.space = DspSpace::x, .address = source0 - 3}) == control0,
          "First unchanged original MOVEP enables DMA0 with its actual control word");
  machine.cpu().step();
  require(machine.cpu().state().pc == stopped && machine.cpu().state().sp == 1 &&
              machine.cpu().debug_state().native_instructions == count + 2 &&
              machine.bus().read({.space = DspSpace::x, .address = source3 - 3}) == control3,
          "Second unchanged original MOVEP enables DMA3; the original REP is not executed");
  intact();
}

void Experiment::grant() {
  const auto result = machine.advance_dma(
      {.cycle = driver.cycle++,
       .timing = DspDmaTiming::reference_sampled_cycles,
       .grant = DspDmaGrant::available},
      {.accesses = {}, .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy});
  require(!result.core_stall, "The explicitly paused CPU has no competing core footprint");
  if (!result.word) { return; }
  const auto &word = *result.word;
  require(word.channel == 0 || word.channel == 3, "Only the two original DMA channels may write");
  const unsigned lane = word.channel == 0 ? 0U : 1U;
  const auto ordinal = driver.words.at(lane)++;
  const auto expected_tick = first_launch + (ordinal * slot_ticks) + (lane == 0 ? 1 : 3);
  require(ordinal < word_count && driver.tick == expected_tick &&
              word.source == DspDmaAddress{.address = ring_start + (ordinal % ring_size)} &&
              word.destination ==
                  DspDmaAddress{.space = lane == 0 ? DspDmaSpace::y : DspDmaSpace::x,
                                .address = lane == 0 ? esai_tx : dax_tx} &&
              word.value == expected_word(word.channel, ordinal, driver.changed),
          "Literal 64-word ring, channel priority, captured data and peripheral targets differ");
  driver.trace += "WORD " + std::to_string(driver.tick) + ' ' + std::to_string(word.channel) + ' ' +
                  std::to_string(ordinal) + ' ' + std::to_string(word.value) + '\n';
}

void Experiment::intact() const {
  for (unsigned i = 0; i < record_.words.size(); ++i) {
    require(machine.bus().inspect_program(record_.address + i) == record_.words[i],
            "The entire original extended-services program must remain unchanged");
  }
}
} // namespace blofeld::test::audio_dma
