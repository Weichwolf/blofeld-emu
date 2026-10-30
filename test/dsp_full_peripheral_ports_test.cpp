// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "dsp_scalar_store_fixture.hpp"

#include <array>
#include <bitset>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned secondary = 0xffff80, clock_register = 22, control_register = 21;
constexpr unsigned gpio_control = 31, gpio_direction = 30, normal24 = 0xf801, mux = 0x818;
constexpr unsigned canary = 0x123456, prefix_bits = 7, seed_count = 12, body = boot + 24;
constexpr unsigned a_store = 0x04ce20, b_store = 0x04cf20, byte_mask = 0xff;
constexpr unsigned middle_shift = 24, extension_shift = 48;
constexpr std::array<std::uint64_t, 6> values{
    0, 0x00123456abcdef, 0x00800000000001, 0xff800000000000, 0x55800001abcdef, 0xffffffffffffff};
constexpr auto which = DspEsaiInstance::secondary;

struct Input {
  std::uint64_t value = 0;
  unsigned scale = 0, flags = 0;
  bool b = false;
};

DspFetchRunInputs contract() {
  auto result = pipeline::contract();
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

std::string serial(DspBus &bus, const unsigned begin, const unsigned end) {
  std::string bits;
  for (unsigned index = begin; index < end; ++index) {
    const bool sync = index == 0;
    bus.observe_esai_transmit(which, {.clock = false, .frame_sync = sync});
    bus.observe_esai_transmit(which, {.clock = true, .frame_sync = sync});
    const auto output = bus.esai_transmit_outputs(which);
    require((output.driven & 1U) == 1U && (output.known & 1U) == 1U,
            "Real ESAI TX0 produces a known bit on its selected pad");
    bits.push_back((output.levels & 1U) != 0 ? '1' : '0');
  }
  return bits;
}

void put(DspProgram &code, const unsigned offset, const unsigned value) {
  code.emit("movep #>" + number(value) + ",y:<<" + number(secondary + offset));
}

DspDebugState prepare(DspMachine &machine, const Input input) {
  DspProgram code;
  for (const auto *name : {"a", "b"}) {
    code.immediate(std::string(name) + '0', input.value & word_mask);
    code.immediate(std::string(name) + '1', (input.value >> middle_shift) & word_mask);
    code.immediate(std::string(name) + '2', (input.value >> extension_shift) & byte_mask);
  }
  code.immediate("sr", masked_status | input.scale | input.flags);
  put(code, clock_register, 0);
  put(code, 0, canary);
  put(code, control_register, normal24);
  put(code, gpio_control, mux);
  put(code, gpio_direction, mux);
  require(code.pc() == body, "Twelve literal two-word native seeds precede the tested instruction");
  code.word(input.b ? b_store : a_store);
  for (unsigned index = 0; index < 16; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  checkpoint::run(machine, seed_count);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == body && before.cpu.a == input.value && before.cpu.b == input.value &&
              before.cpu.sr == (masked_status | input.scale | input.flags),
          "Actual native instructions establish accumulator and valid external-clock ESAI setup");
  machine.bus().observe_esai_transmit(which, {.clock = true, .frame_sync = false});
  require(serial(machine.bus(), 0, prefix_bits) ==
              std::bitset<word_bits>(canary).to_string().substr(0, prefix_bits),
          "The first actual frame starts with independently specified old holding data");
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  return before;
}

void step(DspFetchRun &run, DspMachine &machine, const Input input, const DspDebugState &before) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  const auto period = run.completed_cycles();
  const auto &phase = pipeline::stages(cycle);
  const auto expected =
      scalar_store::expected({.value = input.value, .scale = input.scale, .flags = input.flags});
  auto state = before;
  if (period == 7) {
    ++state.cpu.pc;
    ++state.native_instructions;
    ++state.native_cycles;
    state.cpu.sr = masked_status | input.scale | expected.flags;
  }
  require(phase.memory.has_value() == (period == 6) &&
              phase.retirement_published == (period == 7) && !phase.memory_source_stall &&
              machine.cpu().debug_state() == state,
          "Full-source low-Y MOVEP writes only at E1 and retires complete native state/limiter "
          "flags at E2");
}

void finish(DspFetchRun &run, DspMachine &machine, const Input input, const DspDebugState &before) {
  while (run.completed_cycles() < 7) { step(run, machine, input, before); }
  const auto result =
      scalar_store::expected({.value = input.value, .scale = input.scale, .flags = input.flags});
  require(serial(machine.bus(), prefix_bits, word_bits) ==
              std::bitset<word_bits>(canary).to_string().substr(prefix_bits),
          "A mid-frame native MOVEP leaves the current physical shift unchanged");
  require(serial(machine.bus(), 0, word_bits) == std::bitset<word_bits>(result.value).to_string(),
          "The next actual ESAI frame transmits the independently scaled/limited signed24 word");
}

void one(const Input input) {
  DspMachine machine;
  const auto before = prepare(machine, input);
  DspFetchRun run(contract(), machine);
  finish(run, machine, input, before);
}

void restores(const Input input) {
  for (unsigned cut = 5; cut <= 7; ++cut) {
    DspMachine machine;
    const auto before = prepare(machine, input);
    DspFetchRun run(contract(), machine);
    while (run.completed_cycles() < cut) { step(run, machine, input, before); }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      finish(run, machine, input, before);
    }
    DspMachine clone;
    DspFetchRun continuation(saved, clone);
    finish(continuation, clone, input, before);
  }
}
}

int main() {
  try {
    unsigned streams = 0;
    for (const bool b : {false, true}) {
      for (const auto value : values) {
        for (const unsigned scale : {0U, scale_down, scale_up}) {
          for (const unsigned flags : {0U, byte_mask}) {
            const Input input{.value = value, .scale = scale, .flags = flags, .b = b};
            one(input);
            restores(input);
            ++streams;
          }
        }
      }
    }
    std::cout << "Full MOVEP ESAI streams=" << streams
              << ";432suffixes;216clones;24-bit physical frames\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
