// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_rom_program.hpp"
#include "esai_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>

namespace {
using namespace blofeld;
using blofeld::test::DspRomProgram;
using blofeld::test::DspRomRegister;
using blofeld::test::esai::rejects;
using blofeld::test::esai::require;
constexpr auto timing = OwnDspAutomaticTiming::native_cycle_budget;
constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                   .timebase_hz = 540000000,
                                   .switch_timing = DspClockSwitch::reference_cycle_boundary};
constexpr unsigned repeat_count = 23, repeat_immediate = 0x0617a0;
constexpr unsigned instruction_bound = 256, padding_words = 64, prefill_words = 8;
constexpr unsigned load_x0 = 0x44d800, datum_address = 0x100, datum = 0x123456;
constexpr unsigned port_data = 0xffffbd, port_direction = 0xffffbe, port_control = 0xffffbf;
constexpr unsigned all_pins = 0xfff, movep_immediate = 0x07f400, peripheral_low = 0xffff80;
constexpr std::uint64_t period = 45, reset_ticks = 90;

struct Fixture {
  explicit Fixture(DspRomProgram program) {
    for (unsigned index = 0; index < padding_words; ++index) { program.word(0); }
    machine.bus().install_rom({.start = DspRomProgram::boot, .words = program.words()});
    machine.begin_hardware_reset(
        {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
    require(machine.advance_hardware_reset_ticks(reset_ticks) == reset_ticks,
            "The authored native budget program starts from real reset hold");
    machine.release_hardware_reset();
    require(machine.advance_hardware_reset_ticks(reset_ticks) == reset_ticks,
            "The authored native budget program completes real reset recovery");
    fetch = std::make_unique<OwnDspFetch>(machine);
    for (unsigned index = 0; index < prefill_words; ++index) {
      fetch->issue();
      fetch->capture();
    }
  }

  [[nodiscard]] DspDmaInputs inputs() {
    const auto last = machine.bus().last_dma_cycle();
    return {.cycle = last ? *last + 1 : 0,
            .known_requests = 15,
            .timing = DspDmaTiming::reference_sampled_cycles,
            .grant = DspDmaGrant::available};
  }

  OwnDspFetchCycleResult advance() {
    const auto result = fetch->advance_dma_automatic(inputs(), timing);
    if (result.native && result.native->fault.kind != dsp56300::FaultKind::none) {
      const auto fault = result.native->fault;
      throw Fault("Native budget fault address=" + hex(fault.address, 6) +
                  " pc=" + hex(fault.pc, 6));
    }
    return result;
  }

  void retire(const unsigned pc, const unsigned cycles) {
    const auto before = machine.clock().elapsed_ticks();
    for (unsigned count = 0; count < instruction_bound; ++count) {
      const auto result = advance();
      if (result.native && result.native->retired) {
        require(result.native_pc == pc &&
                    machine.clock().elapsed_ticks() - before == cycles * period,
                "Actual retirement matches the independent native address and instruction budget");
        return;
      }
    }
    throw Fault("Native budget instruction exceeded its bounded cycle count");
  }

  OwnDspMachine machine{clocks};
  std::unique_ptr<OwnDspFetch> fetch;
};

DspRomProgram repeated() {
  DspRomProgram program;
  program.word(repeat_immediate);
  return program;
}

void cadence() {
  Fixture plain{DspRomProgram{}};
  constexpr unsigned sequential_count = 20;
  for (unsigned index = 0; index < sequential_count; ++index) {
    plain.retire(DspRomProgram::boot + index, 1);
  }
  Fixture loop{repeated()};
  loop.retire(DspRomProgram::boot, 5);
  for (unsigned index = 0; index < repeat_count; ++index) {
    loop.retire(DspRomProgram::boot + 1, 1);
  }
  require(loop.machine.cpu().progress() == dsp56300::Progress{.instructions = repeat_count + 1,
                                                              .estimated_cycles = repeat_count + 5},
          "Twenty-three repeated NOPs cost twenty-three native periods after five setup periods");
  for (unsigned index = 0; index < 3; ++index) { loop.retire(DspRomProgram::boot + 2 + index, 1); }
}

void padding_and_mixed_timing() {
  Fixture fixture{repeated()};
  const auto start = fixture.machine.clock().elapsed_ticks();
  const auto first = fixture.advance();
  const auto pending = fixture.machine.cpu().pending_instruction();
  require(first.native && !first.native->retired && pending &&
              pending->effect.kind == dsp56300::EffectKind::retire &&
              fixture.machine.cpu().progress() == dsp56300::Progress{},
          "REP's computed genuine retirement remains pending while its native budget is unpaid");
  const auto state = fixture.machine.cpu().registers();
  const auto dma = fixture.machine.bus().last_dma_cycle();
  require(!rejects([&] {
             static_cast<void>(fixture.fetch->advance_dma_automatic(
                 fixture.inputs(), OwnDspAutomaticTiming::one_effect_per_cycle));
           }).empty() &&
              !rejects([&] { fixture.fetch->issue(); }).empty() &&
              !rejects([&] {
                 static_cast<void>(fixture.fetch->advance_dma_capture(fixture.inputs()));
               }).empty() &&
              fixture.machine.bus().last_dma_cycle() == dma &&
              fixture.machine.clock().elapsed_ticks() == start + period,
          "A pending budget rejects manual/reference timing before advancing clocks or DMA");

  for (unsigned count = 0; count < 3; ++count) {
    const auto held = fixture.advance();
    require(!held.native && fixture.machine.cpu().registers() == state &&
                fixture.machine.cpu().progress() == dsp56300::Progress{},
            "Pending padding advances actual clocks without fabricated CPU effects or retirement");
  }
  fixture.retire(DspRomProgram::boot, 1);
  require(fixture.machine.clock().elapsed_ticks() == start + 5 * period,
          "The real continuation retires REP at exactly its fifth native period");
  fixture.retire(DspRomProgram::boot + 1, 1);
}

void bus_phases(const bool known) {
  DspRomProgram program;
  program.immediate(DspRomRegister::r0, datum_address);
  program.word(load_x0);
  Fixture fixture{program};
  if (known) {
    fixture.machine.bus().write({.space = DspSpace::x, .address = datum_address, .value = datum});
  }
  fixture.retire(DspRomProgram::boot, 2);
  const auto before = fixture.machine.clock().elapsed_ticks();
  const auto prefix = fixture.advance();
  const auto pending = fixture.machine.cpu().pending_instruction();
  require(prefix.native && !prefix.native->retired && pending &&
              pending->effect.kind == dsp56300::EffectKind::read &&
              pending->effect.address == datum_address &&
              fixture.machine.cpu().registers().r[0] == datum_address + 1,
          "A genuine AGU prefix leaves its actual operand read for the next admitted bus phase");
  if (!known) {
    const auto result = fixture.fetch->advance_dma_automatic(fixture.inputs(), timing);
    require(result.native && result.native->fault.kind == dsp56300::FaultKind::memory_read &&
                result.native->fault.address == datum_address &&
                result.native->fault.pc == DspRomProgram::boot + 2 &&
                fixture.machine.cpu().progress().instructions == 1,
            "Unknown operands retain the original address/PC fault without successful retirement");
    return;
  }
  fixture.retire(DspRomProgram::boot + 2, 1);
  require(
      fixture.machine.clock().elapsed_ticks() == before + 2 * period &&
          fixture.machine.cpu().registers().x == datum &&
          fixture.machine.cpu().progress().estimated_cycles == 3,
      "Actual bus-phase excess remains explicit instead of being disguised as native base cost");
}

void immediate_write(DspRomProgram &program, const DspStore store) {
  program.word(movep_immediate | (store.address - peripheral_low));
  program.word(store.value);
}

void gpio_cadence() {
  DspRomProgram program;
  immediate_write(program, {.address = port_control});
  immediate_write(program, {.address = port_direction, .value = all_pins});
  immediate_write(program, {.address = port_data, .value = 1});
  program.word(repeat_immediate);
  program.word(0);
  immediate_write(program, {.address = port_data, .value = 2});
  Fixture fixture{program};
  std::uint64_t first = 0;
  for (unsigned count = 0; count < instruction_bound; ++count) {
    static_cast<void>(fixture.advance());
    const auto output = fixture.machine.bus().gpio(DspPort::c).outputs();
    if (output.known != all_pins || output.mask != all_pins) { continue; }
    const auto now = fixture.machine.clock().elapsed_ticks();
    if (output.levels == 1 && first == 0) { first = now; }
    if (output.levels == 2) {
      constexpr unsigned expected_interval = 30; // One remaining MOVEP period + REP5 +23NOP +1.
      require(
          first != 0 && now - first == expected_interval * period,
          "Native GPIO latch changes follow independent instruction timing through the REP body");
      return;
    }
  }
  throw Fault("Actual GPIO writes were not observed within the authored instruction bound");
}
}

int main() {
  try {
    cadence();
    padding_and_mixed_timing();
    bus_phases(true);
    bus_phases(false);
    gpio_cadence();
    std::cout << "Native cycle budgets, REP, real GPIO and memory phases and refusals pass\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
