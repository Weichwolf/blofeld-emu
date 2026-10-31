// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>

namespace {
using namespace blofeld;
using namespace dsp56300;
constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                   .timebase_hz = 540000000,
                                   .switch_timing = DspClockSwitch::reference_cycle_boundary};
constexpr Word boot_entry = 0xff0000;
constexpr unsigned hold_ticks = 90, recovery_ticks = 135, period = 45;
constexpr Word timer_control = 0xffff8f, timer_count = 0xffff8c;
constexpr Word timer_compare = 0xffff8d, timer_load = 0xffff8e;
constexpr Word timer_compare_flag = 0x200000;

void require(const bool value, const char *message) {
  if (!value) { throw std::runtime_error(message); }
}

template <class Operation> void rejects(Operation operation, const char *message) {
  try {
    operation();
  } catch (const std::logic_error &) { return; } catch (const blofeld::Fault &) {
    return;
  }
  throw std::runtime_error(message);
}

void boot(OwnDspMachine &machine, const std::span<const Word> program) {
  machine.bus().install_rom({.space = DspSpace::p, .start = boot_entry, .words = program});
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3});
  require(machine.advance_hardware_reset_ticks(hold_ticks) == hold_ticks, "Reset hold differs");
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset_ticks(recovery_ticks) == recovery_ticks,
          "Reset recovery differs");
}

void retire(OwnDspMachine &machine) {
  for (unsigned count = 0; count < 32; ++count) {
    const auto pending = machine.cpu().pending_instruction();
    if (!pending) { throw std::runtime_error("Native instruction lost its effect"); }
    const auto result = machine.advance(pending->ticket);
    require(result.fault.kind == FaultKind::none, "Native effect failed");
    if (result.retired) { return; }
  }
  throw std::runtime_error("Native instruction exceeded its effect bound");
}

void timer_and_pending() {
  OwnDspMachine machine(clocks);
  const std::array<Word, 1> rom{0};
  boot(machine, rom);
  auto &timer = machine.bus().timers();
  timer.write({.address = timer_control, .value = 0});
  timer.write({.address = timer_load, .value = 7});
  timer.write({.address = timer_compare, .value = 9});
  timer.write({.address = timer_control, .value = 7});
  static_cast<void>(machine.begin_fetch());
  const auto pending = machine.cpu().pending_instruction();
  if (!pending) { throw std::runtime_error("Expected native opcode fetch"); }
  const auto before = machine.cpu().registers();
  const auto saved = machine.checkpoint();
  for (unsigned cycle = 0; cycle < 6; ++cycle) {
    require(machine.advance_reference_cycle() == period, "Reference period differs");
    const auto current = machine.cpu().pending_instruction();
    if (!current) { throw std::runtime_error("Clock advance lost its pending instruction"); }
    require(machine.cpu().registers() == before && machine.cpu().progress().instructions == 0 &&
                current->effect == pending->effect,
            "Clock advance must not admit or retire a CPU effect");
  }
  require(machine.clock().elapsed_ticks() == hold_ticks + recovery_ticks + 6 * period &&
              machine.clock().native_cycle_units() == 6 &&
              machine.clock().timer_clk2_edges() == 3 && timer.read(timer_count) == 9 &&
              (timer.read(timer_control, timer_compare_flag) & timer_compare_flag) != 0,
          "Six explicit periods must generate the documented three timer edges");
  machine.restore(saved);
  require(machine.clock().elapsed_ticks() == hold_ticks + recovery_ticks &&
              machine.clock().native_cycle_units() == 0 && machine.clock().timer_clk2_edges() == 0,
          "Coordinated restore must rewind reference time and timer ownership");
  retire(machine);
  require(machine.cpu().progress().instructions == 1 &&
              machine.clock().elapsed_ticks() == hold_ticks + recovery_ticks,
          "Actual CPU retirement must not manufacture an extra reference period");
}

void pll_contract() {
  constexpr Word faster_pll = 0x02e112;
  constexpr Word reset_omr = 0x305;
  constexpr unsigned faster_period = 10;
  constexpr std::uint64_t three_periods = std::uint64_t{3} * period;
  OwnDspMachine machine(clocks);
  const std::array<Word, 1> rom{0};
  boot(machine, rom);
  require(machine.advance_reference_cycle() == period, "Initial reference period differs");
  machine.bus().pll().write(faster_pll);
  require(machine.advance_reference_cycle() == faster_period &&
              machine.clock().elapsed_ticks() ==
                  hold_ticks + recovery_ticks + period + faster_period,
          "Explicit cycle-boundary PLL policy must apply the real next-period ratio");
  DspRunClock separate(clocks);
  rejects([&] { static_cast<void>(separate.next_step_ticks(machine.bus(), 2)); },
          "Cycle-boundary PLL policy accepted a multi-cycle step");
  auto instruction_timing = clocks;
  instruction_timing.switch_timing = DspClockSwitch::reference_instruction_boundary;
  OwnDspMachine incompatible(instruction_timing);
  boot(incompatible, rom);
  rejects([&] { static_cast<void>(incompatible.advance_reference_cycle()); },
          "Instruction-boundary PLL policy accepted an implicit cycle policy");
  require(incompatible.clock().native_cycle_units() == 0 && !incompatible.clock().faulted(),
          "Rejected PLL policy must not consume or fault the clock");
  DspRunClock instruction_clock(instruction_timing);
  instruction_clock.begin_step(incompatible.bus(), {.pc = boot_entry, .omr = reset_omr});
  instruction_clock.finish_step(incompatible.bus(), 3);
  require(instruction_clock.elapsed_ticks() == three_periods &&
              instruction_clock.timer_clk2_edges() == 1,
          "Existing instruction-boundary policy must retain multi-cycle steps");
}

void dma_refusal() {
  constexpr Word source = 0xffffef;
  constexpr Word destination = 0xffffee;
  constexpr Word count = 0xffffed;
  constexpr Word control = 0xffffec;
  constexpr Word enabled_copy = 0x880244;
  OwnDspMachine machine(clocks);
  const std::array<Word, 1> rom{0};
  boot(machine, rom);
  for (const auto write :
       std::array{DspStore{.space = DspSpace::x, .address = source, .value = 1},
                  DspStore{.space = DspSpace::x, .address = destination, .value = 2},
                  DspStore{.space = DspSpace::x, .address = count, .value = 0},
                  DspStore{.space = DspSpace::x, .address = control, .value = enabled_copy}}) {
    machine.bus().write(write);
  }
  try {
    static_cast<void>(machine.advance_reference_cycle());
    throw std::runtime_error("Unarbitrated DMA cycle was accepted");
  } catch (const blofeld::Fault &error) {
    const std::string detail(error.what());
    require(detail.contains("DMA arbitration") && detail.contains("X:0xfffff4") &&
                detail.contains("pc=0xff0000"),
            "DMA refusal must identify the actual missing arbitration and PC");
  }
  require(!machine.clock().faulted() && machine.clock().native_cycle_units() == 0,
          "A refused DMA sample must not consume or fault a reference period");
}

void held_core_dma() {
  constexpr Word source_register = 0xffffef;
  constexpr Word destination_register = 0xffffee;
  constexpr Word counter_register = 0xffffed;
  constexpr Word control_register = 0xffffec;
  constexpr Word source = 0x100;
  constexpr Word destination = 0x200;
  constexpr Word sample = 0x5a1234;
  constexpr Word software_copy = 0x9802d4;
  OwnDspMachine machine(clocks);
  const std::array<Word, 1> rom{0};
  boot(machine, rom);
  machine.bus().write({.space = DspSpace::x, .address = source, .value = sample});
  for (const auto write : std::array{
           DspStore{.space = DspSpace::x, .address = source_register, .value = source},
           DspStore{.space = DspSpace::x, .address = destination_register, .value = destination},
           DspStore{.space = DspSpace::x, .address = counter_register, .value = 0},
           DspStore{.space = DspSpace::x, .address = control_register, .value = software_copy}}) {
    machine.bus().write(write);
  }
  static_cast<void>(machine.begin_fetch());
  const auto before = machine.cpu().pending_instruction();
  if (!before) { throw std::runtime_error("Expected a held native fetch"); }
  const auto saved = machine.checkpoint();
  unsigned reads = 0;
  unsigned writes = 0;
  for (unsigned cycle = 0; cycle < 6; ++cycle) {
    const auto result =
        machine.advance_dma_reference_cycle({.cycle = cycle,
                                             .known_requests = 15,
                                             .timing = DspDmaTiming::reference_sampled_cycles,
                                             .grant = DspDmaGrant::available});
    if (result.phase && result.phase->kind == DspDmaPhaseKind::source_read) { ++reads; }
    if (result.word) {
      ++writes;
      require(result.word->value == sample, "DMA returned a different physical source word");
    }
    const auto pending = machine.cpu().pending_instruction();
    require(pending && pending->effect == before->effect && !result.core_stall,
            "Held CPU must not execute or claim a bus conflict");
  }
  require(reads == 1 && writes == 1 &&
              machine.bus().read({.space = DspSpace::y, .address = destination}) == sample &&
              machine.clock().native_cycle_units() == 6 && machine.clock().timer_clk2_edges() == 3,
          "Real DMA read/write and peripheral clock did not complete together");
  rejects([&] { static_cast<void>(machine.advance_reference_cycle()); },
          "DMA observation cannot silently skip later request samples");
  try {
    static_cast<void>(
        machine.advance_dma_reference_cycle({.cycle = 7,
                                             .known_requests = 15,
                                             .timing = DspDmaTiming::reference_sampled_cycles,
                                             .grant = DspDmaGrant::available}));
    throw std::runtime_error("Skipped DMA sample was accepted");
  } catch (const blofeld::Fault &error) {
    const std::string detail(error.what());
    require(detail.contains("pc=0xff0000") && detail.contains("cycle=7") &&
                machine.clock().faulted(),
            "Invalid DMA sequence must fault with the actual cycle and PC");
  }
  machine.restore(saved);
  require(machine.clock().native_cycle_units() == 0 && !machine.bus().last_dma_cycle(),
          "Checkpoint must restore both DMA sampling and reference time");
}

DspDmaInputs dma_input(const unsigned cycle, const DspDmaGrant grant = DspDmaGrant::available) {
  return {.cycle = cycle,
          .known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = grant};
}

void prepare_native_dma(OwnDspMachine &machine, const Word operand) {
  constexpr Word source_register = 0xffffef;
  constexpr Word destination_register = 0xffffee;
  constexpr Word counter_register = 0xffffed;
  constexpr Word control_register = 0xffffec;
  constexpr Word source = 0x100;
  constexpr Word destination = 0x200;
  constexpr Word sample = 0x5a1234;
  constexpr Word initial_destination = 0x112233;
  constexpr Word x_copy = 0x9802d0;
  constexpr Word load_x0_absolute = 0x44f000;
  const std::array<Word, 2> rom{load_x0_absolute, operand};
  boot(machine, rom);
  machine.bus().write({.space = DspSpace::x, .address = source, .value = sample});
  machine.bus().write({.space = DspSpace::x, .address = destination, .value = initial_destination});
  static_cast<void>(machine.begin_fetch());
  for (unsigned index = 0; index < 8; ++index) {
    const auto pending = machine.cpu().pending_instruction();
    if (!pending) { throw std::runtime_error("Native DMA fixture lost pending load"); }
    if (pending->effect.kind == EffectKind::read) {
      require(pending->effect.space == Space::x && pending->effect.address == operand,
              "Native DMA fixture must use the actual expected data read");
      for (const auto write : std::array{
               DspStore{.space = DspSpace::x, .address = source_register, .value = source},
               DspStore{
                   .space = DspSpace::x, .address = destination_register, .value = destination},
               DspStore{.space = DspSpace::x, .address = counter_register},
               DspStore{.space = DspSpace::x, .address = control_register, .value = x_copy}}) {
        machine.bus().write(write);
      }
      return;
    }
    require(machine.advance(pending->ticket).fault.kind == FaultKind::none,
            "Native DMA fixture fetch failed");
  }
  throw std::runtime_error("Native DMA fixture did not reach its real data read");
}

void native_dma_fetch_independent() {
  constexpr Word source_register = 0xffffef;
  constexpr Word destination_register = 0xffffee;
  constexpr Word counter_register = 0xffffed;
  constexpr Word control_register = 0xffffec;
  constexpr Word source = 0x100;
  constexpr Word destination = 0x200;
  constexpr Word sample = 0x5a1234;
  constexpr Word x_copy = 0x9802d0;
  OwnDspMachine machine(clocks);
  const std::array<Word, 1> rom{0};
  boot(machine, rom);
  machine.bus().write({.space = DspSpace::x, .address = source, .value = sample});
  for (const auto write : std::array{
           DspStore{.space = DspSpace::x, .address = source_register, .value = source},
           DspStore{.space = DspSpace::x, .address = destination_register, .value = destination},
           DspStore{.space = DspSpace::x, .address = counter_register},
           DspStore{.space = DspSpace::x, .address = control_register, .value = x_copy}}) {
    machine.bus().write(write);
  }
  rejects(
      [&] {
        static_cast<void>(machine.advance_dma_native_cycle(
            dma_input(0), OwnDspNativeCycleModel::isolated_effect));
      },
      "DMA/native cycle without a pending core effect was accepted");
  require(!machine.bus().last_dma_cycle() && machine.clock().native_cycle_units() == 0,
          "Rejected native admission must not consume a DMA sample");
  static_cast<void>(machine.begin_fetch());
  const auto pending = machine.cpu().pending_instruction();
  require(pending && pending->effect.kind == EffectKind::fetch_opcode,
          "Expected the actual native program fetch");
  rejects(
      [&] {
        static_cast<void>(
            machine.advance_dma_native_cycle(dma_input(0), OwnDspNativeCycleModel::unqualified));
      },
      "DMA/native cycle must require an explicit isolated-effect hypothesis");
  require(!machine.bus().last_dma_cycle() && machine.clock().native_cycle_units() == 0,
          "Unqualified model must not consume a DMA sample");
  const auto result =
      machine.advance_dma_native_cycle(dma_input(0), OwnDspNativeCycleModel::isolated_effect);
  require(result.dma.phase && result.dma.phase->kind == DspDmaPhaseKind::source_read &&
              !result.dma.core_stall && result.native &&
              result.native->fault.kind == FaultKind::none,
          "Independent native program fetch and DMA data read must both execute");
  const auto destination_phase = machine.advance_dma_reference_cycle(dma_input(1));
  require(destination_phase.word && destination_phase.word->value == sample,
          "Independent native fetch must preserve the real DMA destination");
}

void native_dma_source_conflict() {
  constexpr Word source = 0x100;
  constexpr Word sample = 0x5a1234;
  OwnDspMachine machine(clocks);
  prepare_native_dma(machine, source);
  const auto first =
      machine.advance_dma_native_cycle(dma_input(0), OwnDspNativeCycleModel::isolated_effect);
  require(first.dma.phase && first.dma.phase->kind == DspDmaPhaseKind::source_read &&
              !first.dma.word && !first.dma.core_stall && first.native &&
              first.native->fault.kind == FaultKind::none,
          "Conflicting native read must execute while DMA source waits");
  const auto second = machine.advance_dma_reference_cycle(dma_input(1));
  const auto third = machine.advance_dma_reference_cycle(dma_input(2));
  require(second.phase && second.phase->kind == DspDmaPhaseKind::source_read && third.word &&
              third.word->value == sample && machine.clock().native_cycle_units() == 3,
          "Deferred DMA source must later read and copy the real value");
  retire(machine);
  require(machine.cpu().registers().x == sample, "Native load must read the original source");
}

void native_dma_destination_conflict() {
  constexpr Word destination = 0x200;
  constexpr Word sample = 0x5a1234;
  OwnDspMachine machine(clocks);
  prepare_native_dma(machine, destination);
  const auto original = machine.cpu().pending_instruction();
  const auto source = machine.advance_dma_reference_cycle(dma_input(0));
  const auto waiting = machine.advance_dma_native_cycle(dma_input(1, DspDmaGrant::wait),
                                                        OwnDspNativeCycleModel::isolated_effect);
  const auto stalled =
      machine.advance_dma_native_cycle(dma_input(2), OwnDspNativeCycleModel::isolated_effect);
  const auto pending = machine.cpu().pending_instruction();
  require(source.phase && source.phase->kind == DspDmaPhaseKind::source_read &&
              waiting.dma.core_stall && !waiting.dma.word && !waiting.native &&
              stalled.dma.core_stall && stalled.dma.word && !stalled.native && original &&
              pending && original->effect == pending->effect &&
              machine.bus().read({.space = DspSpace::x, .address = destination}) == sample,
          "DMA destination must hold the genuine pending core read through wait and write");
  const auto admitted =
      machine.advance_dma_native_cycle(dma_input(3), OwnDspNativeCycleModel::isolated_effect);
  require(!admitted.dma.core_stall && admitted.native &&
              admitted.native->fault.kind == FaultKind::none,
          "The held native read must execute after DMA releases the bank");
  retire(machine);
  require(machine.cpu().registers().x == sample && machine.clock().native_cycle_units() == 4,
          "Native load must observe the actually written DMA value");
}

void native_dma_bit_refusal() {
  constexpr Word memory_clear = 0x0b8000;
  OwnDspMachine machine(clocks);
  const std::array<Word, 1> rom{memory_clear};
  boot(machine, rom);
  static_cast<void>(machine.begin_fetch());
  for (unsigned index = 0; index < 8; ++index) {
    const auto pending = machine.cpu().pending_instruction();
    if (!pending) { throw std::runtime_error("Native bit fixture lost its effect"); }
    if (pending->effect.kind == EffectKind::bit) {
      try {
        static_cast<void>(machine.advance_dma_native_cycle(
            dma_input(0), OwnDspNativeCycleModel::isolated_effect));
        throw std::runtime_error("Unqualified atomic native bit cycle was accepted");
      } catch (const blofeld::Fault &error) {
        const std::string detail(error.what());
        require(detail.contains("bit X:0xffffc0") && detail.contains("pc=0xff0000") &&
                    !machine.bus().last_dma_cycle() && machine.clock().native_cycle_units() == 0 &&
                    !machine.clock().faulted(),
                "Unqualified bit cycle must retain address and PC without consuming time");
      }
      return;
    }
    const auto result = machine.advance(pending->ticket);
    if (result.fault.kind != FaultKind::none) {
      throw std::runtime_error(
          "Native bit fixture fault=" + std::to_string(static_cast<unsigned>(result.fault.kind)) +
          " address=" + std::to_string(result.fault.address));
    }
  }
  throw std::runtime_error("Native bit fixture did not reach its transaction");
}

void timer_fault_recovery() {
  OwnDspMachine machine(clocks);
  const std::array<Word, 1> rom{0};
  boot(machine, rom);
  machine.bus().timers().write({.address = timer_control, .value = 7});
  const auto healthy = machine.checkpoint();
  require(machine.advance_reference_cycle() == period, "First half timer period differs");
  try {
    static_cast<void>(machine.advance_reference_cycle());
    throw std::runtime_error("Unknown timer load was accepted");
  } catch (const blofeld::Fault &error) {
    const std::string detail(error.what());
    require(detail.contains("pc=0xff0000") && detail.contains("timer"),
            "Clock failure must retain actual CPU context and timer cause");
  }
  require(machine.clock().faulted() && machine.cpu().progress().instructions == 0,
          "Unknown peripheral timing must fail stop without CPU retirement");
  rejects([&] { static_cast<void>(machine.advance_reference_cycle()); },
          "A faulted clock accepted another reference cycle");
  machine.restore(healthy);
  require(!machine.clock().faulted() && machine.clock().native_cycle_units() == 0 &&
              machine.clock().elapsed_ticks() == hold_ticks + recovery_ticks,
          "Coordinated checkpoint must recover the clock and bus");
}

void stopped_and_waiting(const Word opcode, const ProcessingState expected) {
  OwnDspMachine machine(clocks);
  const std::array<Word, 1> rom{opcode};
  boot(machine, rom);
  static_cast<void>(machine.begin_fetch());
  retire(machine);
  require(machine.cpu().processing_state() == expected, "Native low-power state differs");
  if (expected == ProcessingState::stopped) {
    rejects([&] { static_cast<void>(machine.advance_reference_cycle()); },
            "Unqualified STOP oscillator was advanced");
    require(machine.clock().native_cycle_units() == 0, "Rejected STOP advanced the clock");
  } else {
    require(machine.advance_reference_cycle() == period &&
                machine.clock().native_cycle_units() == 1,
            "WAIT must retain its explicit peripheral clock");
  }
}

void callback_guard() {
  constexpr Word load_x0_absolute = 0x44f000;
  constexpr Word port_c_data = 0xffffbd;
  constexpr Word pins = 0xfff;
  constexpr Word incoming = 0x5a5;
  const std::array<Word, 2> rom{load_x0_absolute, port_c_data};
  OwnDspMachine machine(clocks);
  boot(machine, rom);
  auto &port = machine.bus().gpio(DspPort::c);
  port.write({.reg = DspGpioRegister::control, .value = pins});
  unsigned calls = 0;
  const auto checkpoint = machine.checkpoint();
  port.attach_pad_source([&](const std::uint32_t mask) {
    ++calls;
    rejects([&] { static_cast<void>(machine.advance_reference_cycle()); },
            "A callback advanced time during an active effect");
    rejects([&] { static_cast<void>(machine.checkpoint()); },
            "A callback captured a partially applied effect");
    rejects([&] { machine.restore(checkpoint); }, "A callback restored a partially applied effect");
    rejects(
        [&] {
          machine.begin_hardware_reset(
              {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3});
        },
        "A callback reset the board during an active effect");
    return GpioPadSample{.known = mask, .levels = incoming & mask};
  });
  static_cast<void>(machine.begin_fetch());
  retire(machine);
  require(calls == 1 && machine.cpu().registers().x == incoming && !machine.io().failure() &&
              !machine.clock().faulted() && machine.clock().native_cycle_units() == 0,
          "Rejected reentry must leave the actual GPIO read and clock intact");
}
}

int main() {
  try {
    constexpr Word stop = 0x87;
    constexpr Word wait = 0x86;
    timer_and_pending();
    pll_contract();
    dma_refusal();
    held_core_dma();
    native_dma_fetch_independent();
    native_dma_source_conflict();
    native_dma_destination_conflict();
    native_dma_bit_refusal();
    timer_fault_recovery();
    stopped_and_waiting(stop, ProcessingState::stopped);
    stopped_and_waiting(wait, ProcessingState::waiting);
    callback_guard();
    std::cout << "Own DSP explicit reference-clock checks passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
