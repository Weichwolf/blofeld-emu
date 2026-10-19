// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "mcu_checkpoint_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld::test;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned other_oscillator = 13000000;
constexpr unsigned clock_select = 0x39;
constexpr unsigned usb_high = 0x300;
constexpr unsigned phy_start = 0x40;
constexpr unsigned pll_select = 0x80;
constexpr unsigned timer_counter = 0x44;
constexpr unsigned timer_edges = 0x4b;
constexpr unsigned timer_flags = 0x4e;
constexpr unsigned timer_capture = 0x50;
constexpr unsigned realtime_control = 0x3b;
constexpr unsigned realtime_period = 1024;
constexpr unsigned masked = 0xd0;
constexpr unsigned selected_input = 8;
constexpr unsigned initialized_m_outputs = 0x37;
constexpr unsigned reset_cycles = 5;
constexpr unsigned reset_ticks = 50;
constexpr unsigned first_phy_ticks = 52;
constexpr unsigned final_phase_ticks = 66;
constexpr unsigned replay_phy_ticks = 54;
constexpr unsigned oscillator_nop_ticks = 60;
constexpr unsigned loaded_ticks = 80;
constexpr unsigned sci_mark = 16;
constexpr unsigned sci_bit_cycles = 16;
constexpr unsigned sci_frame_end = 192;
constexpr unsigned sci_checkpoint_cycles = 63;
constexpr unsigned sci_pattern = 0x55;
constexpr unsigned sci_ready = 0xe0;
constexpr unsigned sci_idle = 0xc0;

blofeld::McuRunContract run_contract() {
  return {.clock = {.oscillator_hz = oscillator_hz,
                    .initial_oscillator_phase_ticks = 0,
                    .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = blofeld::McuRunMemory::unstretched_internal};
}

std::string reject_text(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) { return fault.what(); }
  throw blofeld::Fault("Expected coordinated MCU run rejection");
}

void select_phy(blofeld::McuBus &bus) {
  bus.write(clock_select, 0);
  bus.write(usb_high, phy_start);
  bus.observe_phy({.edges = 15});
  bus.write(clock_select, pll_select);
}

void check_time(const blofeld::McuMachine &machine,
                const unsigned cycles,
                const unsigned ticks,
                const unsigned edges) {
  require(machine.clocked() && !machine.timing_faulted() &&
              machine.run_clock().bus_cycles() == cycles &&
              machine.run_clock().elapsed_ticks() == ticks &&
              machine.run_clock().oscillator_cycles() == edges,
          "Owned native cycles must match independent source/divider/phase expectations");
}

void test_literal_phase_and_callback_lifetime() {
  std::optional<blofeld::McuCheckpoint> frozen;
  {
    blofeld::McuMachine machine(checkpoint_image(), BLOFELD_DEFAULT_SLA, run_contract());
    check_time(machine, reset_cycles, reset_ticks, 10);
    machine.bus().write(blofeld::hc12::tscr1, pll_select);
    machine.bus().write(realtime_control, 0x10);
    select_phy(machine.bus());
    unsigned observations = 0;
    machine.set_cycle_observer([&observations](auto /*cycle*/) { ++observations; });
    machine.cpu().step(); // One-cycle NOP; retain 2/5 of an oscillator period.
    frozen.emplace(machine.checkpoint());
    check_time(machine, 6, first_phy_ticks, 10);
    require(observations == 1 && machine.bus().read(timer_counter + 1) == 1 &&
                machine.bus().oscillator_cycles_until_rti() == realtime_period,
            "Capture has no callback/clock effects and preserves the unfinished oscillator edge");
    machine.bus().write(clock_select, 0);
    machine.cpu().step(); // 10 ticks, two OSC edges.
    machine.bus().write(clock_select, pll_select);
    machine.cpu().step();
    machine.cpu().step(); // 2 + 2 ticks; the saved fraction contributes another edge.
    check_time(machine, 9, final_phase_ticks, 13);
    require(machine.bus().read(timer_counter + 1) == 4 && observations == 4,
            "Native NOPs advance TIM once per bus cycle, across source changes");
    machine.restore(*frozen);
    check_time(machine, 6, first_phy_ticks, 10);
    machine.cpu().step();
    check_time(machine, 7, replay_phy_ticks, 10);
    require(observations == 4 && machine.bus().read(timer_counter + 1) == 2,
            "Restore drops the old cycle callback and restores the timer's exact phase");
  }
  blofeld::McuMachine branch(checkpoint_image(), BLOFELD_DEFAULT_SLA, run_contract());
  if (!frozen) { throw blofeld::Fault("The trusted checkpoint must outlive its source machine"); }
  branch.restore(*frozen);
  branch.bus().write(clock_select, 0);
  branch.cpu().step();
  branch.bus().write(clock_select, pll_select);
  branch.cpu().step();
  branch.cpu().step();
  check_time(branch, 9, final_phase_ticks, 13);
  require(branch.cpu().state().pc == entry + 4 &&
              branch.bus().oscillator_cycles_until_rti() == realtime_period - 3,
          "A second owner retains the native PC and RTI phase with no source references");
}

void test_operand_fault_and_coordinated_recovery() {
  constexpr std::array<std::uint8_t, 3> code{0xb6, 2, 0x48}; // Own LDAA PTM.
  blofeld::McuMachine machine(checkpoint_image(code), BLOFELD_DEFAULT_SLA, run_contract());
  machine.bus().write(blofeld::hc12::ddrm, initialized_m_outputs);
  machine.cpu().set_state({.pc = entry, .sp = stack, .ccr = masked});
  const auto frozen = machine.checkpoint();
  const auto error = reject_text([&machine] { machine.cpu().step(); });
  require(error.contains("pc=0x8000") && error.contains("0x0248") && error.contains("mask=0x08") &&
              machine.timing_faulted() && machine.run_clock().bus_cycles() == 6 &&
              machine.run_clock().elapsed_ticks() == oscillator_nop_ticks,
          "An unknown GPIO read consumes its real first cycle and latches the timed adapter");
  machine.bus().set_pin_drive(blofeld::McuPort::m, {.mask = selected_input});
  reject([&machine] { machine.cpu().step(); }, "A new pad observation alone cannot repair time");
  machine.cpu().set_state({.pc = entry, .sp = stack, .ccr = masked});
  reject([&machine] { static_cast<void>(machine.checkpoint()); },
         "Debug register replacement cannot hide a failed timed sequence");
  for (const unsigned level : {0U, selected_input}) {
    machine.restore(frozen);
    check_time(machine, reset_cycles, reset_ticks, 10);
    require(machine.bus().observe_pads(blofeld::McuPort::m, selected_input).known == 0,
            "The restored checkpoint retains the unknown input, without a guessed pull");
    machine.bus().set_pin_drive(
        blofeld::McuPort::m, {.mask = selected_input, .levels = static_cast<std::uint8_t>(level)});
    machine.cpu().step();
    check_time(machine, 8, loaded_ticks, 16);
    require(machine.cpu().state().pc == entry + code.size() &&
                machine.cpu().state().d == level * 256U && machine.cpu().state().sp == stack &&
                machine.cpu().state().ccr == (masked | (level == 0 ? 4U : 0U)),
            "Both explicit input branches execute the same native load with literal flags/time");
  }
}

void test_actual_mapping_qualifies_word_timing() {
  constexpr unsigned remapped_ram = 0x3201;
  constexpr unsigned ram_mapping = 0x29;
  constexpr unsigned word = 0x1234;
  constexpr std::array<std::uint8_t, 3> ram_code{0xfc, 0x32, 1};
  blofeld::McuMachine moved(checkpoint_image(ram_code), BLOFELD_DEFAULT_SLA, run_contract());
  moved.bus().write(blofeld::hc12::initrm, ram_mapping);
  moved.bus().retire_instruction();
  moved.bus().write_memory({.address = remapped_ram, .size = 2, .value = word});
  moved.cpu().step();
  check_time(moved, 8, loaded_ticks, 16);
  require(moved.cpu().state().d == word,
          "Unaligned words are qualified by the actual relocated internal RAM mapping");
  constexpr std::array<std::array<std::uint8_t, 3>, 3> failures{
      {{0xfc, 0x80, 1}, {0xfc, 0x1f, 0xff}, {0xfc, 0x12, 1}}};
  for (unsigned index = 0; index < failures.size(); ++index) {
    blofeld::McuMachine machine(
        checkpoint_image(failures[index]), BLOFELD_DEFAULT_SLA, run_contract());
    if (index == 2) {
      machine.bus().write(blofeld::hc12::initrg, 0x10);
      machine.bus().retire_instruction();
    }
    const auto error = reject_text([&machine] { machine.cpu().step(); });
    require(error.contains("Unqualified MCU reference R address=") && error.contains("pc=0x8000") &&
                machine.run_clock().bus_cycles() == reset_cycles && machine.timing_faulted(),
            "Odd flash/register words and RAM-boundary crossings reject before advancing time");
  }
}

void test_timer_input_mask_and_history() {
  blofeld::McuMachine machine(checkpoint_image(), BLOFELD_DEFAULT_SLA, run_contract());
  machine.bus().set_pin_drive(blofeld::McuPort::t, {.mask = 1});
  machine.set_timer_inputs(1);
  machine.bus().write(blofeld::hc12::tscr1, pll_select);
  machine.bus().write(timer_edges, 1);
  machine.cpu().step();
  const auto frozen = machine.checkpoint();
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    machine.restore(frozen);
    machine.bus().set_pin_drive(blofeld::McuPort::t, {.mask = 1, .levels = 1});
    machine.cpu().step();
    require(machine.bus().read_memory({.address = timer_capture, .size = 2}) == 1 &&
                machine.bus().read(timer_flags) == 1 && machine.bus().read(timer_counter + 1) == 2,
            "The restored observation mask/history captures the real edge at the literal count");
    machine.bus().write(timer_edges, 0);
    machine.set_timer_inputs(0);
  }
  machine.restore(frozen);
  machine.bus().set_pin_drive(blofeld::McuPort::t, {});
  const auto cycles = machine.run_clock().bus_cycles();
  reject([&machine] { machine.cpu().step(); }, "An unknown selected timer pad must fail");
  require(machine.timing_faulted() && machine.run_clock().bus_cycles() == cycles,
          "Sampling fails before a clock; no high/low observation is silently substituted");
}

void attach_serial_observations(blofeld::McuMachine &machine) {
  machine.set_cycle_observer([&machine](auto /*cycle*/) {
    const auto elapsed = machine.run_clock().bus_cycles() - reset_cycles;
    bool high = true;
    if (elapsed >= sci_mark && elapsed < sci_mark + sci_bit_cycles) { high = false; }
    constexpr unsigned data_begin = sci_mark + sci_bit_cycles;
    if (elapsed >= data_begin && elapsed < data_begin + (8 * sci_bit_cycles)) {
      const auto bit = static_cast<unsigned>((elapsed - data_begin) / sci_bit_cycles);
      high = ((sci_pattern >> bit) & 1U) != 0;
    }
    machine.bus().set_pin_drive(blofeld::McuPort::r,
                                {.mask = 4, .levels = static_cast<std::uint8_t>(high ? 4 : 0)});
  });
}

void test_native_sci_half_frame() {
  constexpr std::array<std::uint8_t, 2> code{0x20, 0xfe}; // Three-cycle BRA self.
  blofeld::McuMachine machine(checkpoint_image(code), BLOFELD_DEFAULT_SLA, run_contract());
  machine.bus().write(blofeld::hc12::sci_baud_high, 0);
  machine.bus().write(blofeld::hc12::sci_baud_low, 1);
  machine.bus().write(blofeld::hc12::sci_control2, 4);
  attach_serial_observations(machine);
  while (machine.run_clock().bus_cycles() < reset_cycles + sci_checkpoint_cycles) {
    machine.cpu().step();
  }
  const auto frozen = machine.checkpoint();
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    machine.restore(frozen);
    attach_serial_observations(machine);
    while (machine.run_clock().bus_cycles() < reset_cycles + sci_frame_end) {
      machine.cpu().step();
    }
    require(machine.bus().read(blofeld::hc12::sci_status1) == sci_ready &&
                machine.bus().read(blofeld::hc12::sci_data_low) == sci_pattern &&
                machine.bus().read(blofeld::hc12::sci_status1) == sci_idle,
            "Restored SCI receives the literal NRZ byte using only running native CPU cycles");
    constexpr unsigned final_cycles = reset_cycles + sci_frame_end;
    check_time(machine, final_cycles, final_cycles * 10, final_cycles * 2);
    require(machine.cpu().state().pc == entry, "The native foreground branch stays intact");
  }
}

void test_active_guards_and_contract_compatibility() {
  blofeld::McuMachine machine(checkpoint_image(), BLOFELD_DEFAULT_SLA, run_contract());
  const auto frozen = machine.checkpoint();
  unsigned calls = 0;
  machine.set_cycle_observer([&machine, &frozen, &calls](auto /*cycle*/) {
    reject([&machine] { static_cast<void>(machine.checkpoint()); },
           "A cycle observation cannot snapshot an executing instruction");
    reject([&machine, &frozen] { machine.restore(frozen); },
           "A cycle observation cannot replace its active owner");
    reject([&machine] { machine.set_cycle_observer({}); },
           "An active callback cannot destroy itself");
    reject([&machine] { machine.set_timer_inputs(0); },
           "An active callback cannot change the owner's sampling configuration");
    ++calls;
  });
  machine.cpu().step();
  require(calls == 1 && !machine.timing_faulted(), "Caught active guards preserve valid execution");
  machine.restore(frozen);
  machine.set_cycle_observer(
      [](auto /*cycle*/) { throw blofeld::Fault("Own observation failure"); });
  reject([&machine] { machine.cpu().step(); }, "Observation exceptions must fail the cycle");
  require(machine.timing_faulted() && machine.run_clock().bus_cycles() == reset_cycles,
          "A failed external observation does not advance the owned clock");
  machine.restore(frozen);
  machine.cpu().step();
  check_time(machine, 6, oscillator_nop_ticks, 12);
  blofeld::McuMachine untimed(checkpoint_image(), BLOFELD_DEFAULT_SLA);
  const auto untimed_checkpoint = untimed.checkpoint();
  reject([&untimed, &frozen] { untimed.restore(frozen); }, "Timed state cannot seed untimed time");
  reject([&machine, &untimed_checkpoint] { machine.restore(untimed_checkpoint); },
         "Untimed state cannot synthesize elapsed clock state");
  reject([&untimed] { static_cast<void>(untimed.run_clock()); }, "Untimed runs have no clock");
  auto other = run_contract();
  other.clock.oscillator_hz = other_oscillator;
  blofeld::McuMachine foreign(checkpoint_image(), BLOFELD_DEFAULT_SLA, other);
  reject([&foreign, &frozen] { foreign.restore(frozen); },
         "Different oscillator contracts reject before replacing live state");
  check_time(machine, 6, oscillator_nop_ticks, 12);
  auto unqualified = run_contract();
  unqualified.memory_timing = blofeld::McuRunMemory::unqualified;
  reject(
      [&unqualified] {
        const blofeld::McuMachine invalid(checkpoint_image(), BLOFELD_DEFAULT_SLA, unqualified);
      },
      "Unqualified memory cannot create a clocked owner");
}
}

int main() {
  try {
    test_literal_phase_and_callback_lifetime();
    test_operand_fault_and_coordinated_recovery();
    test_actual_mapping_qualifies_word_timing();
    test_timer_input_mask_and_history();
    test_native_sci_half_frame();
    test_active_guards_and_contract_compatibility();
    std::cout << "Coordinated MCU CPU/bus/run-clock checkpoints and independent replay passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
