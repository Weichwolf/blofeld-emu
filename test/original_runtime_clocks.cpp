// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pll.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_clock.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <ostream>
#include <string>
#include <string_view>

namespace {
constexpr unsigned oscillator = 12000000;
constexpr unsigned full_timebase = 1080000000;
constexpr unsigned short_timebase = 540000000;
constexpr unsigned entry = 0x100;
constexpr unsigned linear = 0x10b;
constexpr unsigned linear_next = 0x10d;
constexpr unsigned repeated_nop = 0x108;
constexpr unsigned delay_count = 0xabe0;
constexpr unsigned instruction_limit = delay_count + 128;
constexpr unsigned memory_mode = 0x600389;
constexpr unsigned phy_poll = 0x8049;
constexpr unsigned first_loopback = 0x8029;
constexpr unsigned second_loopback = 0x8036;
constexpr unsigned boot_limit = 128;
constexpr unsigned boot_inputs = 0xfb;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned select_clock = 0x804d;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw blofeld::Fault(std::string(message)); }
}

void mcu_prefix(const blofeld::Firmware &firmware, std::ostream &trace) {
  blofeld::McuMachine machine(
      blofeld::McuFlashImage::from_known(firmware.mcu),
      BLOFELD_DEFAULT_SLA,
      blofeld::McuRunContract{
          .clock = {.oscillator_hz = oscillator,
                    .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = blofeld::McuRunMemory::internal_flash_reads});
  auto &bus = machine.bus();
  const auto initial = bus.power_off_flash_image();
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = boot_inputs, .levels = 3});
  bool observed_phy = false;
  unsigned instructions = 0;
  machine.set_cycle_observer([&](const blofeld::Hc12BusCycle cycle) {
    if (cycle.context.pc != select_clock) { return; }
    trace << "MCU_SLOT kind=" << cycle.kind << " prior-cycles=" << machine.run_clock().bus_cycles()
          << " prior-ticks=" << machine.run_clock().elapsed_ticks()
          << " source=" << static_cast<unsigned>(bus.run_clock_route().source) << '\n';
  });
  while (bus.run_clock_route().source != blofeld::McuClockSource::usb_phy) {
    require(instructions < boot_limit, "Original MCU clock-prefix instruction bound");
    const auto pc = machine.cpu().state().pc;
    if (pc == first_loopback || pc == second_loopback) {
      const auto pad = bus.observe_pads(blofeld::McuPort::e, 4);
      require(pad.known == 4, "E2 must be known before the declared E2/E3 loopback");
      bus.set_pin_drive(
          blofeld::McuPort::e,
          {.mask = boot_inputs,
           .levels = static_cast<std::uint8_t>(3U | ((pad.levels & 4U) != 0 ? 8U : 0U))});
    }
    if (pc == phy_poll && !observed_phy) {
      bus.observe_phy({.edges = 15});
      observed_phy = true;
    }
    machine.cpu().step();
    ++instructions;
  }
  const auto final = bus.power_off_flash_image();
  require(observed_phy && initial.bytes == final.bytes && initial.known == final.known &&
              initial.fresh == final.fresh && !machine.timing_faulted(),
          "Original MCU clock selection must retain NVM and a healthy executed prefix");
  trace << "MCU pc=" << machine.cpu().state().pc << " instructions=" << instructions
        << " cycles=" << machine.run_clock().bus_cycles()
        << " ticks=" << machine.run_clock().elapsed_ticks()
        << " timebase=" << machine.run_clock().timebase_hz()
        << " divider=" << bus.run_clock_route().bus_divider << '\n';
}

struct Row {
  std::uint64_t units = 0;
  std::uint64_t period = 0;
  unsigned pctl_before = 0;
  unsigned pctl_after = 0;
  unsigned count = 0;
};

struct Runner {
  blofeld::DspMachine machine;
  const blofeld::DspRecord &record;
  std::map<unsigned, Row> rows;
  unsigned steps = 0;

  Runner(const blofeld::DspRecord &component, const unsigned timebase)
      : machine(blofeld::DspRunContract{
            .clock = {.extal_hz = oscillator,
                      .timebase_hz = timebase,
                      .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
            .timing = blofeld::DspRunTiming::native_internal_estimates}),
        record(component) {
    machine.bus().pll().observe_pinit(false);
    machine.cpu().reset_spi_boot();
    for (unsigned index = 0; index < record.words.size(); ++index) {
      machine.bus().write({.address = record.address + index, .value = record.words[index]});
    }
    // Own native entry only; original prefix executes at its unchanged P addresses.
    blofeld::test::DspProgram caller;
    caller.immediate("omr", memory_mode);
    for (unsigned index = 0; index < 3; ++index) { caller.emit("nop"); }
    caller.word(absolute_jump);
    caller.word(entry);
    caller.install(machine.bus());
    for (unsigned index = 0; machine.cpu().state().pc != entry; ++index) {
      require(index < boot_limit, "Own DSP entry instruction bound");
      machine.cpu().step();
    }
  }

  void step() {
    require(steps++ < instruction_limit, "Original DSP clock-prefix instruction bound");
    const auto pc = machine.cpu().state().pc;
    const auto before = machine.bus().pll().read();
    const auto units = machine.run_clock().native_cycle_units();
    const auto ticks = machine.run_clock().elapsed_ticks();
    machine.cpu().step();
    const auto used = machine.run_clock().native_cycle_units() - units;
    const auto elapsed = machine.run_clock().elapsed_ticks() - ticks;
    require(used != 0 && elapsed % used == 0, "Each original instruction needs an integral period");
    auto &row = rows[pc];
    const auto after = machine.bus().pll().read();
    if (row.count == 0) {
      row = {.units = used, .period = elapsed / used, .pctl_before = before, .pctl_after = after};
    }
    require(row.units == used && row.period == elapsed / used && row.pctl_before == before &&
                row.pctl_after == after,
            "Original repeated instruction retains its clock and native cost");
    ++row.count;
  }

  void until(const unsigned pc) {
    while (machine.cpu().state().pc != pc) { step(); }
  }

  void unchanged_program() const {
    for (unsigned index = 0; index < record.words.size(); ++index) {
      require(machine.bus().peek_program(record.address + index) == record.words[index],
              "Every original component word remains unchanged");
    }
  }
};

void state(const std::string_view label, const Runner &runner, std::ostream &trace) {
  const auto cpu = runner.machine.cpu().state();
  trace << label << " pc=" << cpu.pc
        << " cycles=" << runner.machine.run_clock().native_cycle_units()
        << " ticks=" << runner.machine.run_clock().elapsed_ticks()
        << " pctl=" << runner.machine.bus().pll().read() << " x0=" << (cpu.x & word_mask)
        << " m0=" << cpu.m[0] << " sr=" << cpu.sr << '\n';
}

void census(const Runner &runner, std::ostream &trace) {
  for (const auto &[pc, row] : runner.rows) {
    trace << "ROW pc=" << pc << " units=" << row.units << " period=" << row.period
          << " before=" << row.pctl_before << " after=" << row.pctl_after << " count=" << row.count
          << '\n';
  }
}

void positive(const blofeld::DspRecord &record, std::ostream &trace) {
  Runner runner(record, full_timebase);
  state("BEGIN", runner, trace);
  runner.until(repeated_nop);
  for (unsigned index = 0; index < delay_count / 2; ++index) { runner.step(); }
  const auto saved = runner.machine.checkpoint();
  const auto saved_rows = runner.rows;
  const auto saved_steps = runner.steps;
  state("PARTIAL", runner, trace);
  runner.until(linear_next);
  const auto final = runner.machine.cpu().debug_state();
  const auto final_cycles = runner.machine.run_clock().native_cycle_units();
  const auto final_ticks = runner.machine.run_clock().elapsed_ticks();
  state("END", runner, trace);
  census(runner, trace);
  runner.machine.restore(saved);
  runner.rows = saved_rows;
  runner.steps = saved_steps;
  runner.until(linear_next);
  require(runner.machine.cpu().debug_state() == final &&
              runner.machine.run_clock().native_cycle_units() == final_cycles &&
              runner.machine.run_clock().elapsed_ticks() == final_ticks,
          "Mid-REP restoration retains actual CPU state and both subsequent rate changes");
  state("REPLAY", runner, trace);
  runner.unchanged_program();
}

void insufficient_timebase(const blofeld::DspRecord &record, std::ostream &trace) {
  Runner runner(record, short_timebase);
  runner.until(linear);
  const auto saved = runner.machine.checkpoint();
  const auto before = runner.machine.cpu().debug_state();
  const auto ticks = runner.machine.run_clock().elapsed_ticks();
  state("SHORT", runner, trace);
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    std::string failure;
    try {
      runner.machine.cpu().step();
    } catch (const blofeld::Fault &fault) { failure = fault.what(); }
    require(failure.contains("not integral") && failure.contains("pc=0x00010b") &&
                runner.machine.timing_faulted() && runner.machine.cpu().debug_state() == before &&
                runner.machine.run_clock().elapsed_ticks() == ticks,
            "A nonintegral clock fails before executing the next original instruction");
    trace << "FAULT " << failure << '\n';
    runner.machine.restore(saved);
    runner.unchanged_program();
  }
}

void run(const blofeld::Firmware &firmware, std::ostream &trace) {
  mcu_prefix(firmware, trace);
  for (const auto &record : firmware.dsp) {
    if (record.space == 0 && record.address == entry) {
      positive(record, trace);
      insufficient_timebase(record, trace);
      return;
    }
  }
  throw blofeld::Fault("Missing original runtime clock component");
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected original firmware archive and private clock trace");
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    run(firmware, trace);
    std::cout << "Original clock prefixes executed; independent ledger validation is separate.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
