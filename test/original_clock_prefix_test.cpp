// SPDX-License-Identifier: GPL-3.0-only
#include "clock_prefix_memory.hpp"
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_cop.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_run_clock.hpp"
#include "keyboard/hcs08_tpm.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::clock_prefix::Memory;
using blofeld::test::clock_prefix::Profile;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr unsigned records_bytes = 3598, partial_a = 0xfe, known_flags = 0xef;
constexpr unsigned initial_hx = 0x12c, main_stack = 0x475, reset_source = 0x80;
constexpr unsigned adc_failure_period = 22690, phase_failure_period = 22702;
constexpr unsigned adc_failure_pc = 0xe1bc, phase_failure_pc = 0xe1c5;
constexpr unsigned adc_failure_retired = 6193, phase_failure_retired = 6196;
constexpr unsigned clr_flags = 0x6b, positive_flags = 0x69;
constexpr unsigned adc_job = 176, phase_adc_remaining = 164;
constexpr unsigned threshold = 31250, threshold_pc = 0xe1d1, flash_pc = 0xea88;
constexpr unsigned flash_setup_periods = 37, flash_fault_periods = 41, flash_setup_retired = 10;
constexpr unsigned flash_divider = 0x3d, flash_accumulator = 0xff, flash_flags = 0x6c;
constexpr unsigned first_cut = 22671, bounded_end = 22700, strap = 8;
constexpr unsigned prefix_periods = 22697, prefix_retired = 6195;
constexpr unsigned first_status_pc = 0xe1b0, threshold_flags = 0x6a;

class NoDebugger final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(const ExecutionContext context) const override {
    throw Fault("Unqualified coupled BDC security query pc=" + hex(context.pc));
  }

  std::uint8_t read(const std::uint16_t address, const ExecutionContext context) override {
    throw Fault("Unqualified coupled BDC read address=" + hex(address) + " pc=" + hex(context.pc));
  }

  void write(const ByteWrite access, const ExecutionContext context) override {
    throw Fault("Unqualified coupled BDC write address=" + hex(access.address) +
                " pc=" + hex(context.pc));
  }
};

Hcs08 &normal_reset(Hcs08 &cpu) {
  cpu.reset(Hcs08ResetEntry::normal, Hcs08ResetCause::power_on);
  return cpu;
}

Hcs08 &restore(Hcs08 &cpu, const Hcs08Checkpoint &checkpoint) {
  cpu.restore_checkpoint(checkpoint);
  return cpu;
}

struct Saved {
  Memory memory;
  Hcs08Checkpoint cpu;
  Hcs08BdcCheckpoint bdc;
  Hcs08RunCheckpoint clock;
};

class Runner {
public:
  Memory memory;
  Hcs08 cpu;
  NoDebugger mapping;
  Hcs08Bdc bdc;
  Hcs08RunClock clock;

  Runner(const std::span<const std::uint8_t> pages, const Profile profile)
      : memory(pages, profile),
        cpu(memory, BLOFELD_HCS08_SLA),
        bdc(normal_reset(cpu), mapping, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::normal),
        clock(memory, bdc, Hcs08RunContract::reference_instruction_retirement) {}

  explicit Runner(const Saved &saved)
      : memory(saved.memory),
        cpu(memory, BLOFELD_HCS08_SLA),
        bdc(restore(cpu, saved.cpu),
            mapping,
            Hcs08FlashContract::candidate_gt8a,
            Hcs08BdcReset::normal),
        clock(memory, bdc, Hcs08RunContract::reference_instruction_retirement) {
    memory.core.data.bind(strap);
    bdc.restore_checkpoint(saved.bdc);
    clock.restore_checkpoint(saved.clock);
  }

  void advance() {
    if (memory.failed) { throw Fault("HCS08 coupled clock requires coordinated recovery"); }
    memory.devices(cpu.state());
    clock.advance();
    ++memory.completed;
    memory.failed = false;
  }

  void to(const unsigned period) {
    while (memory.completed < period) { advance(); }
  }

  [[nodiscard]] Saved save() const {
    require(!memory.failed, "Failed coupled clock cannot produce a successful checkpoint");
    return {.memory = memory,
            .cpu = cpu.checkpoint(),
            .bdc = bdc.checkpoint(),
            .clock = clock.checkpoint()};
  }
};

void quarantine(Runner &runner) {
  const auto attempted = runner.memory.attempted;
  const auto completed = runner.memory.completed;
  const auto cop = runner.memory.cop.state();
  const auto timer = runner.memory.core.timer.state();
  const auto adc = runner.memory.core.data.adc.state();
  const auto cpu = runner.cpu.state();
  require(
      rejected([&] { runner.advance(); }).contains("coordinated recovery") &&
          rejected([&] { static_cast<void>(runner.save()); }).contains("checkpoint") &&
          runner.memory.attempted == attempted && runner.memory.completed == completed &&
          runner.memory.cop.state() == cop && runner.memory.core.timer.state() == timer &&
          runner.memory.core.data.adc.state() == adc && runner.cpu.state() == cpu,
      "Device/CPU failure quarantines the whole owner without another edge or successful snapshot");
}

void missing_inputs(const std::span<const std::uint8_t> pages, const bool reference) {
  Runner runner(pages, {.phase = std::nullopt, .adc_reference = reference});
  const auto period = reference ? phase_failure_period : adc_failure_period;
  runner.to(period - 1);
  const auto failure = rejected([&] { runner.advance(); });
  const auto cpu = runner.cpu.state();
  require(
      failure.contains(reference ? "unknown prescaler phase" : "unknown bus/supply/reference") &&
          runner.memory.attempted == period && runner.memory.completed == period - 1 &&
          runner.clock.state().executing_periods == period - 1 &&
          runner.memory.core.data.retired ==
              (reference ? phase_failure_retired : adc_failure_retired) &&
          cpu ==
              Hcs08State{
                  .pc = static_cast<std::uint16_t>(reference ? phase_failure_pc : adc_failure_pc),
                  .hx = initial_hx,
                  .sp = main_stack,
                  .a = 0,
                  .ccr_known = known_flags,
                  .ccr = static_cast<std::uint8_t>(reference ? positive_flags : clr_flags),
                  .a_known = partial_a} &&
          runner.memory.cop.state().elapsed == period &&
          runner.memory.cop.state().reset_status == reset_source && runner.memory.services == 0 &&
          !runner.cpu.faulted() && !runner.clock.faulted() && runner.memory.failed,
      "Missing physical inputs fail at their first real device edge before any next CPU period");
  const auto adc = runner.memory.core.data.adc.state();
  require(adc.active && !adc.complete && !adc.result && adc.conversions == 0 &&
              adc.remaining == (reference ? phase_adc_remaining : adc_job) &&
              adc.reference.has_value() == reference &&
              !runner.memory.core.timer.state().prescaler_phase,
          "No ADC completion, analog result or timer phase is supplied to obtain progress");
  quarantine(runner);
}

void copied_prefix(Runner &baseline) {
  baseline.to(first_cut);
  require(baseline.clock.state().remaining_periods == 2 &&
              baseline.cpu.state().pc == first_status_pc,
          "Joint cut lies in the actual first ICG status-load instruction budget");
  const auto saved = baseline.save();
  baseline.to(bounded_end);
  Runner fresh(saved);
  fresh.to(bounded_end);
  const auto &first = baseline.memory.core.data;
  const auto &second = fresh.memory.core.data;
  require(baseline.cpu.state() == fresh.cpu.state() &&
              baseline.clock.state() == fresh.clock.state() &&
              baseline.memory.cop.state() == fresh.memory.cop.state() &&
              baseline.memory.core.timer.state() == fresh.memory.core.timer.state() &&
              first.adc.state() == second.adc.state() &&
              first.flash.image() == second.flash.image() && first.bytes == second.bytes &&
              first.known == second.known && first.partial_known == second.partial_known &&
              first.peripheral_accesses == second.peripheral_accesses &&
              baseline.memory.attempted == fresh.memory.attempted &&
              baseline.memory.completed == fresh.memory.completed,
          "Fresh joint CPU/RAM/BDC/clock/ICG/ADC/TPM/COP copy preserves the bounded native suffix");
}

struct Expected {
  unsigned phase, periods, retired, services, final_phase;
};

constexpr std::array<Expected, 2> references{
    {{.phase = 0, .periods = 4000013, .retired = 1142861, .services = 285714, .final_phase = 9},
     {.phase = 127,
      .periods = 3999887,
      .retired = 1142825,
      .services = 285705,
      .final_phase = 10}}};

void declared_inputs(const std::span<const std::uint8_t> pages, const Expected expected) {
  std::cout << "BEGIN phase=" << expected.phase << '\n' << std::flush;
  Runner runner(pages, {.phase = expected.phase, .adc_reference = true});
  const auto image = runner.memory.core.data.flash.image();
  copied_prefix(runner);
  runner.to(prefix_periods + expected.periods);
  require(runner.cpu.state() == Hcs08State{.pc = threshold_pc,
                                           .hx = threshold,
                                           .sp = main_stack,
                                           .a = 0,
                                           .ccr_known = known_flags,
                                           .ccr = threshold_flags,
                                           .a_known = partial_a} &&
              runner.memory.core.data.retired == prefix_retired + expected.retired &&
              runner.memory.services == expected.services &&
              runner.memory.service_known == partial_a &&
              runner.memory.core.timer.state().counter == threshold &&
              runner.memory.core.timer.state().prescaler_phase == expected.final_phase &&
              runner.memory.cop.state().elapsed == 10 &&
              runner.memory.cop.state().reset_status == reset_source,
          "Whole original reset and timer loop obey independent instruction/edge/COP oracles");
  const auto adc = runner.memory.core.data.adc.state();
  require(adc.complete && !adc.active && adc.result == 0 && adc.conversions == 1 &&
              adc.remaining == 0,
          "The declared stable analog input completes through actual supplied ADC bus periods");
  runner.to(prefix_periods + expected.periods + flash_setup_periods);
  require(runner.cpu.state() == Hcs08State{.pc = flash_pc,
                                           .hx = threshold,
                                           .sp = main_stack - 2,
                                           .a = flash_accumulator,
                                           .ccr_known = known_flags,
                                           .ccr = flash_flags} &&
              runner.memory.core.data.flash.state().divider == flash_divider,
          "Only actual original JSR/LDA/STA instructions reach the idle FSTAT transaction");
  const auto failure =
      rejected([&] { runner.to(prefix_periods + expected.periods + flash_fault_periods); });
  require(failure.contains("unqualified idle FCBEF launch") && failure.contains("pc=0xea88") &&
              runner.memory.core.data.retired ==
                  prefix_retired + expected.retired + flash_setup_retired &&
              runner.clock.state().executing_periods ==
                  prefix_periods + expected.periods + flash_fault_periods &&
              runner.memory.cop.state().elapsed == 10 + flash_fault_periods &&
              runner.memory.core.data.flash.image() == image && runner.cpu.faulted() &&
              runner.clock.faulted() && runner.memory.failed,
          "Full original prefix stops at real unsupported idle FSTAT without changing any firmware "
          "byte");
  quarantine(runner);
  std::cout << "PROFILE phase=" << expected.phase << " periods=" << runner.memory.attempted
            << " retired=" << runner.memory.core.data.retired
            << " services=" << runner.memory.services << " pc=" << runner.cpu.state().pc << '\n';
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Expected pinned original seven-page records");
    std::vector<std::uint8_t> pages(records_bytes);
    std::ifstream input(arguments[1], std::ios::binary);
    input.read(reinterpret_cast<char *>(pages.data()), static_cast<std::streamsize>(pages.size()));
    require(input.gcount() == static_cast<std::streamsize>(pages.size()) &&
                input.peek() == std::char_traits<char>::eof(),
            "Exact original record extent");
    missing_inputs(pages, false);
    missing_inputs(pages, true);
    for (const auto expected : references) { declared_inputs(pages, expected); }
    std::cout
        << "Qualified conditional original reset with real coupled ADC/TPM/COP bus periods.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
