// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_flash_fixture.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08_nvm;
constexpr unsigned bus_limit = 20000000, original_divider = 0x1e;
constexpr unsigned lower_flash_hz = 150000, upper_flash_hz = 200000;
constexpr unsigned read_only_loaded = 0x80;
constexpr unsigned original_lower_bus = 4650000, nominal_reset_bus = 4000000;

struct Counts {
  unsigned accepted = 0, rejected_clock = 0, rejected_bus = 0;
};

struct ClockCase {
  unsigned divider;
  unsigned rate;
  bool admissible;
};

Hcs08Flash prepared(const unsigned divider) {
  Hcs08Flash flash(initial(), contract);
  flash.context({.pc = ram_pc, .ccr = flags});
  flash.write({.address = hf::divider, .value = static_cast<std::uint8_t>(divider)});
  return flash;
}

void rate_case(const ClockCase sample, Counts &counts) {
  auto flash = prepared(sample.divider);
  const auto image = flash.image();
  if (sample.rate > bus_limit) {
    const auto before = flash.state();
    require(rejected([&] { flash.observe_bus_clock(sample.rate); }).contains("bus clock limit") &&
                flash.state() == before && flash.image() == image,
            "Clock qualification cannot exceed the candidate bus limit or mutate NVM");
    ++counts.rejected_bus;
    return;
  }
  flash.observe_bus_clock(sample.rate);
  flash.write({.address = hf::divider, .value = 0});
  require(flash.read(hf::divider) == (sample.divider | read_only_loaded),
          "Later FCDIV writes cannot repair a loaded divider; DIVLD is read-only");
  launch(flash, {});
  const auto pending = flash.state();
  require(pending.buffer == Hcs08FlashBuffer::launched && !pending.active && flash.image() == image,
          "Clock qualification follows a real buffered command, before NVM transfer");
  if (!sample.admissible) {
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
      const auto error = rejected([&] { flash.advance(transfer); });
      require(error.contains("outside150..200kHz") && error.contains("address=0x1820") &&
                  error.contains("pc=0x00a0") && error.contains("agent=CPU") &&
                  flash.state() == pending && flash.image() == image,
              "Out-of-range clocks refuse repeatedly with context and the command intact");
    }
    ++counts.rejected_clock;
    return;
  }
  flash.advance(transfer);
  const auto active = flash.state();
  require(active.buffer == Hcs08FlashBuffer::empty && active.active == pending.staged &&
              flash.image() == image && flash.read(hf::status) == 0,
          "Qualified transfer starts the actual command without completing it");
  require(rejected([&] { flash.observe_bus_clock(sample.rate); }).contains("during a command") &&
              flash.state() == active && flash.image() == image,
          "Even a repeated sample cannot waive unqualified active-clock changes");
  ++counts.accepted;
}

Counts divider_matrix() {
  Counts counts;
  for (unsigned code = 0; code < 128; ++code) {
    // Independent reading of GT4.6.1: codes64..127 select /8 before /1../64.
    const auto factor = code < 64 ? code + 1 : (code - 64 + 1) * 8;
    const auto lower = lower_flash_hz * factor;
    const auto upper = upper_flash_hz * factor;
    const std::array<unsigned, 6> rates{lower - 1, lower, lower + 1, upper - 1, upper, upper + 1};
    for (const auto loaded_write : std::array<unsigned, 2>{0, read_only_loaded}) {
      for (unsigned sample = 0; sample < rates.size(); ++sample) {
        rate_case({.divider = code | loaded_write,
                   .rate = rates[sample],
                   .admissible = sample != 0 && sample != 5},
                  counts);
      }
    }
  }
  return counts;
}

void original_rate_limits() {
  // Candidate reset BUSCLK limits are ICGOUT5.5..10.5MHz /2, not measured rates.
  // The 6MHz fixture qualifies FLASH but lies above that reset interval.
  constexpr std::array<unsigned, 11> rates{2750000,
                                           4000000,
                                           4649999,
                                           4650000,
                                           4800000,
                                           5250000,
                                           6000000,
                                           6200000,
                                           6200001,
                                           20000000,
                                           20000001};
  constexpr std::array<bool, 11> expected{
      false, false, false, true, true, true, true, true, false, false, false};
  Counts counts;
  for (unsigned n = 0; n < rates.size(); ++n) {
    rate_case({.divider = original_divider, .rate = rates[n], .admissible = expected[n]}, counts);
  }
  require(counts.accepted == 5 && counts.rejected_clock == 5 && counts.rejected_bus == 1,
          "Literal original-divider rate expectations retain both candidate and fixture limits");
}

void unknown_and_reset() {
  auto flash = prepared(original_divider);
  require(rejected([&] { flash.observe_bus_clock(0); }).contains("bus clock limit"),
          "Zero is not an observed oscillator rate");
  launch(flash, {});
  const auto before = flash.state();
  const auto image = flash.image();
  require(rejected([&] { flash.advance(transfer); }).contains("unknown operating clock") &&
              flash.state() == before && flash.image() == image,
          "A divider, reset profile name and pending command cannot establish a frequency");
  require(
      rejected([&] { flash.observe_bus_clock(original_lower_bus); }).contains("during a command") &&
          flash.state() == before && flash.image() == image,
      "Late clock observation cannot qualify an already launched command");
  flash.reset(Hcs08FlashAbort::reference_command_range);
  flash.write({.address = hf::divider, .value = original_divider});
  flash.observe_bus_clock(original_lower_bus);
  launch(flash, {});
  const auto saved = flash;
  for (unsigned replay = 0; replay < 2; ++replay) {
    auto restored = saved;
    restored.advance(transfer);
    require(restored.state().active == before.staged && restored.image() == image,
            "An independent FLASH checkpoint retains the explicitly observed rate");
  }
  flash.reset(Hcs08FlashAbort::reference_command_range);
  require(flash.state() == Hcs08FlashState{} && flash.image() == image,
          "Reset drops volatile state; a queued untransferred reference command changed no NVM");
  flash.write({.address = hf::divider, .value = original_divider});
  launch(flash, {});
  const auto reset_pending = flash.state();
  require(rejected([&] { flash.advance(transfer); }).contains("unknown operating clock") &&
              flash.state() == reset_pending && flash.image() == image,
          "Reset must discard the former rate even when the same FCDIV is loaded again");
  flash.reset(Hcs08FlashAbort::reference_command_range);
  flash.write({.address = hf::divider, .value = original_divider});
  flash.observe_bus_clock(nominal_reset_bus);
  launch(flash, {});
  require(rejected([&] { flash.advance(transfer); }).contains("outside150..200kHz") &&
              flash.state() == reset_pending && flash.image() == image,
          "Explicit nominal reset rate cannot inherit a previously valid programming rate");
}
} // namespace

int main() {
  try {
    const auto counts = divider_matrix();
    constexpr Counts expected{.accepted = 624, .rejected_clock = 312, .rejected_bus = 600};
    require(counts.accepted == expected.accepted &&
                counts.rejected_clock == expected.rejected_clock &&
                counts.rejected_bus == expected.rejected_bus,
            "All1536 independent divider endpoints and adjacent integer rates are classified");
    original_rate_limits();
    unknown_and_reset();
    std::cout << "HCS08 FLASH clock:128 dividers, both DIVLD write values,1536 boundary cases; "
              << counts.accepted << " transfers, " << counts.rejected_clock << " NVM refusals, "
              << counts.rejected_bus << " bus-limit refusals; original-divider and reset controls "
              << "passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
