// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_short_load_fixture.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::short_load;

void one(const Case sample) {
  DspMachine machine;
  const auto before = prepare(machine, sample);
  for (unsigned index = 1; index < 16; ++index) { machine.bus().write({.address = entry + index}); }
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  for (unsigned count = 1; count <= 7; ++count) { period(run, machine, before, sample, count); }
}

unsigned snapshots(const Case sample) {
  for (unsigned cut = 0; cut < 7; ++cut) {
    DspMachine machine;
    const auto before = prepare(machine, sample);
    for (unsigned index = 1; index < 16; ++index) {
      machine.bus().write({.address = entry + index});
    }
    static_cast<void>(
        machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
    DspFetchRun run(contract(), machine);
    for (unsigned count = 1; count <= cut; ++count) { period(run, machine, before, sample, count); }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      for (unsigned count = cut + 1; count <= 7; ++count) {
        period(run, machine, before, sample, count);
      }
    }
  }
  return 7;
}

unsigned variants(const unsigned target, const unsigned address, const bool y) {
  unsigned count = 0;
  for (const auto value : values) {
    for (const auto ccr : flags) {
      one({.destination = target, .address = address, .value = value, .ccr = ccr, .y = y});
      ++count;
    }
  }
  return count;
}

unsigned migration_cases() {
  constexpr unsigned former_ea_address = 32;
  constexpr unsigned former_scalar_address = 24;
  unsigned count = 0;
  for (unsigned target = 0; target < 8; ++target) {
    for (const bool y : {false, true}) {
      one({.destination = target,
           .address = former_ea_address,
           .value = values.back(),
           .ccr = flags.back(),
           .y = y});
      ++count;
    }
  }
  for (const bool y : {false, true}) {
    one({.destination = 1,
         .address = former_scalar_address,
         .value = values.back(),
         .ccr = flags.back(),
         .y = y});
    ++count;
  }
  return count;
}

unsigned full_addresses() {
  unsigned count = 0;
  for (unsigned target = data_begin; target < destination_count; ++target) {
    for (unsigned address = 0; address < last_cell; ++address) {
      if (std::ranges::contains(addresses, address)) { continue; }
      for (const bool y : {false, true}) {
        for (unsigned index = 0; index < 4; ++index) {
          for (const auto ccr : flags) {
            one({.destination = target,
                 .address = address,
                 .value = values[index],
                 .ccr = ccr,
                 .y = y});
            ++count;
          }
        }
      }
    }
  }
  return count;
}
}

int main(const int arguments, char **argv) {
  try {
    const std::string_view mode = arguments == 2 ? argv[1] : "--matrix";
    if (arguments > 2 || (mode != "--matrix" && mode != "--addresses")) {
      throw Fault("Usage: short-load-test [--matrix|--addresses]");
    }
    if (mode == "--addresses") {
      constexpr unsigned expected = 7680;
      constexpr unsigned expected_migrated = 18;
      require(full_addresses() == expected && migration_cases() == expected_migrated,
              "All remaining60 literals and every migrated ordinary guard form execute");
      std::cout << "7680 remaining AA6 data-address cases and18 migrated guard positives passed\n";
      return 0;
    }
    unsigned count = 0;
    unsigned cuts = 0;
    for (unsigned target = 0; target < destination_count; ++target) {
      for (const auto address : addresses) {
        for (const bool y : {false, true}) { count += variants(target, address, y); }
      }
      for (const auto address : {addresses.front(), addresses.back()}) {
        for (const bool y : {false, true}) {
          cuts += snapshots({.destination = target,
                             .address = address,
                             .value = values.back(),
                             .ccr = flags.back(),
                             .y = y});
        }
      }
    }
    constexpr unsigned expected_cases = 6912;
    constexpr unsigned expected_cuts = 1008;
    require(count == expected_cases && cuts == expected_cuts,
            "All36 destination/value/address/CCR cases and snapshot boundaries execute");
    std::cout << "6912 staged AA6 data/accumulator/R/N/M loads,1008cuts/2016suffixes,full "
                 "CPU/CCR/E1/E2/AGU/timing and130 cells passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
