// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_short_load_fixture.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::short_load;

void accesses(const std::vector<DspNativeAccess> &events, const Case sample) {
  require(events.size() == 4,
          "Exactly one opcode and one data read have before/after observations");
  for (unsigned index = 0; index < events.size(); ++index) {
    const auto &event = events[index];
    const bool instruction = index < 2;
    const bool after = (index % 2) != 0;
    const auto kind = instruction ? DspNativeAccessKind::opcode : DspNativeAccessKind::read;
    auto space = DspSpace::p;
    if (!instruction) { space = sample.y ? DspSpace::y : DspSpace::x; }
    const auto address = instruction ? entry : sample.address;
    require(event.kind == kind &&
                event.stage ==
                    (after ? DspNativeAccessStage::after : DspNativeAccessStage::before) &&
                event.target.space == space && event.target.address == address &&
                event.ordinal == index / 2 && event.pc == entry && event.omr == ordinary_omr &&
                !event.bit && event.value.has_value() == after,
            "Native access trace has exact operation,address,ordinal,PC/OMR and no hidden "
            "write/extension");
    if (after) {
      require(event.value == (instruction ? opcode(sample) : sample.value),
              "Native observed read value matches independent input");
    }
  }
}

void one(const Case sample) {
  DspMachine machine;
  auto expected = prepare(machine, sample);
  destination(expected.cpu, sample);
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles; // FM A-6:MOVEX/Y:aa,D and MOVEC:aa,D1 each have nominal T1.
  std::vector<DspNativeAccess> events;
  events.reserve(4);
  machine.cpu().step(
      {.observe = [&events](const DspNativeAccess &event) { events.push_back(event); }});
  accesses(events, sample);
  const auto actual = machine.cpu().debug_state();
  if (actual != expected) {
    std::cerr << name(sample.destination) << " word=" << hex(opcode(sample), 6)
              << " value=" << hex(sample.value, 6) << " pc=" << hex(actual.cpu.pc, 6) << '/'
              << hex(expected.cpu.pc, 6) << " sr=" << hex(actual.cpu.sr, 6) << '/'
              << hex(expected.cpu.sr, 6) << " A=" << hex(actual.cpu.a, 14) << '/'
              << hex(expected.cpu.a, 14) << " B=" << hex(actual.cpu.b, 14) << '/'
              << hex(expected.cpu.b, 14) << " cycles=" << actual.native_cycles << '/'
              << expected.native_cycles << '\n';
  }
  require(actual == expected,
          "Independent full CPU/CCR oracle changes only the actual36-way load destination,PC and "
          "one SDK clock");
  memory(machine, sample);
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
}

int main() {
  try {
    unsigned count = 0;
    for (unsigned target = 0; target < destination_count; ++target) {
      for (const auto address : addresses) {
        for (const bool y : {false, true}) {
          const Case encoding{.destination = target, .address = address, .y = y};
          std::cout << "AA6-encoding " << std::to_string(target) << ' ' << std::to_string(address)
                    << ' ' << (y ? "1" : "0") << ' ' << hex(opcode(encoding), 6) << '\n';
          count += variants(target, address, y);
        }
      }
    }
    constexpr unsigned expected_cases = 6912;
    require(count == expected_cases,
            "All36 native AA6 destinations execute their complete directed matrix");
    std::cout
        << "6912 native AA6 data/accumulator/R/N/M loads,full CPU/CCR/part preservation,one-cycle "
           "cost and130 RAM cells passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
