// SPDX-License-Identifier: GPL-3.0-only
// Execute unchanged original workspace routines with explicit native callers.
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_workspace_fixture.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <exception>
#include <fstream>
#include <ios>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace blofeld::test::workspace {
namespace {
constexpr unsigned original_records = 127;
constexpr unsigned oscillator = 12000000;
constexpr unsigned timebase = 540000000;
constexpr unsigned x_low = 0x3000;
constexpr unsigned y_low = 0x2000;
constexpr unsigned high_begin = 0xc000;
constexpr unsigned x_high = 0x10000;
constexpr unsigned y_high = 0x16000;

DspRunContract contract() {
  return {.clock = {.extal_hz = oscillator,
                    .timebase_hz = timebase,
                    .switch_timing = DspClockSwitch::reference_instruction_boundary},
          .timing = DspRunTiming::native_internal_estimates};
}

Case parse(const std::string &line) {
  std::istringstream stream(line);
  std::vector<unsigned> values;
  for (unsigned value = 0; stream >> std::hex >> value;) {
    require(value <= mask, "Workspace fixture word exceeds 24 bits");
    values.push_back(value);
  }
  constexpr unsigned fields = 7;
  require(stream.eof() && values.size() >= fields && values.size() == fields + (2 * values[6]),
          "Malformed workspace case");
  Case result{.id = values[0],
              .entry = values[1],
              .mode = values[2],
              .request = values[3],
              .selected = values[4],
              .x0 = values[5],
              .writes = {}};
  require(result.entry == allocate || result.entry == remove_entry || result.entry == cleanup,
          "Unknown workspace entry");
  constexpr unsigned rounding_mode = 0x200000;
  require(result.mode == 0 || result.mode == rounding_mode, "Unsupported workspace SR mode");
  if (result.entry == allocate) {
    constexpr unsigned request_unit = 0x200;
    require(result.request >= request_unit && result.request <= 6 * request_unit &&
                result.request % request_unit == 0,
            "Unsupported workspace allocation request");
  }
  for (unsigned i = fields; i < values.size(); i += 2) {
    result.writes.emplace_back(values[i], values[i + 1]);
  }
  return result;
}
}

Source::Source(const char *path) : firmware(Firmware::load(path)) {
  require(firmware.dsp.size() == original_records, "All 127 ordered original records are supplied");
  std::set<unsigned> overwritten;
  for (const auto &record : firmware.dsp) {
    if (record.space != 0) { continue; }
    for (unsigned i = 0; i < record.words.size(); ++i) {
      if (program.contains(record.address + i)) { overwritten.insert(record.address + i); }
      program[record.address + i] = record.words[i];
    }
  }
  constexpr unsigned second = 0x11;
  require(overwritten == std::set<unsigned>{16, second}, "Unexpected original record replacement");
}

Fixture::Fixture(const Source &source, std::ostream &trace)
    : machine_(contract()), source_(source), trace_(trace) {
  auto &bus = machine_.bus();
  bus.observe_omr(memory_mode);
  bus.advance_instruction_cycles(3);
  constexpr std::array<std::array<unsigned, 3>, 4> ranges{
      {{1, 0, x_low}, {2, 0, y_low}, {1, high_begin, x_high}, {2, high_begin, y_high}}};
  for (const auto &[space, first, end] : ranges) {
    for (unsigned at = first; at < end; ++at) {
      bus.write({.space = static_cast<DspSpace>(space), .address = at, .value = 0});
    }
  }
  for (const auto &record : source.firmware.dsp) {
    for (unsigned i = 0; i < record.words.size(); ++i) {
      bus.write({.space = static_cast<DspSpace>(record.space),
                 .address = record.address + i,
                 .value = record.words[i]});
    }
  }
  caller();
  bus.pll().observe_pinit(false);
  machine_.cpu().reset_spi_boot();
}

void Fixture::caller() {
  DspProgram code;
  code.immediate("x1", memory_mode);
  code.emit("move x1,omr");
  for (unsigned i = 0; i < 3; ++i) { code.emit("nop"); }
  begin_ = code.pc();
  for (unsigned i = 0; i < 8; ++i) {
    code.immediate("m" + std::to_string(i), mask);
    constexpr unsigned note_stride = 0x53;
    code.immediate("n" + std::to_string(i), i == 0 ? note_stride : 0);
  }
  unsigned slot = argument_slot;
  for (const auto *const name : {"a2", "a1", "a0", "b2", "b1", "b0", "x0", "y0", "y1"}) {
    code.emit("move x:>" + number(slot++) + ',' + name);
  }
  code.emit("move x:>" + number(mode_slot) + ",x1");
  code.emit("move x1,sr");
  code.emit("move x:>" + number(argument_slot + 11) + ",x1");
  code.emit("move x:>" + number(argument_slot + 9) + ",r0");
  code.immediate("r2", voice);
  code.emit("move x:>" + number(argument_slot + 13) + ",r5");
  code.emit("move x:>" + number(argument_slot + 12) + ",r3");
  code.immediate("r4", target_slot);
  code.immediate("r6", 0);
  code.immediate("r7", 0);
  code.emit("move x:>" + number(target_slot) + ",r1");
  code.emit("jsr (r1)");
  returned_ = code.pc();
  code.emit("nop");
  constexpr unsigned absolute_jump = 0x0af080;
  code.word(absolute_jump);
  code.word(begin_);
  code.install(machine_.bus());
}

void Fixture::put(const unsigned address, const unsigned value) {
  machine_.bus().write({.space = DspSpace::x, .address = address, .value = value});
}

unsigned Fixture::get(const unsigned address) {
  return machine_.bus().read({.space = DspSpace::x, .address = address});
}

void Fixture::report() const {
  trace_ << "CALLER_RETURN " << std::hex << returned_ << '\n';
  trace_ << "VISITED" << std::hex;
  for (const auto pc : visited_) { trace_ << ' ' << pc; }
  trace_ << "\nTOTAL " << std::dec << calls_ << "\nSNAPSHOTS " << snapshots_ << ' ' << replays_
         << '\n';
}
}

int main(const int count, char **values) {
  try {
    using namespace blofeld::test::workspace;
    blofeld::test::require(count == 4, "Expected original firmware, cases and private trace");
    const Source source(values[1]);
    std::ifstream cases(values[2]);
    std::ofstream trace(values[3]);
    blofeld::test::require(cases.is_open() && trace.is_open(),
                           "Cannot open workspace fixture files");
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    Fixture fixture(source, trace);
    for (std::string line; std::getline(cases, line);) { fixture.run(parse(line)); }
    blofeld::test::require(cases.eof(), "Workspace corpus read failed");
    fixture.report();
    trace.flush();
    trace.close();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
