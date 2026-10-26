// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_memory_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test::peripheral_automatic {
namespace {
enum class Scenario : std::uint8_t { transmit_status, clock_control, control_forward };
enum class Reader : std::uint8_t { word, set, clear };
constexpr unsigned hckr = 0xffff90;
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned htx = 0xffff93;
constexpr unsigned baseline_status = 0x4082a9;
constexpr unsigned settled_status = 0x4002a9;
constexpr unsigned first_word = 0x123456;
constexpr unsigned second_word = 0x654321;
constexpr unsigned control_word = 0xa9;
constexpr unsigned quiet = 8;
constexpr unsigned maximum_clocks = 96;
constexpr unsigned body = memory_pipeline::body;

struct Window {
  Scenario scenario = Scenario::transmit_status;
  Reader reader = Reader::word;
  bool baseline = false;
  bool immediate = false;
  unsigned gap = 0;
};

unsigned port(const Scenario scenario) {
  return scenario == Scenario::clock_control ? hckr : hcsr;
}

unsigned source(const Scenario scenario) {
  switch (scenario) {
    case Scenario::transmit_status: return second_word;
    case Scenario::clock_control: return 3;
    case Scenario::control_forward: return control_word;
    default: throw Fault("Invalid automatic source scenario");
  }
}

bool old_source(const Window window) {
  return window.baseline || (window.immediate ? 2U : 1U) + window.gap <= 2;
}

unsigned expected_word(const Window window) {
  if (window.scenario == Scenario::clock_control) { return old_source(window) ? 1U : 3U; }
  if (window.scenario == Scenario::control_forward) { return baseline_status; }
  return old_source(window) ? baseline_status : settled_status;
}

bool expected_bit(const Window window) {
  if (window.scenario == Scenario::clock_control) { return !window.baseline; }
  if (window.scenario == Scenario::control_forward) {
    return window.baseline || !old_source(window);
  }
  return old_source(window);
}

std::string read_instruction(const Window window, const unsigned target) {
  const auto operand = "x:<<" + number(port(window.scenario));
  if (window.reader == Reader::word) { return "movep " + operand + ",x0"; }
  const auto bit = window.scenario == Scenario::clock_control ? 1U : 15U;
  return std::string(window.reader == Reader::set ? "jset #" : "jclr #") + std::to_string(bit) +
         ',' + operand + ',' + number(target);
}

struct Program {
  std::vector<std::string> code;
  unsigned producer = body + quiet;
  unsigned reader = 0;
  unsigned target = 0;
};

Program program(const Window window) {
  Program result;
  result.code.insert(result.code.end(), quiet, "nop");
  const auto address = window.scenario == Scenario::transmit_status ? htx : port(window.scenario);
  const auto operand = window.immediate ? "#>" + number(source(window.scenario)) : "r0";
  result.code.push_back(window.baseline ? "nop" : "movep " + operand + ",x:<<" + number(address));
  result.code.insert(result.code.end(), window.gap, "nop");
  const unsigned cost = !window.baseline && window.immediate ? 2U : 1U;
  result.reader = result.producer + cost + window.gap;
  result.target = result.reader + (window.reader == Reader::word ? 1U : 4U);
  result.code.push_back(read_instruction(window, result.target));
  if (window.reader != Reader::word) { result.code.emplace_back("move #>$654321,y0"); }
  result.code.insert(result.code.end(), quiet, "nop");
  return result;
}

struct Anchors {
  std::uint64_t before = 0;
  std::uint64_t after = 0;
};

Anchors run_window(DspFetchRun &run, DspMachine &machine, const Program &code) {
  std::optional<std::uint64_t> before;
  std::optional<std::uint64_t> after;
  for (unsigned clock = 0; clock < maximum_clocks && !after; ++clock) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &stages = memory_pipeline::stages(cycle);
    if (!stages.retirement_published || !stages.execute2) { continue; }
    const auto pc = stages.execute2->address;
    if (pc == code.producer - 1) { before = cycle.completed_cycles; }
    if (pc == code.target + 5) { after = cycle.completed_cycles; }
  }
  if (!before || !after) {
    throw Fault("Automatic peripheral stream lost its quiet retirement anchors");
  }
  return {.before = *before, .after = *after};
}

unsigned settled_word(const Window window) {
  if (window.scenario == Scenario::clock_control) { return window.baseline ? 1U : 3U; }
  if (window.scenario == Scenario::control_forward || window.baseline) { return baseline_status; }
  return settled_status;
}

void one_window(const Window window) {
  const auto code = program(window);
  DspMachine machine;
  memory_pipeline::prepare(machine, code.code, {.pointer = source(window.scenario)});
  auto expected = machine.cpu().debug_state();
  machine.bus().shi().observe_spi({.mosi = false});
  if (window.scenario != Scenario::clock_control) {
    machine.bus().write({.space = DspSpace::x, .address = hcsr, .value = control_word});
    machine.bus().write({.space = DspSpace::x, .address = htx, .value = first_word});
    machine.bus().shi().observe_spi({.selected = true, .mosi = false});
  }
  DspFetchRun run(memory_pipeline::contract(), machine);
  const auto anchors = run_window(run, machine, code);
  const bool branch = window.reader != Reader::word;
  const bool taken = branch && expected_bit(window) == (window.reader == Reader::set);
  const bool marker = branch && !taken;
  const unsigned producer_cost = !window.baseline && window.immediate ? 2U : 1U;
  const unsigned branch_cost = taken ? 4U : 5U;
  const unsigned reader_cost = branch ? branch_cost : 1U;
  const unsigned clocks = producer_cost + window.gap + reader_cost + (marker ? 2U : 0U) + 6;
  const auto context = " scenario=" + std::to_string(static_cast<unsigned>(window.scenario)) +
                       " reader=" + std::to_string(static_cast<unsigned>(window.reader)) +
                       " immediate=" + std::string(window.immediate ? "1" : "0") +
                       " baseline=" + std::string(window.baseline ? "1" : "0") +
                       " gap=" + std::to_string(window.gap);
  require(anchors.after - anchors.before == clocks,
          "Independent automatic quiet window expected=" + std::to_string(clocks) +
              " observed=" + std::to_string(anchors.after - anchors.before) + context);
  expected.cpu.pc = code.target + 6;
  if (!branch) {
    expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | expected_word(window);
  }
  if (marker) { expected.cpu.y = (expected.cpu.y & ~std::uint64_t{word_mask}) | second_word; }
  expected.native_instructions += 16 + window.gap + (marker ? 1U : 0U);
  expected.native_cycles +=
      14 + producer_cost + window.gap + (branch ? 4U : 1U) + (marker ? 2U : 0U);
  require(
      machine.cpu().debug_state() == expected,
      "Automatic native registers/flags/stack/PC/counters differ from independent source windows" +
          context);
  const auto settled = settled_word(window);
  require(machine.bus().read({.space = DspSpace::x, .address = port(window.scenario)}) == settled,
          "Automatic source admission changed settled device state" + context);
}

void visibility() {
  for (const auto scenario :
       {Scenario::transmit_status, Scenario::clock_control, Scenario::control_forward}) {
    for (const auto reader : {Reader::word, Reader::set, Reader::clear}) {
      one_window({.scenario = scenario, .reader = reader, .baseline = true});
      for (const bool immediate : {false, true}) {
        for (unsigned gap = 0; gap < 3; ++gap) {
          one_window({.scenario = scenario, .reader = reader, .immediate = immediate, .gap = gap});
        }
      }
    }
  }
}
}
}

int main(const int argc, const char *const argv[]) {
  try {
    blofeld::test::require(argc == 2, "Select automatic peripheral visibility");
    if (std::string_view(argv[1]) == "visibility") {
      blofeld::test::peripheral_automatic::visibility();
    } else {
      throw blofeld::Fault("Unknown automatic peripheral read group");
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
