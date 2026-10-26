// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_loop_source.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned guards = 4;
constexpr unsigned setup = pipeline::body + guards;
constexpr unsigned first = setup + 2;
constexpr unsigned body_words = 3;
constexpr unsigned register_do = 0x06c000;
constexpr unsigned forbidden_ssh = 0x3c;
constexpr unsigned pending_sp = 0x3b;
constexpr unsigned pending_ssl = 0x3d;
constexpr unsigned pending_lc = 0x3f;
constexpr unsigned unqualified_x1 = 5;
constexpr unsigned unqualified_r1 = 17;
constexpr unsigned above_word = 0x106c400;
constexpr unsigned maximum_clocks = 12;

void unsupported(const unsigned source) {
  const auto text = "do x0," + number(first + body_words);
  std::vector<const char *> code(guards, "nop");
  code.push_back(text.c_str());
  code.insert(code.end(), body_words, "nop");
  DspMachine machine;
  pipeline::prepare(machine, code);
  // Replace only our own test instruction before its first actual F1/F2.
  machine.bus().write({.address = setup, .value = register_do | (source << 8)});
  auto input = pipeline::contract();
  input.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(input, machine);
  for (unsigned clock = 1; clock <= maximum_clocks; ++clock) {
    const auto before = machine.cpu().debug_state();
    try {
      static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
    } catch (const Fault &error) {
      const std::string message = error.what();
      require(clock == 3 + guards && run.faulted() && machine.cpu().debug_state() == before &&
                  message.contains("Unqualified automatic DSP decode P:" + hex(setup, 6)) &&
                  message.contains("pc=" + hex(setup, 6)) && message.contains("omr="),
              "Unsupported source refuses at actual Decode with context before a native DO effect");
      return;
    }
  }
  throw Fault("Unsupported automatic DO count source was silently admitted");
}
}

int main() {
  try {
    require(!automatic_loop_source(above_word),
            "Non-24-bit words cannot alias an admitted DO form");
    for (const auto source : std::array{
             forbidden_ssh, pending_sp, pending_ssl, pending_lc, unqualified_x1, unqualified_r1}) {
      unsupported(source);
    }
    std::cout
        << "Automatic DO source refusals preserve actual CPU state and report Decode context\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
