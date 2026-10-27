// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_long_store_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>

namespace blofeld::test::long_saturation {
namespace {
void reserved_case(const bool b, const unsigned reg) {
  DspMachine machine;
  prepare(machine, {.value = 1, .scale = scale_up | scale_down, .mode = 3, .reg = reg, .b = b});
  const auto before = machine.cpu().debug_state();
  auto contract = fetch_clock::contract();
  contract.execution = execution;
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  for (unsigned index = 0; index < 2; ++index) {
    static_cast<void>(run.advance_cycle(machine, fetch_clock::input()));
  }
  const auto diagnostic = checkpoint::rejects(
      [&] { static_cast<void>(run.advance_cycle(machine, fetch_clock::input())); },
      "Reserved full-accumulator scaling must refuse before native AGU or memory effects");
  require(machine.cpu().debug_state() == before && diagnostic.contains("scaling read P:") &&
              diagnostic.contains("pc=") && diagnostic.contains("omr="),
          "Reserved scaling refusal must retain the complete entry state and hardware context");
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    require(machine.bus().read({.space = space, .address = data}) == canary,
            "Reserved scaling must not create either X or Y store");
  }
}
}
}

int main() {
  try {
    using namespace blofeld::test;
    for (const bool b : {false, true}) {
      for (const unsigned reg : {0U, 7U}) { long_saturation::reserved_case(b, reg); }
      for (const unsigned scale : {0U, scale_down, scale_up}) {
        constexpr std::uint64_t value = 0x00800000000001;
        long_saturation::one({.value = value, .scale = scale, .mode = 3, .reg = 7, .b = b});
      }
    }
    std::cout << "4 reserved-scaling refusals and 6 genuine full A/B scaling controls passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
