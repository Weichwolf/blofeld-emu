// SPDX-License-Identifier: GPL-3.0-only
#include "display/lcd_image.hpp"
#include "firmware/firmware.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard_chain_fixture.hpp"
#include "keyboard_handoff_fixture.hpp"
#include "original_update_transport_fixture.hpp"
#include "storage/private_file.hpp"

#include <cstddef>
#include <exception>
#include <fstream>
#include <ios>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::keyboard_chain;
using blofeld::test::hcs08::require;
constexpr Scenario second_case{.order = 1, .phase = 18, .target_phase = 14};

struct Inputs {
  const Firmware &firmware;
  const std::vector<transport::Block> &blocks;
  const transport::Templates &templates;
};

void run_case(const Inputs input,
              const Scenario scenario,
              std::ostream &trace,
              const std::string &images) {
  std::ostringstream events;
  Chain run(input.firmware, input.blocks, input.templates, events, scenario);
  trace << "CASE order=" << scenario.order << " phase=" << scenario.phase
        << " target-phase=" << scenario.target_phase << '\n';
  try {
    run.prefix();
    run.until_saving();
  } catch (...) {
    trace << events.str();
    throw;
  }
  const auto saved = run.checkpoint();
  const auto offset = static_cast<std::size_t>(events.tellp());
  try {
    run.until_overlap();
  } catch (...) {
    trace << events.str();
    throw;
  }
  const auto complete = events.str();
  const auto suffix = complete.substr(offset);
  trace << complete;
  for (unsigned n = 0; n < 2; ++n) {
    std::ostringstream actual;
    Chain restored(input.firmware, input.blocks, input.templates, actual, scenario);
    restored.restore(saved);
    actual.str({});
    actual.clear();
    restored.until_overlap();
    require(actual.str() == suffix,
            "Fresh joint three-CPU/reset/BDC/wire/LCD owners repeat every native suffix event");
    restored.compare_endpoint(run);
    trace << "REPLAY index=" << n << " bytes=" << actual.str().size() << " identical=1\n";
  }
  const auto image =
      capture_lcd(run.display->controller(), {.width = 128, .reverse_commons = true});
  const auto base =
      images + "." + std::to_string(scenario.order) + "." + std::to_string(scenario.phase);
  write_private_file(base + ".pbm", encode_lcd_pbm(image));
  write_private_file(base + ".png", encode_lcd_png(image));
  trace << "CASE_END order=" << scenario.order << " phase=" << scenario.phase
        << " target-phase=" << scenario.target_phase
        << " host-steps=" << run.sender.pair.link.msteps << " target-retired=0 uploaded=0\n";
}
}

int main(const int count, char **arguments) {
  try {
    blofeld::test::hcs08::require(
        count == 5,
        "Expected original main firmware, Keyboard packets, templates and private trace");
    const auto firmware = Firmware::load(arguments[1]);
    const auto blocks = transport::read_blocks(arguments[2], kh::layouts);
    const auto templates = transport::read_templates(arguments[3]);
    const Inputs inputs{.firmware = firmware, .blocks = blocks, .templates = templates};
    std::ofstream trace(arguments[4]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    run_case(inputs, {.order = 0, .phase = 0, .target_phase = 1}, trace, arguments[4]);
    run_case(inputs, second_case, trace, arguments[4]);
    trace << "END cases=2 replays=4 target-retired=0 uploaded=0\n";
    std::cout
        << "Original Keyboard reset/LCD/BDC chain retains its strict reference wire frontier.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
