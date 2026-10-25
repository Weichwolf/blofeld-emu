// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/nand_read_setup.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>

namespace {
using blofeld::NandReadSetup;
using blofeld::NandReadSetupPins;
constexpr unsigned ghz = 1000000000;
constexpr unsigned changed_at = 100;
constexpr unsigned later = 200;
constexpr unsigned ready_again = 300;
constexpr unsigned ready_setup = 20;
constexpr unsigned write_setup = 60;
// Independent primary-table conversions: Hz, WHR, AR, CLR, RR in whole ticks.
constexpr std::array<std::array<unsigned, 5>, 3> vectors{{
    {{1000000000, 60, 10, 10, 20}},
    {{1080000000, 65, 11, 11, 22}},
    {{216000000, 13, 3, 3, 5}},
}};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &reason) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) {
    require(std::string(fault.what()).contains(reason),
            "Wrong read setup fault: " + std::string(fault.what()));
    return;
  }
  throw blofeld::Fault("Missing NAND read setup fault: " + reason);
}

NandReadSetupPins pins() {
  return {.ce_n = false, .re_n = true, .we_n = true, .ale = false, .cle = false, .ready = true};
}

void no_edge(NandReadSetup &model, const std::uint64_t tick, const NandReadSetupPins value) {
  require(!model.observe(tick, value), "No selected read edge is present");
}

void prepare_signal(NandReadSetupPins &value, const unsigned field, const bool settled) {
  switch (field) {
    case 0: value.we_n = settled; break;
    case 1: value.ale = !settled; break;
    case 2: value.cle = !settled; break;
    case 3: value.ready = settled; break;
    default: throw blofeld::Fault("Invalid independent setup signal");
  }
}

void independent_boundaries() {
  constexpr std::array<const char *, 4> names{"tWHR", "tAR", "tCLR", "tRR"};
  for (const auto &row : vectors) {
    for (unsigned field = 0; field < names.size(); ++field) {
      auto value = pins();
      prepare_signal(value, field, false);
      NandReadSetup model(row[0]);
      no_edge(model, 0, value);
      prepare_signal(value, field, true);
      no_edge(model, changed_at, value);
      auto independent = model;
      value.re_n = false;
      const auto tick = changed_at + row[field + 1];
      reject([&] { static_cast<void>(model.observe(tick - 1, value)); }, names[field]);
      require(model.observe(tick, value) && independent.observe(tick, value),
              "Exact independent minimum and a fresh saved branch admit the same real edge");
      no_edge(model, tick, value);
    }
  }
}

void unknown_controls_and_modes() {
  reject([] { const NandReadSetup bad(0); }, "zero reference");
  for (const auto member : {&NandReadSetupPins::ce_n,
                            &NandReadSetupPins::re_n,
                            &NandReadSetupPins::we_n,
                            &NandReadSetupPins::ale,
                            &NandReadSetupPins::cle}) {
    NandReadSetup model(ghz);
    auto value = pins();
    (value.*member).reset();
    reject([&] { no_edge(model, 0, value); }, "unknown control");
  }
  for (const unsigned field : {0U, 1U, 2U, 3U}) {
    NandReadSetup model(ghz);
    auto value = pins();
    prepare_signal(value, field, false);
    no_edge(model, 0, value);
    value.re_n = false;
    reject([&] { static_cast<void>(model.observe(changed_at, value)); },
           field == 3 ? "observed Ready high" : "WE high and ALE/CLE low");
  }
  NandReadSetup initial(ghz);
  auto value = pins();
  value.re_n = false;
  reject([&] { no_edge(initial, 0, value); }, "initial RE-high");
}

void readiness_knowledge_and_selection() {
  NandReadSetup model(ghz);
  auto value = pins();
  value.ready.reset();
  no_edge(model, 0, value);
  value.re_n = false;
  reject([&] { static_cast<void>(model.observe(changed_at, value)); }, "observed Ready high");
  value.re_n = true;
  value.ready = true;
  no_edge(model, changed_at, value);
  value.re_n = false;
  require(model.observe(changed_at + ready_setup, value), "Observed initial Ready settles");
  value.re_n = true;
  value.ready.reset();
  no_edge(model, later, value);
  value.ready = true;
  no_edge(model, ready_again, value);
  value.re_n = false;
  reject([&] { static_cast<void>(model.observe(ready_again + ready_setup - 1, value)); }, "tRR");
  require(model.observe(ready_again + ready_setup, value),
          "Withdrawn Ready knowledge cannot reuse an older known-high interval");
  value.re_n = true;
  value.ce_n = true;
  reject([&] { no_edge(model, ready_again + write_setup, value); }, "unordered");
  value.ce_n = false;
  no_edge(model, ready_again + write_setup, value);
  value.ce_n = true;
  no_edge(model, ready_again + write_setup + 1, value);
  value.re_n = false;
  no_edge(model, ready_again + write_setup + 2, value);
}

void monotonicity_and_counter_end() {
  NandReadSetup model(ghz);
  auto value = pins();
  no_edge(model, changed_at, value);
  reject([&] { no_edge(model, changed_at - 1, value); }, "backwards");
  value.we_n = false;
  value.re_n = false;
  reject([&] { no_edge(model, later, value); }, "unordered");
  constexpr auto last_tick = std::numeric_limits<std::uint64_t>::max();
  NandReadSetup near_end(ghz);
  value = pins();
  no_edge(near_end, last_tick - write_setup, value);
  value.re_n = false;
  reject([&] { static_cast<void>(near_end.observe(last_tick - 1, value)); }, "tWHR");
  require(near_end.observe(last_tick, value),
          "Elapsed-time checking admits a complete interval at the last representable tick");
}
}

int main() {
  try {
    independent_boundaries();
    unknown_controls_and_modes();
    readiness_knowledge_and_selection();
    monotonicity_and_counter_end();
    std::cout << "NAND read admission: independent WHR/AR/CLR/RR boundaries, unknown readiness, "
                 "selection and saved state verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
