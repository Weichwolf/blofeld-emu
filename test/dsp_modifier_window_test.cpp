// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp_modifier_window_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld::test::modifier_window;

void raw_cases(std::vector<Case> &cases) {
  for (const unsigned value : {loaded, linear_modifier}) {
    for (unsigned gap = 0; gap < 2; ++gap) {
      for (const unsigned destination : {2U, 8U, 9U}) {
        cases.push_back(
            {.family = Family::chain, .gap = gap, .value = value, .destination = destination});
      }
      for (const bool producer_b : {false, true}) {
        for (const bool consumer_b : {false, true}) {
          cases.push_back({.family = Family::clear,
                           .gap = gap,
                           .value = value,
                           .producer_b = producer_b,
                           .consumer_b = consumer_b});
          for (const bool y : {false, true}) {
            cases.push_back({.family = Family::store,
                             .gap = gap,
                             .value = value,
                             .producer_b = producer_b,
                             .consumer_b = consumer_b,
                             .y = y});
          }
        }
      }
    }
  }
}

void address_cases(std::vector<Case> &cases) {
  constexpr std::array upper_bytes{0U, 1U, 0x7fU, 0x80U, 0xffU};
  for (const auto upper : upper_bytes) {
    const auto modifier = (upper << high_shift) | 7U;
    for (unsigned gap = 0; gap < 4; ++gap) {
      for (unsigned route = 0; route < 3; ++route) {
        const auto existing = route == 1 ? modifier : 7U;
        const auto initial = route == 0 ? linear_modifier : existing;
        for (const bool y : {false, true}) {
          cases.push_back({.family = Family::agu,
                           .gap = gap,
                           .value = modifier,
                           .destination = route == 2 ? 7U : 0U,
                           .initial_modifier = initial,
                           .y = y});
        }
      }
    }
  }
}
}

int main(const int count, char **arguments) {
  try {
    const std::string_view mode = count == 2 ? arguments[1] : "--all";
    if (count > 2 || (mode != "--smoke" && mode != "--all")) {
      throw blofeld::Fault("Usage: modifier-window-test [--smoke|--all]");
    }
    constexpr unsigned high_modulo = 0xff0007;
    constexpr std::array examples{
        Case{},
        Case{.family = Family::clear},
        Case{.family = Family::store},
        Case{.family = Family::agu, .value = high_modulo, .destination = 0}};
    for (const auto sample : examples) { one(sample); }
    if (mode == "--smoke") {
      std::cout << "Four independent modifier producer windows passed\n";
      return 0;
    }
    std::vector<Case> cases;
    raw_cases(cases);
    address_cases(cases);
    for (const auto sample : cases) {
      one(sample);
      restores(sample);
    }
    std::cout << cases.size() << " independent modifier windows,1080 cuts/2160 replays passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
