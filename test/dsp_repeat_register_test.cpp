// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_repeat_register_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string_view>

namespace {
using namespace blofeld::test;
using namespace blofeld::test::repeat_register;
constexpr std::array sources{4U, 5U, 6U, 7U, 24U, 25U, 26U, 27U, 28U, 29U, 30U, 31U};
constexpr std::array counters{Counter::immediate, Counter::x0, Counter::a1};
constexpr unsigned streams_per_mode = 432, snapshot_streams_per_mode = 144;

struct Options {
  bool baseline = false, snapshots = false;
};

struct Stats {
  unsigned streams = 0, cuts = 0;
};

void run_case(Case sample, const Options options, Stats &stats) {
  sample.mutate = options.snapshots;
  if (options.baseline) {
    refused(sample);
  } else {
    stats.cuts += one(sample, options.snapshots);
  }
  ++stats.streams;
}

void counts(Case sample, const Options options, Stats &stats) {
  if (options.snapshots) {
    run_case(sample, options, stats);
    return;
  }
  for (const unsigned count : {1U, 3U, 17U}) {
    sample.count = count;
    run_case(sample, options, stats);
  }
}

void counter_forms(Case sample, const Options options, Stats &stats) {
  for (const auto counter : counters) {
    sample.counter = counter;
    counts(sample, options, stats);
  }
}

void targets(Case sample, const Options options, Stats &stats) {
  for (const unsigned reg : {1U, 3U}) {
    sample.reg = reg;
    for (const bool y : {false, true}) {
      sample.y = y;
      counter_forms(sample, options, stats);
    }
  }
}

void source_fields(const unsigned mode, const Options options, Stats &stats) {
  constexpr unsigned full_flags = 0xff;
  for (const auto source : sources) {
    targets({.source = source, .mode = mode, .flags = (source & 1U) != 0 ? full_flags : 1U},
            options,
            stats);
  }
}
}

int main(const int count, char **arguments) {
  try {
    const std::string_view mode = count == 2 ? arguments[1] : "--streams";
    if (count > 2 || (mode != "--streams" && mode != "--baseline" && mode != "--address-modes" &&
                      mode != "--snapshots")) {
      throw blofeld::Fault("Expected --streams, --baseline, --address-modes or --snapshots");
    }
    const Options options{.baseline = mode == "--baseline", .snapshots = mode == "--snapshots"};
    Stats stats;
    unsigned expected = streams_per_mode;
    if (mode == "--address-modes") {
      constexpr std::array modes{0U, 1U, 2U, 4U, 5U, 7U};
      for (const auto address_mode : modes) { source_fields(address_mode, options, stats); }
      expected *= static_cast<unsigned>(modes.size());
    } else if (options.snapshots) {
      constexpr std::array modes{1U, 3U, 5U, 7U};
      for (const auto address_mode : modes) { source_fields(address_mode, options, stats); }
      expected = snapshot_streams_per_mode * static_cast<unsigned>(modes.size());
    } else {
      source_fields(3, options, stats);
    }
    require(stats.streams == expected,
            "All raw register,space,target,count and prefix cases execute");
    std::cout << stats.streams << " raw REP streams passed " << mode << "; cuts=" << stats.cuts
              << " suffixes=" << stats.cuts * 2 << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
