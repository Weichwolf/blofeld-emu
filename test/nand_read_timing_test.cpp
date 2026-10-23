// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/nand_array.hpp"
#include "blofeld/nand_read_timing.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>

namespace {
using blofeld::NandByte;
using blofeld::NandReadOutput;
using blofeld::NandReadTiming;
constexpr NandByte first{.value = 0x96, .known = 0xff};
constexpr NandByte second{.value = 0x29, .known = 0xff};
constexpr unsigned exact_hz = 1000000000;
constexpr unsigned re_access = 30;
constexpr unsigned ce_access = 35;
constexpr unsigned hold = 15;
constexpr unsigned ce_release = 20;
constexpr unsigned pulse = 21;
constexpr unsigned read_cycle = 42;
constexpr unsigned next_read = 60;
constexpr unsigned initial_clearance = 100;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &reason) {
  try {
    operation();
  } catch (const blofeld::Fault &error) {
    require(std::string(error.what()).contains(reason),
            "Wrong timing failure: " + std::string(error.what()));
    return;
  }
  throw blofeld::Fault("Missing timing failure: " + reason);
}

void idle(NandReadTiming &timing, const std::uint64_t tick) {
  timing.observe(0, {.ce_n = true, .re_n = true});
  timing.observe(tick, {.ce_n = false, .re_n = true});
}

// Each directed waveform starts after a declared100-tick idle clearance.
class RelativeTiming {
public:
  explicit RelativeTiming(const unsigned hz) : device_(hz) { idle(device_, initial_clearance); }

  void observe(const std::uint64_t tick,
               const blofeld::NandReadPins pins,
               const NandReadTiming::Source &source = {}) {
    device_.observe(initial_clearance + tick, pins, source);
  }

  NandReadOutput output(const std::uint64_t tick) {
    return device_.output(initial_clearance + tick);
  }

private:
  NandReadTiming device_;
};

RelativeTiming reading(const unsigned hz = exact_hz) {
  RelativeTiming timing(hz);
  timing.observe(0, {.ce_n = false, .re_n = false}, [] { return first; });
  return timing;
}

// Fixed primary-table conversions, independent of the implementation's division.
// Rows are frequency, tCEA, tREA, tOH, tRHZ and tCHZ in whole reference ticks.
void timing_boundaries() {
  constexpr std::array<std::array<unsigned, 6>, 3> rates{{{{1000000000, 35, 30, 15, 30, 20}},
                                                          {{1080000000, 38, 33, 17, 33, 22}},
                                                          {{216000000, 8, 7, 4, 7, 5}}}};
  for (const auto &row : rates) {
    auto timing = reading(row[0]);
    require(timing.output(row[1] - 1) == NandReadOutput{},
            "No byte/drive knowledge before both CE/RE access maxima");
    require(timing.output(row[1]) == NandReadOutput{.driven = true, .data = first},
            "Known protocol byte is valid at the independent rounded CE deadline");
    RelativeTiming delayed(row[0]);
    const auto fall = row[1] * 2;
    delayed.observe(fall, {.ce_n = false, .re_n = false}, [] { return second; });
    require(delayed.output(fall + row[2] - 1) == NandReadOutput{} &&
                delayed.output(fall + row[2]) == NandReadOutput{.driven = true, .data = second},
            "Settled CE leaves the independently rounded RE access bound in control");
    auto selected_release = timing;
    timing.observe(row[1], {.ce_n = false, .re_n = true});
    selected_release.observe(row[1], {.ce_n = true, .re_n = false});
    for (auto *branch : {&timing, &selected_release}) {
      require(branch->output(row[1] + row[3] - 1) == NandReadOutput{.driven = true, .data = first},
              "Read output retains its complete byte throughout guaranteed tOH");
      require(branch->output(row[1] + row[3]) == NandReadOutput{},
              "Release window is neither prematurely floating nor still known data");
    }
    require(timing.output(row[1] + row[4] - 1) == NandReadOutput{} &&
                timing.output(row[1] + row[4]) == NandReadOutput{.driven = false},
            "RE release becomes definitely Hi-Z only at its maximum deadline");
    require(selected_release.output(row[1] + row[5] - 1) == NandReadOutput{} &&
                selected_release.output(row[1] + row[5]) == NandReadOutput{.driven = false},
            "CE release uses its own shorter maximum deadline");
  }
}

void re_access_and_early_release() {
  RelativeTiming timing(exact_hz);
  timing.observe(next_read, {.ce_n = false, .re_n = false}, [] { return second; });
  require(timing.output(next_read + re_access - 1) == NandReadOutput{} &&
              timing.output(next_read + re_access) ==
                  NandReadOutput{.driven = true, .data = second},
          "A settled CE leaves RE's own access bound in control");
  auto early = reading();
  early.observe(pulse, {.ce_n = false, .re_n = true});
  require(early.output(pulse + hold - 1) == NandReadOutput{},
          "A legal short RE pulse cannot invent a byte or guaranteed drive before tREA/tCEA");
  require(early.output(pulse + re_access) == NandReadOutput{.driven = false},
          "Even an unobserved short-pulse output eventually releases");
  auto combined = reading();
  combined.observe(ce_access, {.ce_n = false, .re_n = true});
  combined.observe(ce_access + 1, {.ce_n = true, .re_n = true});
  require(combined.output(ce_access + hold - 1) == NandReadOutput{.driven = true, .data = first} &&
              combined.output(ce_access + hold) == NandReadOutput{} &&
              combined.output(ce_access + 1 + ce_release) == NandReadOutput{.driven = false},
          "Second release edge can shorten Hi-Z deadline but cannot extend guaranteed hold");
}

void pulse_and_cycle_rejections() {
  auto timing = reading();
  reject([&] { timing.observe(pulse - 1, {.ce_n = false, .re_n = true}); }, "tRP");
  timing.observe(pulse, {.ce_n = false, .re_n = true});
  unsigned calls = 0;
  const auto source = [&] {
    ++calls;
    return second;
  };
  reject([&] { timing.observe(pulse + hold - 1, {.ce_n = false, .re_n = false}, source); }, "tREH");
  reject([&] { timing.observe(read_cycle - 1, {.ce_n = false, .re_n = false}, source); }, "tRC");
  require(calls == 0, "Rejected edge timing cannot consume a protocol byte");
  timing.observe(read_cycle, {.ce_n = false, .re_n = false}, source);
  require(calls == 1 && timing.output(read_cycle + re_access) ==
                            NandReadOutput{.driven = true, .data = second},
          "Exact minimum cycle is admitted once, without replaying its failed attempts");
}

void unknown_controls_and_sources() {
  reject([] { const NandReadTiming timing(0); }, "zero reference timebase");
  NandReadTiming unobserved(exact_hz);
  require(unobserved.output(0) == NandReadOutput{}, "Unobserved idle pins are not assumed high");
  reject([&] { unobserved.observe(0, {.ce_n = false, .re_n = true}); }, "idle-high");
  reject([&] { unobserved.observe(0, {.ce_n = true, .re_n = std::nullopt}); }, "unknown CE/RE");
  reject([&] { unobserved.observe(0, {.ce_n = std::nullopt, .re_n = true}); }, "unknown CE/RE");
  unobserved.observe(0, {.ce_n = true, .re_n = true});
  require(unobserved.output(ce_release - 1) == NandReadOutput{} &&
              unobserved.output(ce_release) == NandReadOutput{.driven = false},
          "First idle levels require tCHZ before a floating output can be established");
  RelativeTiming timing(exact_hz);
  timing.observe(0, {.ce_n = true, .re_n = true});
  reject([&] { timing.observe(0, {.ce_n = false, .re_n = false}); }, "unordered");
  timing.observe(0, {.ce_n = false, .re_n = true});
  reject([&] { timing.observe(0, {.ce_n = false, .re_n = false}); }, "no protocol data source");
  reject(
      [&] {
        timing.observe(0, {.ce_n = false, .re_n = false}, []() -> NandByte {
          throw blofeld::Fault("missing array byte");
        });
      },
      "missing array byte");
  reject(
      [&] {
        timing.observe(0, {.ce_n = false, .re_n = false}, [] { return NandByte{.value = 1}; });
      },
      "noncanonical");
  constexpr NandByte partial{.value = 0x80, .known = 0xc1};
  timing.observe(0, {.ce_n = false, .re_n = false}, [partial] { return partial; });
  require(timing.output(ce_access) == NandReadOutput{.driven = true, .data = partial},
          "A valid timing window cannot manufacture missing protocol-bit knowledge");
  reject([&] { static_cast<void>(timing.output(ce_access - 1)); }, "backwards");
  timing.observe(ce_access, {.ce_n = true, .re_n = false});
  reject([&] { timing.observe(ce_access, {.ce_n = false, .re_n = false}); },
         "CE assertion while RE");
  timing.observe(ce_access, {.ce_n = true, .re_n = true});
  reject([&] { timing.observe(ce_access, {.ce_n = false, .re_n = true}); }, "reselect");
}

void copies_and_overflow() {
  auto timing = reading();
  auto saved = timing;
  timing.observe(ce_access, {.ce_n = false, .re_n = true});
  require(saved.output(ce_access + re_access) == NandReadOutput{.driven = true, .data = first} &&
              timing.output(ce_access + re_access) == NandReadOutput{.driven = false},
          "Independent timing snapshots retain active and released branches separately");
  constexpr auto late = std::numeric_limits<std::uint64_t>::max() - 1;
  NandReadTiming overflow(exact_hz);
  idle(overflow, late);
  unsigned calls = 0;
  reject(
      [&] {
        overflow.observe(late, {.ce_n = false, .re_n = false}, [&] {
          ++calls;
          return first;
        });
      },
      "overflow");
  require(calls == 0 && overflow.output(late) == NandReadOutput{.driven = false},
          "Overflow rejection neither consumes a byte nor changes driver state");
}
}

int main() {
  try {
    timing_boundaries();
    re_access_and_early_release();
    pulse_and_cycle_rejections();
    unknown_controls_and_sources();
    copies_and_overflow();
    std::cout << "NAND read timing: independent CE/RE/access/hold/Hi-Z bounds, strict failures and "
                 "snapshots checked.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
