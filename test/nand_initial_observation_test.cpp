// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_read_timing.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;

struct Vector {
  unsigned hz;
  std::uint64_t ce_release;
  std::uint64_t ce_access;
  std::uint64_t re_release;
  std::uint64_t access;
  std::uint64_t high;
  std::uint64_t cycle;
  std::uint64_t hold;
};

// Literal ceilings independently calculated from the candidate's 3.3V table.
constexpr std::array<Vector, 3> vectors{{{.hz = 1000000000,
                                          .ce_release = 20,
                                          .ce_access = 35,
                                          .re_release = 30,
                                          .access = 30,
                                          .high = 15,
                                          .cycle = 42,
                                          .hold = 15},
                                         {.hz = 1080000000,
                                          .ce_release = 22,
                                          .ce_access = 38,
                                          .re_release = 33,
                                          .access = 33,
                                          .high = 17,
                                          .cycle = 46,
                                          .hold = 17},
                                         {.hz = 216000000,
                                          .ce_release = 5,
                                          .ce_access = 8,
                                          .re_release = 7,
                                          .access = 7,
                                          .high = 4,
                                          .cycle = 10,
                                          .hold = 4}}};
constexpr std::array<std::uint64_t, 3> starts{0, 1000, 1000000};
constexpr std::array<std::uint64_t, 8> delays{0, 1, 2, 4, 8, 16, 32, 64};
constexpr std::uint64_t history_start = 1000;
constexpr std::uint64_t old_select = 100;
constexpr std::uint64_t old_width = 100;
constexpr std::uint64_t pulse_spacing = 100;
constexpr NandByte own_byte{.value = 0xa3, .known = 0xff};
constexpr NandByte old_byte{.value = 0x5a, .known = 0xff};

void require(const bool condition, const std::string_view label) {
  if (!condition) { throw Fault(std::string(label)); }
}

void rejected(const auto &operation, const std::string_view expected) {
  try {
    operation();
  } catch (const Fault &error) {
    require(std::string_view(error.what()).contains(expected),
            "Actual rejection retains its cause");
    return;
  }
  throw Fault("Missing expected refusal: " + std::string(expected));
}

void initial_vector(const Vector row, const bool ce, const std::uint64_t start) {
  NandReadTiming timing(row.hz);
  unsigned calls = 0;
  const auto source = [&] {
    ++calls;
    return own_byte;
  };
  require(timing.output(start) == NandReadOutput{}, "Unobserved output remains unknown");
  timing.observe(start, {.ce_n = ce, .re_n = true}, source);
  require(calls == 0, "First known levels cannot invent a source read or CE/RE edge");
  const auto initial_state = timing;
  auto guarded = timing;
  if (!ce) {
    rejected([&] { guarded.observe(start + row.high - 1, {.ce_n = ce, .re_n = false}, source); },
             "tREH");
    rejected([&] { guarded.observe(start + row.cycle - 1, {.ce_n = ce, .re_n = false}, source); },
             "tRC");
    require(calls == 0 && guarded == initial_state && guarded.output(start) == NandReadOutput{},
            "Premature first pulse rejects before source consumption and time commit");
  }
  const auto release = ce ? row.ce_release : row.re_release;
  require(timing.output(start) == NandReadOutput{} &&
              timing.output(start + release - 1) == NandReadOutput{} &&
              timing.output(start + release) == NandReadOutput{.driven = false},
          "First known inactive levels retire unknown drive only at the literal maximum");
  if (ce) { timing.observe(start + release, {.ce_n = false, .re_n = true}, source); }
  const auto fall = start + row.cycle;
  timing.observe(fall, {.ce_n = false, .re_n = false}, source);
  require(calls == 1 && timing.output(fall + row.access - 1) == NandReadOutput{} &&
              timing.output(fall + row.access) == NandReadOutput{.driven = true, .data = own_byte},
          "First real falling RE consumes exactly once, with independently bounded data access");
  timing.observe(fall + row.access, {.ce_n = false, .re_n = false}, source);
  require(calls == 1, "Repeated active levels consume no byte");
}

void initial_guards() {
  constexpr unsigned hz = 1000000000;
  constexpr std::array<NandReadPins, 3> invalid{{{.ce_n = std::nullopt, .re_n = true},
                                                 {.ce_n = false, .re_n = std::nullopt},
                                                 {.ce_n = false, .re_n = false}}};
  for (const auto pins : invalid) {
    NandReadTiming timing(hz);
    unsigned calls = 0;
    const auto source = [&] {
      ++calls;
      return own_byte;
    };
    rejected([&] { timing.observe(0, pins, source); },
             pins.ce_n.has_value() && pins.re_n.has_value() ? "initial RE-high" : "unknown CE/RE");
    require(calls == 0 && timing.output(0) == NandReadOutput{},
            "Invalid initial levels commit nothing");
    timing.observe(0, {.ce_n = false, .re_n = true});
    timing.observe(vectors[0].cycle, {.ce_n = false, .re_n = false}, source);
    require(calls == 1, "A refused initial observation does not manufacture persistent history");
  }
  NandReadTiming overflow(hz);
  const auto before_overflow = overflow;
  constexpr auto near_end = std::numeric_limits<std::uint64_t>::max() - 10;
  rejected([&] { overflow.observe(near_end, {.ce_n = false, .re_n = true}); }, "overflow");
  require(overflow == before_overflow && overflow.output(0) == NandReadOutput{},
          "Initial deadline overflow is transactional");
  NandReadTiming no_source(hz);
  no_source.observe(0, {.ce_n = false, .re_n = true});
  const auto before_missing = no_source;
  rejected([&] { no_source.observe(vectors[0].cycle, {.ce_n = false, .re_n = false}); },
           "no protocol");
  require(no_source == before_missing && no_source.output(0) == NandReadOutput{},
          "Missing source cannot commit an edge or its time");
  rejected([&] { no_source.observe(0, {.ce_n = true, .re_n = false}); }, "unordered");
}

NandReadTiming history(const Vector row, const bool ce, const std::uint64_t delay) {
  NandReadTiming timing(row.hz);
  timing.observe(0, {.ce_n = true, .re_n = true});
  timing.observe(old_select, {.ce_n = false, .re_n = true});
  const auto rise = history_start - delay - (ce ? 1U : 0U);
  timing.observe(rise - old_width, {.ce_n = false, .re_n = false}, [] { return old_byte; });
  timing.observe(rise, {.ce_n = false, .re_n = true});
  if (ce) { timing.observe(history_start - delay, {.ce_n = true, .re_n = true}); }
  timing.observe(history_start, {.ce_n = ce, .re_n = true});
  return timing;
}

void future_suffix(NandReadTiming concrete, NandReadTiming abstract, const Vector row) {
  unsigned concrete_calls = 0;
  unsigned abstract_calls = 0;
  const auto concrete_source = [&] {
    ++concrete_calls;
    return own_byte;
  };
  const auto abstract_source = [&] {
    ++abstract_calls;
    return own_byte;
  };
  for (unsigned pulse = 0; pulse < 3; ++pulse) {
    const auto fall =
        history_start + row.cycle + (static_cast<std::uint64_t>(pulse) * pulse_spacing);
    concrete.observe(fall, {.ce_n = false, .re_n = false}, concrete_source);
    abstract.observe(fall, {.ce_n = false, .re_n = false}, abstract_source);
    for (auto *model : {&concrete, &abstract}) {
      require(model->output(fall + row.access - 1) == NandReadOutput{} &&
                  model->output(fall + row.access) ==
                      NandReadOutput{.driven = true, .data = own_byte},
              "Every future source byte has an independent literal access boundary");
      model->observe(fall + row.access, {.ce_n = false, .re_n = true});
      require(model->output(fall + row.access + row.hold - 1) ==
                      NandReadOutput{.driven = true, .data = own_byte} &&
                  model->output(fall + row.access + row.hold) == NandReadOutput{} &&
                  model->output(fall + row.access + row.re_release) ==
                      NandReadOutput{.driven = false},
              "Observed future RE edge reinstalls literal hold/release boundaries");
    }
    require(concrete_calls == pulse + 1 && abstract_calls == pulse + 1,
            "Actual future waveforms consume one source byte per new selected falling RE");
  }
}

void initial_ce_window(const Vector row, const bool ce) {
  NandReadTiming timing(row.hz);
  timing.observe(history_start, {.ce_n = ce, .re_n = true});
  const auto snapshot = timing;
  rejected([&] { timing.qualify_ce_window(history_start + row.cycle - 1); }, "pulse constraints");
  require(timing == snapshot, "Initial CE-window refusal is completely transactional");
  const auto begin = history_start + row.cycle;
  const auto end = begin + pulse_spacing;
  timing.qualify_ce_window(begin);
  timing.resume_ce_window(begin, end, false);
  unsigned calls = 0;
  timing.observe(end, {.ce_n = false, .re_n = false}, [&] {
    ++calls;
    return own_byte;
  });
  require(calls == 1 && timing.output(end + row.ce_access - 1) == NandReadOutput{} &&
              timing.output(end + row.ce_access) ==
                  NandReadOutput{.driven = true, .data = own_byte},
          "Quiescent unknown CE history retains its literal latest tCEA bound after actual initial "
          "levels");
}

void covers_history(const Vector row, const bool ce, const std::uint64_t delay) {
  auto concrete = history(row, ce, delay);
  NandReadTiming abstract(row.hz);
  unsigned initial_calls = 0;
  abstract.observe(history_start, {.ce_n = ce, .re_n = true}, [&] {
    ++initial_calls;
    return own_byte;
  });
  const auto release = ce ? row.ce_release : row.re_release;
  for (std::uint64_t delta = 0; delta <= release; ++delta) {
    const auto known = concrete.output(history_start + delta);
    const auto uncertain = abstract.output(history_start + delta);
    if (delta < release) {
      require(uncertain == NandReadOutput{},
              "Unknown first release cannot be shortened by choosing one old edge");
    } else {
      require(uncertain == NandReadOutput{.driven = false} && known == uncertain,
              "Latest release bound covers every retained concrete history, including an old known "
              "byte");
    }
  }
  require(initial_calls == 0, "History abstraction does not consume the prior byte again");
  if (ce) {
    concrete.observe(history_start + release, {.ce_n = false, .re_n = true});
    abstract.observe(history_start + release, {.ce_n = false, .re_n = true});
  }
  future_suffix(concrete, abstract, row);
}
}

int main() {
  try {
    for (const auto row : vectors) {
      for (const auto ce : {false, true}) {
        for (const auto start : starts) { initial_vector(row, ce, start); }
        initial_ce_window(row, ce);
        for (const auto delay : delays) { covers_history(row, ce, delay); }
      }
    }
    initial_guards();
    std::cout << "PASS first inactive NAND observation:18 literal initial vectors,48 retained "
                 "histories/144 future pulses,"
                 "6 initial CE-window cases,strict unknown/active/overflow/source/ordering guards; "
                 "no initial source "
                 "callback or fabricated edge\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL initial NAND observation: " << error.what() << '\n';
    return 1;
  }
}
