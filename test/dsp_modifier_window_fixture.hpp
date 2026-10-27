// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"

#include <cstdint>

namespace blofeld::test::modifier_window {
inline constexpr unsigned entry = 0x100;
inline constexpr unsigned data = 0x400;
inline constexpr unsigned pointer = 0x407;
inline constexpr unsigned canary = 0x654321;
inline constexpr unsigned loaded = 0x123456;
inline constexpr unsigned ordinary_omr = 0x300;
inline constexpr unsigned absolute_jump = 0x0af080;
inline constexpr unsigned sign24 = 0x800000;
inline constexpr unsigned linear_modifier = 0xffffff;
inline constexpr unsigned y_bit = 0x80000;
inline constexpr unsigned b_bit = 0x10000;
inline constexpr unsigned clear_flags = 0x14;
inline constexpr unsigned high_shift = 16;
inline constexpr std::uint64_t negative_extension = 0xff000000000000;
inline constexpr std::uint64_t initial_a = 0x00112233abc123;
inline constexpr std::uint64_t initial_b = 0x00102030def123;
enum class Family : std::uint8_t { chain, clear, store, agu };

struct Case {
  Family family = Family::chain;
  unsigned gap = 0;
  unsigned value = loaded;
  unsigned destination = 8;
  unsigned initial_modifier = linear_modifier;
  bool producer_b = false;
  bool consumer_b = false;
  bool y = false;
};

inline unsigned delay(const Case sample) {
  if (sample.family == Family::agu) {
    return sample.destination == 0 && sample.gap < 3 ? 3 - sample.gap : 0U;
  }
  return sample.family == Family::store && sample.producer_b == sample.consumer_b && sample.gap == 0
             ? 1U
             : 0U;
}

inline unsigned consumer_index(const Case sample) {
  return 2 + sample.gap;
}

inline unsigned last(const Case sample) {
  return 12 + sample.gap + delay(sample);
}

inline unsigned consumer_e1(const Case sample) {
  return 8 + sample.gap + delay(sample);
}

inline unsigned consumer_ag2(const Case sample) {
  return 7 + sample.gap + (sample.family == Family::agu ? delay(sample) : 0U);
}

inline std::uint64_t expanded(const unsigned value) {
  constexpr unsigned word_bits = 24;
  return (std::uint64_t{value} << word_bits) | ((value & sign24) != 0 ? negative_extension : 0U);
}

inline unsigned retired(const Case sample, const unsigned period) {
  unsigned result = 0;
  for (unsigned index = 0; index < 6 + sample.gap; ++index) {
    const auto ready = 7 + index + (index >= consumer_index(sample) ? delay(sample) : 0U);
    if (period >= ready) { ++result; }
  }
  return result;
}

DspFetchRunInputs contract();
DspDebugState prepare(DspMachine &machine, Case sample);
DspDebugState expected(const DspDebugState &before, Case sample, unsigned period);
void step(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, Case sample);
void one(Case sample);
void restores(Case sample);
}
