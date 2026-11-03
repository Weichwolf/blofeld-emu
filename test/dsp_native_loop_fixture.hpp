// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_dma.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_fetch_native_fixture.hpp"

#include <cstdint>

namespace blofeld::test::native_loop {
namespace fixture = fetch_native;
inline constexpr unsigned body = fixture::program + 2;
inline constexpr unsigned loop_flag = 0x8000, forever_flag = 0x10000, word_bits = 24;
inline constexpr unsigned setup_periods = 5, padding_words = 32;
inline constexpr std::uint64_t period = 45;
inline constexpr unsigned dor_three = 0x060390;
inline constexpr unsigned dor_two = 0x060290;
inline constexpr unsigned repeat_three = 0x0603a0;

struct Geometry {
  unsigned count;
  unsigned width;
};

inline OwnDspFetchCycleResult advance(fixture::Fixture &f,
                                      const unsigned requests = 0,
                                      const DspDmaGrant grant = DspDmaGrant::available) {
  const auto last = f.machine.bus().last_dma_cycle();
  auto result = f.fetch->advance_dma_automatic({.cycle = last ? *last + 1 : 0,
                                                .known_requests = 15,
                                                .requests = requests,
                                                .timing = DspDmaTiming::reference_sampled_cycles,
                                                .grant = grant},
                                               OwnDspAutomaticTiming::native_cycle_budget);
  fixture::healthy(result);
  return result;
}

inline void attach(fixture::Fixture &f, const unsigned prefill = 2) {
  f.attach();
  for (unsigned index = 0; index < prefill; ++index) {
    f.fetch->issue();
    f.fetch->capture();
  }
}

inline dsp56300::Registers opened(const dsp56300::Registers &initial, const Geometry geometry) {
  auto result = initial;
  result.stack[1] = (std::uint64_t{initial.la} << word_bits) | initial.lc;
  if (geometry.count == 0) {
    result.pc = body + geometry.width;
    return result;
  }
  result.pc = body;
  result.la = body + geometry.width - 1;
  result.lc = geometry.count;
  result.sp = 2;
  result.sc = 2;
  result.sr = (initial.sr & ~forever_flag) | loop_flag;
  result.stack[2] = (std::uint64_t{body} << word_bits) | initial.sr;
  return result;
}

inline dsp56300::Registers finished(const dsp56300::Registers &initial, const Geometry geometry) {
  auto result = initial;
  const auto frame = opened(initial, geometry);
  result.stack = frame.stack;
  result.pc = body + geometry.width;
  return result;
}

void state_checks();
void event_checks();
void control_checks();
} // namespace blofeld::test::native_loop
