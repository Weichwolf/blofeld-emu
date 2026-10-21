// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_timers.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld::test::request {
inline constexpr unsigned scheduler = 0x22d;
inline constexpr unsigned tc_ie = 0x1876;
inline constexpr unsigned input_wait = 0x188a;
inline constexpr unsigned timer = 0xffff8b;
inline constexpr unsigned port = 0xffffbd;
inline constexpr unsigned flags = 6;
inline constexpr unsigned busy = 0x2000;
inline constexpr unsigned clear_flags = 0x300000;
inline constexpr unsigned scheduler_gpio = 0x1894;
inline constexpr unsigned continuation_gpio = 0x18a5;
inline constexpr unsigned compare_bit = 2;
inline constexpr unsigned input_bit = 12;
inline constexpr unsigned latch_bit = 8;
inline constexpr unsigned low_polls = 4;
inline constexpr unsigned edge_polls = 2;
inline constexpr unsigned bound = 64;

struct Observation {
  std::optional<bool> drive;
  unsigned compare_reads = 0;
  unsigned attempts = 0;
  unsigned low = 0;
  unsigned high = 0;
  unsigned gpio_attempts = 0;
  unsigned flag_writes = 0;
  unsigned timer_writes = 0;
  unsigned expected_flags = 0;
};

inline void observe_write(const DspNativeAccess &access, Observation &seen) {
  if (access.kind == DspNativeAccessKind::write) {
    require(access.target.space == DspSpace::x, "Ready service has no Y/P memory writes");
    require((access.target.address == flags && access.value == seen.expected_flags) ||
                (access.target.address == timer && access.value == clear_flags),
            "Only original busy flag and TCSR1 W1C writes are allowed after readiness");
    if (access.stage == DspNativeAccessStage::after) {
      if (access.target.address == flags) {
        ++seen.flag_writes;
      } else {
        ++seen.timer_writes;
      }
    }
  }
}

inline void observe(const DspNativeAccess &access, Observation &seen) {
  observe_write(access, seen);
  if (access.kind != DspNativeAccessKind::bit) { return; }
  if (!access.bit) { throw Fault("Original ready service bit operation lacks context"); }
  if (access.target.address == port) {
    require(access.target.space == DspSpace::x && access.bit->index == latch_bit &&
                access.bit->operation == DspBitOperation::set &&
                access.stage == DspNativeAccessStage::before && !access.value,
            "Unknown PDRC latch must fail before returning a previous value");
    ++seen.gpio_attempts;
    return;
  }
  if (access.target.address != timer) { return; }
  require(access.target.space == DspSpace::x && access.bit->operation == DspBitOperation::test,
          "Original ready service only tests TCSR1 bits");
  if (access.bit->index == compare_bit) {
    require(access.pc == tc_ie, "Actual TCIE instruction context differs");
    if (access.stage == DspNativeAccessStage::after) {
      require(access.value == 0, "Reset TCIE is clear");
      ++seen.compare_reads;
    }
    return;
  }
  require(access.bit->index == input_bit && access.pc == input_wait,
          "Actual readiness decision must read TIO1 DI");
  if (access.stage == DspNativeAccessStage::before) {
    ++seen.attempts;
  } else {
    if (!seen.drive) { throw Fault("Unobserved TIO1 cannot return a fabricated input bit"); }
    require(access.value == static_cast<unsigned>(*seen.drive), "DI differs from explicit TIO1");
    if (*seen.drive) {
      ++seen.high;
    } else {
      ++seen.low;
    }
  }
}

template <class Fixture, class Input>
void validate(Fixture &fixture,
              const Input &input,
              const Observation &seen,
              const DspState &before,
              const std::string &error) {
  require(seen.compare_reads == 1, "Original TCIE decision must execute exactly once");
  const auto sp = input.target == scheduler ? 2U : 1U;
  require(before.sp == sp, "Original scheduler or continuation frame remains pending");
  if (input.ready_profile == 2) {
    require(error.empty() && before.pc == input_wait && seen.low == low_polls && seen.high == 0 &&
                seen.gpio_attempts == 0 && seen.flag_writes == 0 && seen.timer_writes == 0,
            "Low TIO1 must hold the genuine polling loop without device writes");
  } else {
    require(!error.empty(), "A missing pin/latch cannot produce a successful ready service");
    if (input.ready_profile == 1) {
      require(seen.attempts == 1 && seen.low == 0 && seen.high == 0 && before.pc == input_wait &&
                  error.contains("Unknown DSP timer DI read at X:0xffff8b"),
              "Unobserved TIO1 fails at its original DI read");
    } else {
      const auto expected_pc = input.target == scheduler ? scheduler_gpio : continuation_gpio;
      const auto expected_low = input.ready_profile == 4 ? edge_polls : 0U;
      require(before.pc == expected_pc && seen.low == expected_low && seen.high == 1 &&
                  seen.gpio_attempts == 1 && seen.flag_writes == 1 && seen.timer_writes == 1 &&
                  error.contains("Unknown DSP GPIO data read at X:0xffffbd mask=0x000100"),
              "Observed readiness must reach the actual unknown PDRC latch, not fake success");
    }
    bool rejected = false;
    try {
      fixture.cpu.step();
    } catch (const Fault &) { rejected = true; }
    require(rejected, "A failed original readiness step requires diagnostic reset");
  }
}

template <class Fixture, class Input> void probe(Fixture &fixture, const Input &input) {
  require(input.ready_profile >= 1 && input.ready_profile <= 4,
          "Declared readiness profile is unknown, low, high or low-to-high");
  Observation seen;
  seen.expected_flags = input.flags | busy;
  if (input.ready_profile != 1) { seen.drive = input.ready_profile == 3; }
  fixture.bus.timers().set_tio_drive(1, seen.drive);
  const DspNativeAccessObserver observer{
      .observe = [&](const DspNativeAccess &access) { observe(access, seen); }};
  std::string error;
  DspState before;
  unsigned completed = 0;
  for (unsigned attempts = 0; attempts < bound; ++attempts) {
    before = fixture.cpu.state();
    if (before.pc == input_wait) {
      if (input.ready_profile == 2 && seen.low == low_polls) { break; }
      if (input.ready_profile == 4 && seen.low == edge_polls) {
        seen.drive = true;
        fixture.bus.timers().set_tio_drive(1, seen.drive);
      }
    }
    try {
      fixture.cpu.step(observer);
      fixture.bus.advance_instruction_cycles(1);
      ++completed;
    } catch (const Fault &failure) {
      error = failure.what();
      break;
    }
  }
  validate(fixture, input, seen, before, error);
  fixture.trace << "READY_PROFILE " << std::dec << input.ready_profile << "\nREADY_COUNTS "
                << seen.attempts << ' ' << seen.low << ' ' << seen.high << ' ' << seen.gpio_attempts
                << ' ' << seen.flag_writes << ' ' << seen.timer_writes << "\nREADY_STEPS "
                << completed << '\n';
  fixture.trace << "READY_ERROR " << (error.empty() ? "none" : error) << '\n';
  fixture.state("READY_STATE", before);
  fixture.trace << "READY_FLAGS " << std::hex << fixture.get(flags) << '\n';
}
}
