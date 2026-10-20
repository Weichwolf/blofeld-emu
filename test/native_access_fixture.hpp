// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test::native_access {
using Kind = DspNativeAccessKind;
using Stage = DspNativeAccessStage;
inline constexpr unsigned data = 0x100;
inline constexpr unsigned second_data = 0x500;
inline constexpr unsigned first = 0x123456;
inline constexpr unsigned second = 0x234567;
inline constexpr unsigned immediate_x0 = 0x44f400; // FM MOVE long immediate, X0 destination.

inline std::string fault(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected a rejected native-access operation");
}

struct Recorder {
  std::vector<DspNativeAccess> rows;

  [[nodiscard]] DspNativeAccessObserver observer() {
    return {.observe = [this](const DspNativeAccess &event) { rows.push_back(event); }};
  }

  void step(DspMachine &machine) {
    rows.clear();
    machine.cpu().step(observer());
  }
};

struct Expected {
  Kind kind;
  DspAddress target;
  unsigned value;
  std::optional<DspNativeAccessBit> bit = std::nullopt;
  std::size_t offset = 0;
  std::uint64_t ordinal = 0;
  unsigned pc = boot;
};

inline void pair(const std::span<const DspNativeAccess> rows, const Expected expected) {
  require(rows.size() >= expected.offset + 2, "Native access pair is present");
  for (std::size_t index = 0; index < 2; ++index) {
    const auto &row = rows[expected.offset + index];
    require(row.kind == expected.kind && row.target.space == expected.target.space &&
                row.target.address == expected.target.address && row.bit == expected.bit &&
                row.ordinal == expected.ordinal && row.pc == expected.pc && row.omr == 5 &&
                row.stage == (index == 0 ? Stage::before : Stage::after),
            "Native address/kind/order/step context agrees with the literal expectation");
    const auto value = index != 0 || expected.kind == Kind::write
                           ? std::optional<std::uint32_t>{expected.value}
                           : std::nullopt;
    require(row.value == value, "Only completed reads and known write sources have values");
  }
}

inline void execute(DspMachine &machine, const unsigned count) {
  for (unsigned index = 0; index < count; ++index) { machine.cpu().step(); }
}

void failure_checks();
void continuation_checks();
}
