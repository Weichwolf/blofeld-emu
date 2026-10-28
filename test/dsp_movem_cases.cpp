// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_movem_fixture.hpp"

#include <algorithm>
#include <array>
#include <string>
#include <vector>

namespace blofeld::test::movem {
std::vector<Case> cases() {
  std::vector<Case> result;
  constexpr unsigned case_count = 35;
  result.reserve(case_count);
  for (unsigned reg = 0; reg < 8; ++reg) {
    result.push_back({.kind = Kind::load, .reg = reg});
    result.push_back({.kind = Kind::store, .reg = reg});
  }
  for (const auto kind :
       {Kind::absolute_load, Kind::absolute_store, Kind::short_load, Kind::short_store}) {
    result.push_back({.kind = kind});
  }
  for (const auto kind : {Kind::load, Kind::store, Kind::absolute_store}) {
    result.push_back({.kind = kind, .omr = stage_omr});
  }
  for (const auto producer : {Producer::alu, Producer::address}) {
    for (unsigned gap = 0; gap < 3; ++gap) {
      result.push_back({.kind = Kind::store, .gap = gap, .producer = producer});
    }
  }
  for (const auto omr : {ordinary_omr, stage_omr}) {
    for (const bool call : {false, true}) {
      result.push_back({.kind = Kind::loop, .omr = omr, .call = call});
    }
    result.push_back({.kind = Kind::patch_loop, .omr = omr, .call = true});
  }
  return result;
}

std::string name(const Case sample) {
  constexpr std::array kinds{"load",
                             "store",
                             "absolute-load",
                             "absolute-store",
                             "short-load",
                             "short-store",
                             "loop-store",
                             "patch-loop"};
  constexpr std::array producers{"plain", "alu", "address"};
  return std::string(kinds.at(static_cast<unsigned>(sample.kind))) + "-r" +
         std::to_string(sample.reg) + (sample.omr == ordinary_omr ? "-omr300-" : "-omr600389-") +
         producers.at(static_cast<unsigned>(sample.producer)) + "-gap" +
         std::to_string(sample.gap) + "-call" + (sample.call ? "1" : "0");
}

unsigned periods(const Case sample) {
  // Frozen whole-window software observations; this does not locate a silicon sampling edge.
  constexpr unsigned quiet = 25;
  constexpr unsigned patch_clocks = 91;
  constexpr unsigned call_loop_clocks = 75;
  constexpr unsigned loop_clocks = 45;
  if (sample.kind == Kind::patch_loop) { return patch_clocks; }
  if (sample.kind == Kind::loop) { return sample.call ? call_loop_clocks : loop_clocks; }
  if (sample.producer == Producer::alu) {
    return quiet + 1 + sample.gap + (sample.gap == 0 ? 1U : 0U);
  }
  if (sample.producer == Producer::address) { return quiet + 2 + std::max(2U, sample.gap); }
  return quiet +
         (sample.kind == Kind::absolute_load || sample.kind == Kind::absolute_store ? 1U : 0U);
}

Registers initial(const Case sample) {
  constexpr unsigned flags = 0xaa;
  constexpr unsigned status = 0xc00300;
  constexpr unsigned patch_pointer = 0x610;
  constexpr unsigned ordinary_b1 = 0x557799;
  constexpr Registers baseline{0x445566,
                               0x112233,
                               3,
                               0x778899,
                               0xe9,
                               0x91abcd,
                               0xd4e5f6,
                               0x12,
                               ordinary_b1,
                               0x6c7d8e,
                               start,
                               status,
                               ordinary_omr,
                               0x55aa33,
                               0x33};
  auto result = baseline;
  result.at(omr_field) = sample.omr;
  result.at(b1_field) = sample.kind == Kind::patch_loop ? patch_pointer : ordinary_b1;
  if (sample.producer != Producer::alu && sample.kind != Kind::patch_loop) {
    result[sr_field] |= flags;
  }
  for (unsigned reg = 0; reg < 8; ++reg) {
    result.at(first_r + reg) = data;
    result.at(first_m + reg) = value_mask;
  }
  return result;
}
}
