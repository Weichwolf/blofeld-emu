// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_swi_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::swi;
using blofeld::test::hcs08::Access;
using blofeld::test::hcs08::rejected;
constexpr unsigned caller_pc = 0xe00a, caller_sp = 0x47f, caller_hx = 0x1234;
constexpr unsigned setup = 5, phases = 7, cold_flags = 0x68;

std::array<std::uint8_t, 14> program(const unsigned raw) {
  // Own LDHX#0480,TXS,LDHX#1234,LDA#raw,TAP,SWI,NOPs;no CPU state setter.
  constexpr unsigned stack_high = 0x12;
  constexpr unsigned stack_low = 0x34;
  const std::array<std::uint8_t, 14> code{0x45,
                                          4,
                                          0x80,
                                          0x94,
                                          0x45,
                                          stack_high,
                                          stack_low,
                                          0xa6,
                                          static_cast<std::uint8_t>(raw),
                                          0x84,
                                          0x83,
                                          0x9d,
                                          0x9d,
                                          0x9d};
  return code;
}

Hcs08State caller(const unsigned raw) {
  return {.pc = caller_pc,
          .hx = caller_hx,
          .sp = caller_sp,
          .a = static_cast<std::uint8_t>(raw),
          .ccr = static_cast<std::uint8_t>(raw | fixed_flags)};
}

std::vector<Access> expected_full_frame(const unsigned raw, const unsigned target) {
  // Family RM Rev2 SWI stack order,independent literal byte/address expectations.
  constexpr unsigned pcl = 0x0b;
  constexpr unsigned pch = 0xe0;
  constexpr unsigned x = 0x34;
  constexpr std::array<unsigned, 5> addresses{0x47f, 0x47e, 0x47d, 0x47c, 0x47b};
  return {{.address = addresses[0], .value = pcl, .write = true},
          {.address = addresses[1], .value = pch, .write = true},
          {.address = addresses[2], .value = x, .write = true},
          {.address = addresses[3], .value = raw, .write = true},
          {.address = addresses[4], .value = raw | fixed_flags, .write = true},
          {.address = vector, .value = target >> 8U, .write = false},
          {.address = vector + 1, .value = 0, .write = false}};
}

void enter(Unit &unit, const Configuration config, const unsigned raw) {
  require(unit.cpu.state() == caller(raw), "Native setup establishes every caller register/flag");
  unit.memory.accesses.clear();
  unit.cpu.step();
  auto expected = caller(raw);
  expected.pc = static_cast<std::uint16_t>(config.target);
  constexpr unsigned entered_sp = 0x47a;
  expected.sp = entered_sp;
  expected.ccr |= mask_irq;
  require(unit.cpu.state() == expected &&
              unit.memory.accesses == expected_full_frame(raw, config.target) &&
              unit.memory.vector_reads == std::vector<std::uint16_t>{vector, vector + 1},
          "Actual SWI writes PCL/PCH/X/A/CCR then reads both owned vector bytes and branches");
}

void returned(const Unit &unit, const unsigned raw, const Hcs08FlashImage &image) {
  auto expected = caller(raw);
  ++expected.pc;
  require(unit.cpu.state() == expected && unit.memory.flash.image() == image &&
              unit.memory.retired == phases,
          "Native RTI restores independent caller/return PC;NVM remains exact");
}

void run_full(const Configuration config, const unsigned raw, const bool replay) {
  const auto image = own_image(config, program(raw));
  std::optional<Unit> original(std::in_place, image);
  std::vector<Saved> saved;
  for (unsigned phase = 0; phase < phases; ++phase) {
    if (replay) {
      saved.push_back(
          {.memory = original->memory, .cpu = original->cpu.checkpoint(), .phase = phase});
    }
    if (phase == setup) {
      enter(*original, config, raw);
    } else {
      original->cpu.step();
    }
  }
  returned(*original, raw, image);
  original.reset();
  for (const auto &snapshot : saved) {
    for (unsigned copy = 0; copy < 2; ++copy) {
      Unit fresh(snapshot);
      for (unsigned phase = snapshot.phase; phase < phases; ++phase) {
        if (phase == setup) {
          enter(fresh, config, raw);
        } else {
          fresh.cpu.step();
        }
      }
      returned(fresh, raw, image);
    }
  }
}

void cold(const Configuration config) {
  constexpr std::array<std::uint8_t, 3> code{0x83, 0x9d, 0x9d};
  const auto image = own_image(config, code);
  std::optional<Unit> original(std::in_place, image);
  auto &unit = *original;
  std::vector<Saved> saved;
  const Hcs08State before{.pc = entry,
                          .hx = 0,
                          .sp = byte_mask,
                          .a = 0,
                          .ccr_known = cold_flags,
                          .ccr = cold_flags,
                          .a_known = 0,
                          .hx_known = high_known};
  require(unit.cpu.state() == before, "Real cold reset leaves A/X unknown and only H known");
  unit.memory.accesses.clear();
  saved.push_back({.memory = unit.memory, .cpu = unit.cpu.checkpoint(), .phase = 0});
  unit.cpu.step();
  saved.push_back({.memory = unit.memory, .cpu = unit.cpu.checkpoint(), .phase = 1});
  const std::vector<Access> frame{
      {.address = 0xff, .value = 1, .write = true},
      {.address = 0xfe, .value = 0xe0, .write = true},
      {.address = 0xfd, .value = 0, .write = true, .known = 0},
      {.address = 0xfc, .value = 0, .write = true, .known = 0},
      {.address = 0xfb, .value = cold_flags, .write = true, .known = cold_flags},
      {.address = vector, .value = config.target >> 8U, .write = false},
      {.address = vector + 1, .value = 0, .write = false}};
  auto expected = before;
  expected.pc = static_cast<std::uint16_t>(config.target);
  constexpr unsigned entered_sp = 0xfa;
  expected.sp = entered_sp;
  require(unit.cpu.state() == expected && unit.memory.accesses == frame,
          "First-instruction SWI copies actual unknown frame masks without inventing values");
  unit.cpu.step();
  expected = before;
  ++expected.pc;
  require(unit.cpu.state() == expected && unit.memory.flash.image() == image,
          "Real RTI restores cold partial state exactly");
  original.reset();
  for (const auto &snapshot : saved) {
    for (unsigned copy = 0; copy < 2; ++copy) {
      Unit fresh(snapshot);
      if (snapshot.phase == 0) {
        fresh.cpu.step();
        auto entered = before;
        entered.pc = static_cast<std::uint16_t>(config.target);
        entered.sp = entered_sp;
        require(fresh.cpu.state() == entered && fresh.memory.accesses == frame,
                "Fresh CPU performs the same independent partial native frame and owned reads");
      }
      fresh.cpu.step();
      require(fresh.cpu.state() == expected && fresh.memory.flash.image() == image,
              "Destroyed-owner partial SWI/RTI checkpoints retain masks and real memory");
    }
  }
}

void changed_handler() {
  constexpr unsigned raw = 0x91;
  constexpr unsigned new_high = 0xab00;
  auto image = own_image(cases.front(), program(raw));
  constexpr std::array<std::uint8_t, 8> code{0x45, 0xab, 0xcd, 0xa6, 0x5a, 0x80, 0x9d, 0x9d};
  for (unsigned i = 0; i < code.size(); ++i) { put(image, handler + i, code[i]); }
  Unit unit(image);
  for (unsigned i = 0; i < setup; ++i) { unit.cpu.step(); }
  enter(unit, cases.front(), raw);
  for (unsigned i = 0; i < 3; ++i) { unit.cpu.step(); }
  auto expected = caller(raw);
  ++expected.pc;
  expected.hx = new_high | (caller_hx & byte_mask);
  require(unit.cpu.state() == expected && unit.memory.flash.image() == image,
          "RTI restores frame X/A/CCR while retaining the handler's actual unstacked H");
}

void missing_vector() {
  constexpr unsigned raw = 0x69;
  for (unsigned byte = 0; byte < 2; ++byte) {
    const auto config = cases.front();
    auto image = own_image(config, program(raw));
    image.known.reset(config.physical + byte - hcs08_flash::first);
    Unit unit(image);
    for (unsigned i = 0; i < setup; ++i) { unit.cpu.step(); }
    const auto error = rejected([&] { unit.cpu.step(); });
    require(error.contains("interrupt vector read") && error.contains("pc=" + hex(caller_pc)) &&
                error.contains(hex(config.physical + byte)) && error.contains(hex(vector + byte)) &&
                unit.memory.flash.image() == image,
            "Missing actual high/low vector byte refuses with logical/physical/PC context");
    require(rejected([&] { static_cast<void>(unit.cpu.checkpoint()); }).contains("healthy"),
            "Failed native vector read cannot publish a successful checkpoint");
  }
}

void obsolete_decoder(const std::string &spec) {
  constexpr unsigned raw = 0x68;
  constexpr std::array<std::uint8_t, 3> cold_code{0x83, 0x9d, 0x9d};
  for (const bool partial : {false, true}) {
    const auto full = program(raw);
    const auto image = own_image(cases.front(),
                                 partial ? std::span<const std::uint8_t>(cold_code)
                                         : std::span<const std::uint8_t>(full));
    Unit unit(image, spec);
    for (unsigned i = 0; i < (partial ? 0U : setup); ++i) { unit.cpu.step(); }
    const auto before = unit.cpu.state();
    const auto generation = unit.memory.generation;
    const auto error = rejected([&] { unit.cpu.step(); });
    require(error.contains("SWI decode") && error.contains("address=0xfffc") &&
                error.contains("pc=" + hex(before.pc)) && error.contains("agent=CPU") &&
                unit.cpu.state() == before && unit.memory.generation == generation &&
                unit.memory.accesses.empty() && unit.memory.vector_reads.empty() &&
                unit.memory.flash.image() == image,
            "Old constant-target decoder refuses before CPU/frame/vector effects for both callers");
  }
}
} // namespace

int main(const int argc, const char *const argv[]) {
  try {
    if (argc == 2) {
      obsolete_decoder(argv[1]);
      std::cout << "Verified old-decoder refusal before native SWI effects.\n";
      return 0;
    }
    require(argc == 1, "SWI test accepts only an optional prior decoder path");
    for (unsigned raw = 0; raw <= byte_mask; ++raw) { run_full(cases.front(), raw, false); }
    constexpr unsigned replay_flags = 0x91;
    for (const auto config : cases) {
      run_full(config, replay_flags, true);
      cold(config);
    }
    changed_handler();
    missing_vector();
    std::cout << "Verified256CCR inputs,six routing/cold cases,108destroyed-owner suffixes,"
                 "literal native frame/RTI/modified-H and missing-vector refusals.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
