// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "dsp56kEmu/assembler.h"
#include "dsp_norm_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test::checkpoint {
inline constexpr unsigned data = 0x100;
inline constexpr unsigned cookie = 0x123456;
inline constexpr unsigned other_cookie = 0x654321;
inline constexpr unsigned iprp = 0xfffffe;
inline constexpr unsigned csr0 = 0xffff8f;
inline constexpr unsigned csr2 = 0xffff87;
inline constexpr unsigned timer_vector = 0x54;
inline constexpr unsigned second_vector = 0x5c;
inline constexpr unsigned handler = 0x300;
inline constexpr unsigned nested_handler = 0x320;
inline constexpr unsigned ordinary_status = 0xc00000;
inline constexpr unsigned mask_bits = 0x300;

inline std::string rejects(const auto &operation, const char *message) {
  try {
    operation();
  } catch (const Fault &fault) { return fault.what(); }
  throw Fault(message);
}

inline std::vector<std::uint32_t> assemble(const char *source) {
  const dsp56k::Assembler assembler;
  const auto instruction = assembler.assemble(source);
  require(instruction.success(),
          "Cannot assemble own DSP checkpoint diagnostic: " + std::string(source));
  std::vector<std::uint32_t> result;
  result.reserve(instruction.wordCount);
  for (unsigned index = 0; index < instruction.wordCount; ++index) {
    result.push_back(instruction.word[index]);
  }
  return result;
}

inline void ram(DspBus &bus, const unsigned address, const std::span<const std::uint32_t> words) {
  for (std::size_t index = 0; index < words.size(); ++index) {
    bus.write({.address = address + static_cast<unsigned>(index), .value = words[index]});
  }
}

inline void run(DspMachine &machine, const unsigned steps) {
  for (unsigned index = 0; index < steps; ++index) { machine.cpu().step(); }
}

inline void until(DspMachine &machine, const unsigned pc) {
  constexpr unsigned limit = 128;
  unsigned steps = 0;
  while (machine.cpu().state().pc != pc) {
    require(++steps <= limit, "Own DSP checkpoint setup exceeded its instruction bound");
    machine.cpu().step();
  }
}

void test_checkpoint_peripherals();
void test_checkpoint_interrupts();
}
