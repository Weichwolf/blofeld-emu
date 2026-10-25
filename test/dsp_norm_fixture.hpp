// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp56kEmu/assembler.h"

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace blofeld::test {
inline constexpr unsigned boot = 0xff0000;
inline constexpr unsigned reset_omr = 0x305; // FM figure 5-3/table 5-2, mode-five pins.
inline constexpr unsigned word_mask = 0xffffff;
inline constexpr unsigned word_bits = 24;
inline constexpr unsigned data_bits = word_bits * 2;
inline constexpr std::uint64_t accumulator_mask = 0xffffffffffffff;
inline constexpr std::uint64_t sign_bit = 0x80000000000000;
inline constexpr unsigned carry = 1;
inline constexpr unsigned overflow = 2;
inline constexpr unsigned zero = 4;
inline constexpr unsigned negative = 8;
inline constexpr unsigned unnormalized = 0x10;
inline constexpr unsigned extension = 0x20;
inline constexpr unsigned limit = 0x40;
inline constexpr unsigned sticky = 0x80;
inline constexpr unsigned scale_down = 0x400;
inline constexpr unsigned scale_up = 0x800;
inline constexpr unsigned compatibility = 0x2000;
inline constexpr unsigned masked_status = 0xc00300;
inline constexpr unsigned address_seed = 0x135790;
inline constexpr unsigned offset_seed = 0x246890;
inline constexpr unsigned modifier_seed = 7;
inline constexpr std::uint64_t other_accumulator = 0x34567890123456;

inline void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

inline std::string number(const std::uint64_t value) {
  std::ostringstream text;
  text << '$' << std::hex << value;
  return text.str();
}

class DspProgram {
public:
  void word(const std::uint32_t value) {
    require(value <= word_mask, "Own DSP diagnostic word exceeds 24 bits");
    words_.push_back(value);
  }

  void emit(const std::string &text) {
    const auto instruction = assembler_.assemble(text.c_str());
    require(instruction.success(), "Cannot assemble own DSP diagnostic: " + text);
    for (unsigned index = 0; index < instruction.wordCount; ++index) {
      words_.push_back(instruction.word[index]);
    }
  }

  void immediate(const std::string &reg, const std::uint64_t value) {
    emit("move #>" + number(value) + ',' + reg);
  }

  [[nodiscard]] unsigned pc() const { return boot + static_cast<unsigned>(words_.size()); }

  void install(DspBus &bus) const { bus.install_rom({.start = boot, .words = words_}); }

private:
  dsp56k::Assembler assembler_;
  std::vector<std::uint32_t> words_;
};

struct NormInput {
  std::uint64_t value = 0;
  unsigned counter = 0;
  unsigned status = 0;
  unsigned address_register = 0;
  bool accumulator_b = false;
};

struct NormResult {
  std::uint64_t value = 0;
  unsigned counter = 0;
  unsigned status = 0;
};

// Independent bit definitions: DSP56300FM Rev. 5 sections 5.4.1.2, 12.4.2
// and NORM 13-146. Integer/fraction boundaries move with the scaling mode.
inline unsigned value_flags(const NormInput input) {
  constexpr unsigned ordinary_top = 47;
  constexpr unsigned up_top = 46;
  constexpr unsigned down_top = 48;
  unsigned fraction_top = ordinary_top;
  if ((input.status & scale_up) != 0) {
    fraction_top = up_top;
  } else if ((input.status & scale_down) != 0) {
    fraction_top = down_top;
  }
  const auto value = input.value;
  const auto integer = value >> fraction_top;
  const auto integer_mask = (std::uint64_t{1} << (56 - fraction_top)) - 1;
  unsigned result = 0;
  if (integer != 0 && integer != integer_mask) { result |= extension; }
  if (((value >> fraction_top) & 1U) == ((value >> (fraction_top - 1)) & 1U)) {
    result |= unnormalized;
  }
  if ((value & sign_bit) != 0) { result |= negative; }
  if (value == 0) { result |= zero; }
  return result;
}

inline NormResult expected_norm(const NormInput input) {
  NormResult result{.value = input.value, .counter = input.counter, .status = input.status};
  const unsigned counter_mask = (input.status & compatibility) != 0 ? 0xffff : word_mask;
  bool did_overflow = false;
  if ((input.status & extension) != 0) {
    result.value = (input.value >> 1U) | (input.value & sign_bit);
    result.counter = (input.counter + 1) & counter_mask;
  } else if ((input.status & (unnormalized | zero)) == unnormalized) {
    result.value = (input.value << 1U) & accumulator_mask;
    result.counter = (input.counter + word_mask) & counter_mask;
    did_overflow = ((result.value ^ input.value) & sign_bit) != 0;
  }
  constexpr unsigned changed = extension | unnormalized | negative | zero | overflow;
  result.status =
      (input.status & ~changed) | value_flags({.value = result.value, .status = input.status});
  if (did_overflow) { result.status |= overflow | limit; }
  return result;
}
}
