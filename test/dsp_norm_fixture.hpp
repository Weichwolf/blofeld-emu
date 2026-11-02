// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp56kEmu/assembler.h"
#include "dsp_norm_expectations.hpp" // IWYU pragma: export

#include <cstdint>
#include <string>
#include <vector>

namespace blofeld::test {
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

}
