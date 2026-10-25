// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <string>

namespace blofeld::test::peripheral_pipeline {
inline void register_encodings() {
  struct Encoding {
    const char *source;
    std::array<unsigned, 4> opcodes;
  };

  // Independent Freescale asm56300 6.3.28 encodings of our own instructions.
  constexpr std::array encodings{
      Encoding{.source = "x0", .opcodes = {0x08c43c, 0x09c43c, 0x04c490, 0x04c43f}},
      Encoding{.source = "x1", .opcodes = {0x08c53c, 0x09c53c, 0x04c590, 0x04c53f}},
      Encoding{.source = "y0", .opcodes = {0x08c63c, 0x09c63c, 0x04c690, 0x04c63f}},
      Encoding{.source = "y1", .opcodes = {0x08c73c, 0x09c73c, 0x04c790, 0x04c73f}},
      Encoding{.source = "r0", .opcodes = {0x08d03c, 0x09d03c, 0x04d090, 0x04d03f}},
      Encoding{.source = "r1", .opcodes = {0x08d13c, 0x09d13c, 0x04d190, 0x04d13f}},
      Encoding{.source = "r2", .opcodes = {0x08d23c, 0x09d23c, 0x04d290, 0x04d23f}},
      Encoding{.source = "r3", .opcodes = {0x08d33c, 0x09d33c, 0x04d390, 0x04d33f}},
      Encoding{.source = "r4", .opcodes = {0x08d43c, 0x09d43c, 0x04d490, 0x04d43f}},
      Encoding{.source = "r5", .opcodes = {0x08d53c, 0x09d53c, 0x04d590, 0x04d53f}},
      Encoding{.source = "r6", .opcodes = {0x08d63c, 0x09d63c, 0x04d690, 0x04d63f}},
      Encoding{.source = "r7", .opcodes = {0x08d73c, 0x09d73c, 0x04d790, 0x04d73f}},
      Encoding{.source = "n0", .opcodes = {0x08d83c, 0x09d83c, 0x04d890, 0x04d83f}},
      Encoding{.source = "n1", .opcodes = {0x08d93c, 0x09d93c, 0x04d990, 0x04d93f}},
      Encoding{.source = "n2", .opcodes = {0x08da3c, 0x09da3c, 0x04da90, 0x04da3f}},
      Encoding{.source = "n3", .opcodes = {0x08db3c, 0x09db3c, 0x04db90, 0x04db3f}},
      Encoding{.source = "n4", .opcodes = {0x08dc3c, 0x09dc3c, 0x04dc90, 0x04dc3f}},
      Encoding{.source = "n5", .opcodes = {0x08dd3c, 0x09dd3c, 0x04dd90, 0x04dd3f}},
      Encoding{.source = "n6", .opcodes = {0x08de3c, 0x09de3c, 0x04de90, 0x04de3f}},
      Encoding{.source = "n7", .opcodes = {0x08df3c, 0x09df3c, 0x04df90, 0x04df3f}},
      Encoding{.source = "m0", .opcodes = {0x08e03c, 0x09e03c, 0x04e090, 0x04e03f}},
      Encoding{.source = "m1", .opcodes = {0x08e13c, 0x09e13c, 0x04e190, 0x04e13f}},
      Encoding{.source = "m2", .opcodes = {0x08e23c, 0x09e23c, 0x04e290, 0x04e23f}},
      Encoding{.source = "m3", .opcodes = {0x08e33c, 0x09e33c, 0x04e390, 0x04e33f}},
      Encoding{.source = "m4", .opcodes = {0x08e43c, 0x09e43c, 0x04e490, 0x04e43f}},
      Encoding{.source = "m5", .opcodes = {0x08e53c, 0x09e53c, 0x04e590, 0x04e53f}},
      Encoding{.source = "m6", .opcodes = {0x08e63c, 0x09e63c, 0x04e690, 0x04e63f}},
      Encoding{.source = "m7", .opcodes = {0x08e73c, 0x09e73c, 0x04e790, 0x04e73f}},
  };
  constexpr std::array spaces{DspSpace::x, DspSpace::y, DspSpace::x, DspSpace::y};
  constexpr std::array addresses{0xfffffcU, 0xfffffcU, 0xffff90U, 0xffff9fU};
  for (const auto &sample : encodings) {
    for (std::size_t index = 0; index < sample.opcodes.size(); ++index) {
      const auto space = spaces.at(index);
      const std::string name = space == DspSpace::x ? "x" : "y";
      const auto instruction =
          "movep " + std::string(sample.source) + ',' + name + ":<<" + number(addresses.at(index));
      const auto words = checkpoint::assemble(instruction.c_str());
      const auto form = decode_memory_stage(sample.opcodes.at(index));
      require(std::ranges::equal(words, std::array{sample.opcodes.at(index)}) && form &&
                  form->peripheral && form->peripheral_register &&
                  form->peripheral->space == space &&
                  form->peripheral->address == addresses.at(index),
              "Each register/port form matches its independently assembled single-word oracle");
    }
  }
}
}
