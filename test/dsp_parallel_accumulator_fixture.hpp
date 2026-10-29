// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace blofeld::test::parallel_accumulator {
inline constexpr unsigned pointer = 0x401, offset = 3, canary = 0x345678;
inline constexpr unsigned first_word = 0x800001, second_word = 0x123456, tail = 16;
inline constexpr unsigned saturation_mode = 0x100000, extension_mask = 0xff;
inline constexpr unsigned all_ccr = 0xff;
inline constexpr std::uint64_t accumulator_range = std::uint64_t{1} << 56U;

struct Pair {
  std::uint64_t a = 0, b = 0;
};

// FM13-7/8: full56 ADD, including signed56 and fractional48 boundaries.
inline constexpr std::array operands{
    Pair{},
    Pair{.a = 1, .b = 2},
    Pair{.a = sign_bit - 1, .b = 1},
    Pair{.a = sign_bit, .b = accumulator_mask},
    Pair{.a = 0x007fffff000000, .b = 0x1000000},
    Pair{.a = 0xff800000000000, .b = accumulator_mask},
    Pair{.a = 0x12a1b2c3d4e5f6, .b = 0xe93f4a5b6c7d8e},
    Pair{.a = 0x123456789abc, .b = accumulator_range - 0x123456789abc}};

struct Input {
  Pair value;
  unsigned word = first_word, status = masked_status, reg = 4, mode = 3;
  bool b = false, y = false;
};

struct Sum {
  std::uint64_t value;
  unsigned status;
};

[[nodiscard]] Sum addition(Pair input, unsigned status);
[[nodiscard]] std::uint64_t loaded(unsigned word);
[[nodiscard]] unsigned opcode(Input input);
[[nodiscard]] unsigned address(Input input);
[[nodiscard]] unsigned updated_pointer(Input input);
[[nodiscard]] DspDebugState prepare(DspMachine &machine,
                                    Input input,
                                    std::span<const std::uint32_t> body,
                                    bool unknown = false);
void sources(DspMachine &machine, Input input);
void dependencies();
void checkpoints();
void guards();
}
