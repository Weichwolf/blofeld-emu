// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::test::short_load {
inline constexpr unsigned entry = 0x100, canary = 0x5a5a5a, ordinary_omr = 0x300;
inline constexpr unsigned absolute_jump = 0x0af080, address_shift = 8, last_cell = 64;
inline constexpr unsigned ordinary_y = 0x80000, control_y = 0x40, register_shift = 16;
inline constexpr unsigned m_begin = 24;
inline constexpr unsigned r_base = 0x608000, n_base = 0x708000, m_base = 0x058020;
inline constexpr unsigned r_seed_base = 0x123, r_seed_stride = 0x100, n_seed_base = 3;
inline constexpr unsigned m_seed_base = 0xff0007, extension_mask = 0xff, sign_mask = 0x800000;
inline constexpr std::uint64_t a_seed = 0x7a123456654321, b_seed = 0x90112233445566;
inline constexpr std::array data_names{"x0", "x1", "y0", "y1"};
inline constexpr std::array data_seeds{0x112233U, 0x445566U, 0x778899U, 0xaabbccU};
inline constexpr std::array accumulator_names{"a0", "a1", "a2", "b0", "b1", "b2", "a", "b"};
// Independent FM13-118/122 and MOVEC13-131 literal fields;no SDK decoding oracle.
inline constexpr std::array accumulator_words{
    0x508000U, 0x548000U, 0x528000U, 0x518000U, 0x558000U, 0x538000U, 0x568000U, 0x578000U};
inline constexpr std::array addresses{0U, 1U, 31U, 63U};
inline constexpr std::array values{0U, 0x7fffffU, 0x800000U, 0xffffffU, 0x1234abU, 0x812345U};
inline constexpr std::array flags{0U, 0x3fU, 0xc0U, 0xffU};

struct Case {
  unsigned destination = 0, address = 0, value = 0, ccr = 0;
  bool y = false, small_offsets = false;
};

inline constexpr unsigned data_begin = 32, destination_count = 36;
unsigned r_seed(unsigned index);
unsigned n_seed(unsigned index, bool small = false);
unsigned m_seed(unsigned index);
std::string name(unsigned destination);
unsigned opcode(Case sample);
DspDebugState prepare(DspMachine &machine, Case sample, bool known_source = true);
void destination(DspState &state, Case sample);
void memory(DspMachine &machine, Case sample, bool known_source = true);
DspFetchRunInputs contract();
void address(const DspAddressStageView &view, Case sample, bool published);
void period(DspFetchRun &run,
            DspMachine &machine,
            const DspDebugState &before,
            Case sample,
            unsigned count,
            bool known_source = true);
void begin(DspMachine &machine);
}
