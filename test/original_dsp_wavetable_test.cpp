// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <set>
#include <span>
#include <string>
#include <vector>

namespace {
using blofeld::DspSpace;
using blofeld::test::require;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned memory_mode = 0x600389;
constexpr unsigned rounding_mode = 1U << 21;
constexpr unsigned program_base = 0x2718, program_words = 1086;
constexpr unsigned kernel = 0x2a4c, kernel_end = 0x2a77;
constexpr unsigned coefficients = 0xde96, coefficient_words = 1032;
constexpr unsigned slot = 0x120, fm_source = 0x180;
constexpr unsigned output = 0x1c0, fm_output = 0x1e0;
constexpr unsigned cache = 0xc800, frame_stride = 256, cache_words = 512;
constexpr unsigned control = 0x900, inherited_modifier = 31;
constexpr unsigned frequency = 0x23456;
constexpr unsigned fm_positive = 0x246800, fm_negative = 0xdb9800;
constexpr unsigned fm_quarter = 0x200000, fm_minus_quarter = 0xe00000;
constexpr unsigned most_positive = 0x7fffff, most_negative = 0x800000;
constexpr unsigned phase_fraction_witness = 0x823454;
constexpr std::uint64_t accumulator_mask = 0xffffffffffffff;
constexpr unsigned cache_guard = 0x5a6b7c, output_guard = 0x123456;
constexpr unsigned coefficient_pointer = 0x0c;
constexpr unsigned own_frame_stride = 7919, own_frame_bias = 1234, halfword_mask = 0xffff;
constexpr unsigned high_cache_word = 0x7fff00, low_cache_word = 0x800000;
constexpr unsigned instruction_bound = 180, original_instruction_starts = 42;
constexpr unsigned absolute_jump = 0x0af080, almost_unity = 0x7fffff;
// Independent source-row0 endpoint witness, under both original RM modes.
constexpr std::int64_t witness_first = 2766947, witness_second = -2767011;
constexpr std::int64_t witness_blend = -2767010;
constexpr std::int64_t unit = std::int64_t{1} << 24;
constexpr std::int64_t half = unit / 2;

std::int64_t signed_word(const unsigned raw) {
  return raw < half ? raw : static_cast<std::int64_t>(raw) - unit;
}

unsigned word(const std::int64_t value) {
  return static_cast<unsigned>(static_cast<std::uint64_t>(value) & word_mask);
}

std::int64_t floor_word(const std::int64_t value) {
  return value >= 0 ? value / unit : -((-value + unit - 1) / unit);
}

std::int64_t limited(const std::int64_t value) {
  return std::clamp(floor_word(value), -half, half - 1);
}

std::int64_t rounded(const std::int64_t value, const bool twos) {
  auto whole = floor_word(value);
  const auto fraction = value - (whole * unit);
  if (fraction > half || (fraction == half && (twos || whole % 2 != 0))) { ++whole; }
  return whole * unit;
}

const blofeld::DspRecord &
record(const blofeld::Firmware &firmware, const DspSpace space, const unsigned address) {
  const auto found = std::ranges::find_if(firmware.dsp, [&](const auto &entry) {
    return entry.space == static_cast<unsigned>(space) && entry.address == address;
  });
  require(found != firmware.dsp.end(), "Missing pinned original wavetable record");
  return *found;
}

struct Input {
  unsigned length, row, index, weight, pattern, fm_depth, fm_word;
  bool twos;
};

struct Expected {
  std::array<unsigned, cache_words> memory{};
  unsigned result = 0, phase = 0, row = 0, start = 0;
  std::int64_t first = 0, second = 0, phase_accumulator = 0;
  unsigned next_phase = 0;

  Expected(const Input input, const std::span<const unsigned> table) {
    memory.fill(cache_guard);
    row = coefficients + input.row * 8;
    const unsigned offset = frame_stride - (2 * input.length);
    start = cache + offset + input.index;
    const auto raw_phase = (static_cast<std::uint64_t>(input.index) * unit +
                            static_cast<std::uint64_t>(input.row) * 65536U) /
                           input.length;
    phase = static_cast<unsigned>(raw_phase);
    phase_accumulator = ((signed_word(phase) + frequency) * unit) +
                        (2 * signed_word(input.fm_depth) * signed_word(input.fm_word));
    next_phase = word(floor_word(phase_accumulator));
    for (unsigned stream = 0; stream < 2; ++stream) {
      for (unsigned index = 0; index < input.length; ++index) {
        unsigned value = (((index * own_frame_stride) + (stream * own_frame_bias)) & halfword_mask)
                         << 8U;
        if (input.pattern != 0) {
          const bool positive = signed_word(table[(input.row * 8) + (index % 8)]) >= 0;
          const bool high = positive != (stream != 0);
          value = high ? high_cache_word : low_cache_word;
        }
        memory[(stream * frame_stride) + offset + index] = value;
      }
    }
    for (unsigned tap = 0; tap < 8; ++tap) {
      const auto coefficient = signed_word(table[(input.row * 8) + tap]);
      const unsigned index = offset + ((input.index + tap) % input.length);
      first += 2 * coefficient * signed_word(memory[index]);
      second += 2 * coefficient * signed_word(memory[index + frame_stride]);
    }
    first = rounded(first, input.twos);
    second = rounded(second, input.twos);
    const auto blend = first + (2 * static_cast<std::int64_t>(input.weight) * limited(second)) -
                       (2 * static_cast<std::int64_t>(input.weight) * limited(first));
    result = word(limited(rounded(blend, input.twos)));
  }
};

struct Reads {
  std::vector<unsigned> coefficients, first, second;

  void observe(const blofeld::DspNativeAccess &access) {
    if (access.stage != blofeld::DspNativeAccessStage::after ||
        access.kind != blofeld::DspNativeAccessKind::read || access.pc < kernel ||
        access.pc >= kernel_end) {
      return;
    }
    const auto address = access.target.address;
    if (access.target.space == DspSpace::x && address >= ::coefficients &&
        address < ::coefficients + coefficient_words) {
      coefficients.push_back(address);
    }
    if (access.target.space == DspSpace::y && address >= cache && address < cache + cache_words) {
      (address < cache + frame_stride ? first : second).push_back(address);
    }
  }

  void verify(const Input input) const {
    require(coefficients.size() == 8 && first.size() == 8 && second.size() == 8,
            "Original FIR reads exactly eight coefficients and eight words per frame");
    const auto base = cache + frame_stride - (2 * input.length);
    for (unsigned tap = 0; tap < 8; ++tap) {
      const auto frame_address = base + ((input.index + tap) % input.length);
      require(coefficients[tap] == ::coefficients + (input.row * 8) + tap &&
                  first[tap] == frame_address && second[tap] == frame_address + frame_stride,
              "Original FIR ordered coefficient/circular frame reads");
    }
  }
};

class Fixture {
public:
  explicit Fixture(const blofeld::Firmware &firmware)
      : table_(record(firmware, DspSpace::x, coefficients).words), cpu_(prepare(firmware)) {
    require(table_.size() == coefficient_words, "Original coefficient extent differs");
  }

  void run(const Input input) {
    const Expected expected(input, table_);
    Reads reads;
    seed(input, expected);
    for (unsigned step = 0; cpu_.state().pc != stop_ || step == 0; ++step) {
      require(step < instruction_bound, "Original wavetable callback instruction bound");
      const auto pc = cpu_.state().pc;
      require((pc >= kernel && pc < kernel_end) ||
                  (pc >= blofeld::test::boot && pc <= continuation_),
              "Original wavetable callback left the declared program");
      if (pc >= kernel && pc < kernel_end) { visited_.insert(pc); }
      cpu_.step({.observe = [&](const auto &access) { reads.observe(access); }});
      bus_.advance_instruction_cycles(1);
      ++instructions_;
    }
    verify(input, expected);
    reads.verify(input);
    ++calls_;
  }

  void summary() const {
    require(visited_.size() == original_instruction_starts,
            "All original FIR instruction starts execute");
    std::cout << "calls=" << calls_ << " instructions=" << instructions_
              << " original-starts=" << visited_.size() << '\n';
  }

private:
  blofeld::DspBus &prepare(const blofeld::Firmware &firmware) {
    require(record(firmware, DspSpace::p, program_base).words.size() == program_words,
            "Original wavetable program extent differs");
    bus_.observe_omr(memory_mode);
    bus_.advance_instruction_cycles(3);
    for (const auto &entry : firmware.dsp) {
      for (unsigned index = 0; index < entry.words.size(); ++index) {
        put(static_cast<DspSpace>(entry.space), entry.address + index, entry.words[index]);
      }
    }
    blofeld::test::DspProgram code;
    code.immediate("x1", memory_mode);
    code.emit("move x1,omr");
    for (unsigned index = 0; index < 3; ++index) { code.emit("nop"); }
    const auto begin = code.pc();
    for (unsigned index = 0; index < 8; ++index) {
      code.immediate("m" + std::to_string(index), word_mask);
      code.immediate("n" + std::to_string(index), 0);
    }
    for (const auto index : {0U, 4U}) { code.immediate("r" + std::to_string(index), slot); }
    code.immediate("n0", 3);
    code.immediate("n4", 5);
    code.immediate("m2", inherited_modifier);
    code.immediate("r5", fm_source);
    code.immediate("r7", output);
    code.immediate("r3", fm_output);
    code.immediate("x1", 0);
    code.emit("move x:>" + blofeld::test::number(control) + ",x1");
    code.emit("move x1,sr");
    code.immediate("r1", kernel);
    code.emit("jsr (r1)");
    stop_ = code.pc();
    code.emit("nop");
    code.word(absolute_jump); // Own absolute JMP back to the next declared caller setup.
    code.word(begin);
    continuation_ = code.pc();
    code.emit("rts");
    code.install(bus_);
    return bus_;
  }

  void put(const DspSpace space, const unsigned address, const unsigned value) {
    bus_.write({.space = space, .address = address, .value = value});
  }

  unsigned get(const DspSpace space, const unsigned address) {
    return bus_.read({.space = space, .address = address});
  }

  void seed(const Input input, const Expected &expected) {
    put(DspSpace::x, control, input.twos ? rounding_mode : 0);
    put(DspSpace::x, coefficient_pointer, coefficients);
    put(DspSpace::x, fm_source, input.fm_word);
    for (unsigned index = 0; index < 9; ++index) {
      put(DspSpace::x, slot + index, output_guard);
      put(DspSpace::y, slot + index, 0);
    }
    put(DspSpace::x, slot, expected.phase);
    put(DspSpace::y, slot, input.length / 2);
    put(DspSpace::x, slot + 1, continuation_);
    put(DspSpace::y, slot + 1, frequency);
    put(DspSpace::y, slot + 2, fm_source);
    put(DspSpace::x, slot + 3, cache + frame_stride - (2 * input.length));
    put(DspSpace::y, slot + 3, frame_stride);
    put(DspSpace::x, slot + 4, input.length - 1);
    put(DspSpace::y, slot + 5, input.weight);
    put(DspSpace::y, slot + 7, input.fm_depth);
    for (unsigned index = 0; index < cache_words; ++index) {
      put(DspSpace::y, cache + index, expected.memory[index]);
    }
    for (const auto address : {output - 1, output, output + 1}) {
      put(DspSpace::x, address, output_guard);
    }
    for (const auto address : {fm_output - 1, fm_output, fm_output + 1}) {
      put(DspSpace::y, address, output_guard);
    }
  }

  void verify(const Input input, const Expected &expected) {
    const auto actual = get(DspSpace::x, output);
    require(actual == expected.result,
            "Independent FIR mismatch case=" + std::to_string(calls_) + " actual=" +
                std::to_string(actual) + " expected=" + std::to_string(expected.result));
    require(get(DspSpace::x, slot + 5) == expected.result,
            "Original FIR exposed sample equals its rounded/limited output");
    require(get(DspSpace::x, slot) == expected.next_phase,
            "Original FIR native phase/frequency update");
    require(get(DspSpace::y, fm_output) == input.fm_depth, "Original signed FM side output");
    require(cpu_.state().a ==
                (static_cast<std::uint64_t>(expected.phase_accumulator) & accumulator_mask),
            "Independent full native phase accumulator,including fractional product/extension");
    if (input.index == 0 && input.row == 0 && input.fm_depth == most_positive &&
        input.fm_word == most_positive) {
      require(expected.next_phase == phase_fraction_witness && (cpu_.state().a & word_mask) == 2,
              "Literal positive-extension/A0=2 phase witness bypasses full-word clipping");
    }
    for (const auto address : {output - 1, output + 1}) {
      require(get(DspSpace::x, address) == output_guard, "FIR output guard changed");
    }
    for (const auto address : {fm_output - 1, fm_output + 1}) {
      require(get(DspSpace::y, address) == output_guard, "FIR FM guard changed");
    }
    for (unsigned index = 0; index < cache_words; ++index) {
      require(get(DspSpace::y, cache + index) == expected.memory[index], "FIR cache write");
    }
    const auto state = cpu_.state();
    require(state.sp == 0 && state.pc == stop_ && state.m[1] == word_mask &&
                state.m[2] == inherited_modifier && state.n[1] == inherited_modifier &&
                state.r[0] == slot + 8 && state.r[4] == slot + 8 && state.r[5] == fm_source &&
                state.r[6] == slot,
            "Original FIR native stack/cursors/modifier restoration");
    if (input.length == 128 && input.row == 0 && input.index == 0 && input.pattern == 1 &&
        input.weight == almost_unity) {
      require(limited(expected.first) == witness_first &&
                  limited(expected.second) == witness_second &&
                  signed_word(actual) == witness_blend,
              "Original near-unity blend retains the independent one-LSB endpoint witness");
    }
  }

  std::span<const unsigned> table_;
  blofeld::DspBus bus_;
  unsigned stop_ = 0, continuation_ = 0;
  blofeld::DspCore cpu_;
  unsigned calls_ = 0;
  std::uint64_t instructions_ = 0;
  std::set<unsigned> visited_;
};

void run_baseline(Fixture &fixture) {
  for (const bool twos : {false, true}) {
    for (const unsigned length : {2U, 4U, 8U, 16U, 32U, 64U, 128U}) {
      for (const unsigned row : {0U, 1U, 63U, 64U, 127U, 128U}) {
        for (const unsigned weight : {0U, 1U, 0x100000U, 0x400000U, most_positive}) {
          for (const unsigned pattern : {0U, 1U}) {
            for (const unsigned index : {0U, length - 1}) {
              fixture.run({.length = length,
                           .row = row,
                           .index = index,
                           .weight = weight,
                           .pattern = pattern,
                           .fm_depth = 0,
                           .fm_word = fm_positive,
                           .twos = twos});
            }
          }
        }
      }
    }
  }
}

void run_fm_inputs(Fixture &fixture, Input input) {
  for (const unsigned depth : {fm_quarter, fm_minus_quarter, most_positive, most_negative}) {
    for (const unsigned value : {fm_positive, fm_negative, most_positive, most_negative}) {
      input.fm_depth = depth;
      input.fm_word = value;
      fixture.run(input);
    }
  }
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Usage: private-wavetable-probe original.zip");
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    Fixture fixture(firmware);
    run_baseline(fixture);
    for (const bool twos : {false, true}) {
      for (const unsigned length : {2U, 16U, 128U}) {
        for (const unsigned row : {0U, 128U}) {
          for (const unsigned weight : {0U, most_positive}) {
            for (const unsigned index : {0U, length - 1}) {
              run_fm_inputs(fixture,
                            {.length = length,
                             .row = row,
                             .index = index,
                             .weight = weight,
                             .pattern = 0,
                             .fm_depth = 0,
                             .fm_word = 0,
                             .twos = twos});
            }
          }
        }
      }
    }
    fixture.summary();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
