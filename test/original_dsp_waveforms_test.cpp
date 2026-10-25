// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <span>
#include <string>
#include <vector>

namespace {
using blofeld::DspSpace;
using blofeld::test::require;
constexpr unsigned mask = 0xffffff;
constexpr unsigned word_bits = 24;
constexpr unsigned memory_mode = 0x600389;
constexpr unsigned rounding_mode = 1U << 21;
constexpr unsigned program_base = 0x2d73;
constexpr unsigned program_words = 501;
constexpr unsigned scalar_base = 0xbc5;
constexpr unsigned scalar_words = 289;
constexpr unsigned cubic_base = 0x13f4;
constexpr unsigned cubic_words = 258;
constexpr unsigned generator = 0x2ec3;
constexpr unsigned generator_end = 0x2efa;
constexpr unsigned recurrence_complete = 0x2ed3;
constexpr unsigned term_complete = 0x2ef2;
constexpr unsigned bank_complete = 0x2ef4;
constexpr unsigned sine_base = 0xc800;
constexpr unsigned sine_length = 1024;
constexpr unsigned bank_base = 0xce6;
constexpr unsigned bank_count = 7;
constexpr unsigned sample_count = 256;
constexpr unsigned bank_stride = sample_count + 2;
constexpr unsigned bank_words = bank_count * bank_stride;
constexpr unsigned counts_base = 0xcb9;
constexpr unsigned coefficients_base = 0xcc0;
constexpr unsigned sine_seed = 0x78c7ac;
constexpr unsigned sine_feedback = 0x54c362;
constexpr unsigned sine_stride = 201;
constexpr unsigned target_slot = 0x800;
constexpr unsigned status_slot = target_slot + 1;
constexpr unsigned modifier_slot = target_slot + 2;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned instruction_bound = 100000;
constexpr unsigned term_count = 99;
constexpr unsigned case_count = 8;
constexpr unsigned instruction_starts = 42;
constexpr unsigned witness_count = 1474;
constexpr unsigned final_sine_r3 = 154;
constexpr unsigned final_sine_r4 = 450;
constexpr unsigned final_sine_y1 = 150;
constexpr unsigned final_b = 308;
constexpr unsigned final_n4 = 300;
constexpr unsigned status_mask = 0xffff00;
constexpr std::int64_t unit = std::int64_t{1} << word_bits;
constexpr std::int64_t half = unit / 2;
constexpr std::uint64_t accumulator_mask = 0xffffffffffffff;
using Samples = std::array<unsigned, sample_count>;

std::int64_t signed_word(const unsigned value) {
  return value < half ? value : static_cast<std::int64_t>(value) - unit;
}

unsigned raw(const std::int64_t value) {
  return static_cast<unsigned>(static_cast<std::uint64_t>(value) & mask);
}

std::uint64_t full(const std::int64_t value) {
  return static_cast<std::uint64_t>(value) & accumulator_mask;
}

std::int64_t floor_div(const std::int64_t value, const std::int64_t divisor) {
  return value >= 0 ? value / divisor : -((-value + divisor - 1) / divisor);
}

std::int64_t store(const std::int64_t value) {
  return std::clamp(floor_div(value, unit), -half, half - 1);
}

std::int64_t round_word(const std::int64_t value, const bool twos, unsigned &ties) {
  auto integer = floor_div(value, unit);
  const auto fraction = value - (integer * unit);
  if (fraction == half) { ++ties; }
  if (fraction > half || (fraction == half && (twos || integer % 2 != 0))) { ++integer; }
  return integer * unit;
}

const blofeld::DspRecord &
record(const blofeld::Firmware &firmware, const DspSpace space, const unsigned address) {
  const auto found = std::ranges::find_if(firmware.dsp, [&](const auto &value) {
    return value.space == static_cast<unsigned>(space) && value.address == address;
  });
  require(found != firmware.dsp.end(), "Missing original waveform input record");
  return *found;
}

struct Images {
  std::span<const unsigned> program;
  std::span<const unsigned> scalars;
  std::span<const unsigned> cubic;

  explicit Images(const blofeld::Firmware &firmware)
      : program(record(firmware, DspSpace::p, program_base).words),
        scalars(record(firmware, DspSpace::y, scalar_base).words),
        cubic(record(firmware, DspSpace::y, cubic_base).words) {
    require(program.size() == program_words && scalars.size() == scalar_words &&
                cubic.size() == cubic_words,
            "Original waveform program/scalar/cubic extents must match");
  }

  [[nodiscard]] unsigned scalar(const unsigned address) const {
    require(address >= scalar_base && address - scalar_base < scalars.size(),
            "Independent waveform model scalar is outside the supplied original record");
    return scalars[address - scalar_base];
  }
};

struct Term {
  unsigned bank;
  unsigned index;
  unsigned body;
  unsigned stride;
  unsigned offset;
  std::int64_t accumulator;
  Samples values;
};

struct Model {
  std::array<unsigned, sine_length> sine{};
  std::array<unsigned, bank_words> banks{};
  std::vector<Term> terms;
  std::int64_t sine_a = static_cast<std::int64_t>(sine_seed) * unit;
  std::int64_t sine_previous = 0;
  unsigned ties = 0;
  unsigned floor_witnesses = 0;
  unsigned contribution_limits = 0;

  Model(const Images images, const bool twos) {
    for (unsigned index = 1; index < sine_length; ++index) {
      const auto current = store(sine_a);
      sine_a = round_word((-sine_previous * unit) +
                              (2 * static_cast<std::int64_t>(sine_feedback) * current),
                          twos,
                          ties);
      sine[(sine_stride * index) % sine_length] = raw(current);
      sine_previous = current;
    }
    terms.reserve(term_count);
    for (unsigned bank = 0; bank < bank_count; ++bank) { build_bank(images, bank); }
    require(terms.size() == term_count, "Independent model must contain all 99 harmonic stages");
  }

private:
  void build_bank(const Images images, const unsigned bank) {
    constexpr std::array<unsigned, bank_count> expected_counts{1, 3, 6, 10, 16, 25, 38};
    const auto count = images.scalar(counts_base + bank);
    require(count == expected_counts.at(bank), "Original harmonic-count scalar changed");
    const auto body = bank_base + 1 + (bank_stride * bank);
    Samples destination{};
    std::array<std::int64_t, sample_count> unquantized{};
    for (unsigned term = 0; term < count; ++term) {
      const auto coefficient = signed_word(images.scalar(coefficients_base + term));
      const auto offset = 2 + (4 * term);
      const auto stride = 4 + (8 * term);
      std::int64_t last = 0;
      for (unsigned index = 0; index < sample_count; ++index) {
        const auto sample = signed_word(sine[(offset + (index * stride)) % sine_length]);
        last = (signed_word(destination.at(index)) * unit) + (2 * coefficient * sample);
        const auto unbounded = floor_div(last, unit);
        contribution_limits += static_cast<unsigned>(unbounded >= half || unbounded < -half);
        destination.at(index) = raw(store(last));
        unquantized.at(index) += 2 * coefficient * sample;
      }
      terms.push_back({.bank = bank,
                       .index = term,
                       .body = body,
                       .stride = stride,
                       .offset = offset,
                       .accumulator = last,
                       .values = destination});
    }
    const auto at = body - bank_base;
    for (unsigned index = 0; index < sample_count; ++index) {
      banks.at(at + index) = destination.at(index);
      floor_witnesses +=
          static_cast<unsigned>(destination.at(index) != raw(store(unquantized.at(index))));
    }
    banks.at(at - 1) = destination.back();
    banks.at(at + sample_count) = destination.front();
  }
};

struct Input {
  bool twos;
  unsigned modifier;
  unsigned seed;
};

class Fixture {
public:
  explicit Fixture(const Images images) : images_(images), cpu_(prepare()) {}

  void run(const Input input) {
    const Model model(images_, input.twos);
    seed(input);
    unsigned terms = 0;
    unsigned banks = 0;
    bool recurrence = false;
    for (unsigned steps = 0; cpu_.state().pc != stop_ || steps == 0; ++steps) {
      require(steps < instruction_bound,
              "Original waveform generator did not return within its bound");
      const auto state = cpu_.state();
      if (state.pc == recurrence_complete) {
        require(!recurrence, "Original recurrence completes exactly once");
        verify_recurrence(model);
        recurrence = true;
      }
      if (state.pc == term_complete) {
        require(recurrence && terms < model.terms.size(), "Original harmonic stage is unexpected");
        verify_term(model.terms.at(terms++), model);
      }
      if (state.pc == bank_complete) {
        require(banks < bank_count && state.sp == 3,
                "Original outer harmonic DO frame must remain");
        ++banks;
      }
      if (state.pc >= generator && state.pc < generator_end) { visited_.insert(state.pc); }
      cpu_.step();
      bus_.advance_instruction_cycles(1);
      ++instructions_;
    }
    require(recurrence && terms == term_count && banks == bank_count,
            "Every original recurrence/harmonic/bank loop must complete");
    verify_return(model, input);
    intact();
    ++calls_;
    terms_ += terms;
    require(model.floor_witnesses == witness_count && model.ties == 0 &&
                model.contribution_limits == 0,
            "Independent quantization witnesses must retain their exact scope");
  }

  void summary(std::ostream &trace) const {
    require(calls_ == case_count && terms_ == case_count * term_count &&
                visited_.size() == instruction_starts,
            "Every declared original call, harmonic stage and instruction start executes");
    trace << "calls=" << calls_ << " harmonic-stages=" << terms_
          << " final-words=" << calls_ * (sine_length + bank_words)
          << " intermediate-samples=" << terms_ * sample_count
          << " instruction-starts=" << visited_.size() << " instructions=" << instructions_ << '\n';
    trace << "visited" << std::hex;
    for (const auto pc : visited_) { trace << ' ' << pc; }
    trace << std::dec << '\n';
  }

private:
  blofeld::DspBus &prepare() {
    bus_.observe_omr(memory_mode);
    bus_.advance_instruction_cycles(3);
    install(DspSpace::p, program_base, images_.program);
    install(DspSpace::y, scalar_base, images_.scalars);
    install(DspSpace::y, cubic_base, images_.cubic);
    blofeld::test::DspProgram code;
    code.immediate("x1", memory_mode);
    code.emit("move x1,omr");
    for (unsigned index = 0; index < 3; ++index) { code.emit("nop"); }
    const auto begin = code.pc();
    for (unsigned index = 0; index < 8; ++index) {
      if (index != 4) { code.immediate("m" + std::to_string(index), mask); }
    }
    code.emit("move x:>" + blofeld::test::number(modifier_slot) + ",x1");
    code.emit("move x1,m4");
    code.emit("move x:>" + blofeld::test::number(status_slot) + ",x1");
    code.emit("move x1,sr");
    code.emit("move x:>" + blofeld::test::number(target_slot) + ",r1");
    code.emit("jsr (r1)");
    stop_ = code.pc();
    code.emit("nop");
    code.word(absolute_jump);
    code.word(begin);
    code.install(bus_);
    return bus_;
  }

  void install(const DspSpace space, const unsigned base, const std::span<const unsigned> words) {
    for (std::size_t index = 0; index < words.size(); ++index) {
      bus_.write(
          {.space = space, .address = base + static_cast<unsigned>(index), .value = words[index]});
    }
  }

  unsigned read(const unsigned address) {
    return bus_.read({.space = DspSpace::y, .address = address});
  }

  void write(const DspSpace space, const unsigned address, const unsigned value) {
    bus_.write({.space = space, .address = address, .value = value});
  }

  void seed(const Input input) {
    for (unsigned index = 0; index < sine_length; ++index) {
      write(DspSpace::y, sine_base + index, input.seed);
    }
    for (unsigned index = 0; index < bank_words; ++index) {
      write(DspSpace::y, bank_base + index, input.seed);
    }
    write(DspSpace::y, sine_base - 1, input.seed);
    write(DspSpace::y, sine_base + sine_length, input.seed);
    write(DspSpace::x, target_slot, generator);
    write(DspSpace::x, status_slot, input.twos ? rounding_mode : 0);
    write(DspSpace::x, modifier_slot, input.modifier);
  }

  void verify_recurrence(const Model &model) {
    for (unsigned index = 0; index < sine_length; ++index) {
      require(read(sine_base + index) == model.sine.at(index),
              "Independent recurrence sample differs");
    }
    constexpr unsigned recurrence_modifier = 0x83ff;
    const auto state = cpu_.state();
    require(state.a == full(model.sine_a) && (state.y & mask) == raw(model.sine_previous) &&
                state.r[4] == sine_base && state.n[4] == sine_stride &&
                state.m[4] == recurrence_modifier && state.sp == 1,
            "Original recurrence accumulator/cursor/modulo/JSR frame differs");
  }

  void verify_term(const Term &term, const Model &model) {
    for (unsigned index = 0; index < sample_count; ++index) {
      require(read(term.body + index) == term.values.at(index),
              "Independent harmonic sample differs");
    }
    const auto state = cpu_.state();
    require(state.a == full(term.accumulator) &&
                state.b == full(static_cast<std::int64_t>(term.stride) * unit) &&
                state.r[1] == term.body + sample_count && state.r[2] == term.body &&
                state.r[3] == sine_base + term.offset &&
                state.r[4] == sine_base + ((term.offset + ((sample_count + 1) * term.stride)) %
                                           sine_length) &&
                state.r[5] == coefficients_base + 1 + term.index && state.n[4] == term.stride &&
                state.sp == 5 &&
                (state.y & mask) == images_.scalar(coefficients_base + term.index) &&
                (state.y >> word_bits) == model.sine.at(term.offset),
            "Original harmonic accumulator/prefetch/cursors/three nested loop frames differ");
  }

  void verify_return(const Model &model, const Input input) {
    for (unsigned index = 0; index < sine_length; ++index) {
      require(read(sine_base + index) == model.sine.at(index), "Final recurrence sample changed");
    }
    for (unsigned index = 0; index < bank_words; ++index) {
      require(read(bank_base + index) == model.banks.at(index),
              "Final harmonic sample/guard differs");
    }
    require(read(sine_base - 1) == input.seed && read(sine_base + sine_length) == input.seed,
            "Original recurrence writes preserve adjacent guards");
    const auto state = cpu_.state();
    require(state.sp == 0 && state.a == full(model.terms.back().accumulator) &&
                state.b == full(static_cast<std::int64_t>(final_b) * unit) &&
                state.r[0] == coefficients_base && state.r[1] == cubic_base + 1 &&
                state.r[2] == bank_base + ((bank_count - 1) * bank_stride) &&
                state.r[3] == sine_base + final_sine_r3 &&
                state.r[4] == sine_base + final_sine_r4 && state.r[5] == bank_base &&
                state.n[1] == 3 && state.n[3] == 4 && state.n[4] == final_n4 &&
                state.m[0] == mask && state.m[4] == mask && (state.x & mask) == half &&
                (state.x >> word_bits) == sine_feedback &&
                (state.y & mask) == model.banks.at(bank_words - 2) &&
                (state.y >> word_bits) == model.sine.at(final_sine_y1) &&
                (state.sr & status_mask) == (input.twos ? rounding_mode : 0),
            "Original waveform RTS/full accumulator/register return differs");
  }

  void intact() {
    for (std::size_t index = 0; index < images_.program.size(); ++index) {
      require(bus_.peek_program(program_base + static_cast<unsigned>(index)) ==
                  images_.program[index],
              "Original waveform program was modified");
    }
    for (std::size_t index = 0; index < images_.scalars.size(); ++index) {
      require(read(scalar_base + static_cast<unsigned>(index)) == images_.scalars[index],
              "Original scalar/coefficient record was modified");
    }
    for (std::size_t index = 0; index < images_.cubic.size(); ++index) {
      require(read(cubic_base + static_cast<unsigned>(index)) == images_.cubic[index],
              "Adjacent original cubic table was modified");
    }
  }

  const Images images_;
  blofeld::DspBus bus_;
  unsigned stop_ = 0;
  blofeld::DspCore cpu_;
  unsigned calls_ = 0;
  unsigned terms_ = 0;
  std::uint64_t instructions_ = 0;
  std::set<unsigned> visited_;
};
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Usage: original-dsp-waveforms firmware.zip private.trace");
    std::ofstream trace(arguments[2]);
    require(trace.is_open(), "Cannot create private original waveform evidence");
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    const auto firmware = blofeld::Firmware::load(std::filesystem::path(arguments[1]));
    Fixture fixture{Images(firmware)};
    constexpr unsigned guard_seed = 0x13579b;
    constexpr unsigned negative_seed = 0x800000;
    for (const bool twos : {false, true}) {
      for (const unsigned modifier : {mask, 0U}) {
        for (const unsigned seed : {guard_seed, negative_seed}) {
          fixture.run({.twos = twos, .modifier = modifier, .seed = seed});
        }
      }
    }
    fixture.summary(trace);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
