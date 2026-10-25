// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_master_output_fixture.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp_master_output_model.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test::master {
namespace {
constexpr unsigned record_index = 44;
constexpr unsigned program_begin = 0x11d;
constexpr unsigned program_words = 793;
constexpr unsigned gain_slot = 0x17;
constexpr unsigned status_slot = 0x50;
constexpr unsigned phase_slot = 0x51;
constexpr unsigned unused = 0x765432;
// DSP56300FM DO uses the after-loop label minus one. The two history
// stores are outside the eight-instruction body: 14 + 16*8 + 3 retirements.
constexpr unsigned native_steps = 14 + (16 * 8) + 3;
constexpr unsigned midpoint = native_steps / 2;
constexpr unsigned instruction_limit = 256;
constexpr unsigned limit_flag = 0x40;

void put(DspBus &bus, const DspSpace space, const unsigned address, const unsigned value) {
  bus.write({.space = space, .address = address, .value = value});
}

unsigned get(DspBus &bus, const DspSpace space, const unsigned address) {
  return bus.read({.space = space, .address = address});
}

bool selected(const unsigned pc) {
  return pc >= entry && pc < end;
}

unsigned caller(DspBus &bus) {
  DspProgram program;
  program.emit("move x:>" + number(status_slot) + ",x1");
  program.emit("move x1,sr");
  for (unsigned index = 0; index < 8; ++index) {
    program.immediate("m" + std::to_string(index), word_mask);
  }
  program.immediate("r0", input_base);
  program.immediate("r1", output_base);
  program.emit("move x:>" + number(phase_slot) + ",n1");
  program.immediate("r3", entry);
  program.emit("jsr (r3)");
  const auto stop = program.pc();
  program.emit("nop");
  constexpr unsigned jump = 0x0af080;
  program.word(jump);
  program.word(boot);
  program.install(bus);
  return stop;
}

constexpr std::array<unsigned, 10> guards{0,
                                          2,
                                          gain_slot - 1,
                                          gain_slot + 1,
                                          history - 1,
                                          history + 2,
                                          input_base - 1,
                                          input_base + 34,
                                          output_base - 1,
                                          output_base + 64};
}

Fixture::Fixture(const Firmware &firmware) {
  const auto &record = firmware.dsp.at(record_index);
  require(record.space == 0 && record.address == program_begin &&
              record.words.size() == program_words,
          "The complete original master/core program record retains its pinned layout");
  program_ = record.words;
  // Scalar extension positions in the hash-pinned original record; no copied assets.
  constexpr unsigned current_extension = 0x27d;
  constexpr unsigned previous_extension = 0x27f;
  constexpr unsigned feedback_extension = 0x285;
  constexpr unsigned loop_extension = 0x28c;
  constexpr unsigned epilogue = 0x295;
  require(program_.at(loop_extension - program_begin) == epilogue - 1,
          "The original DO extension ends before the two history stores");
  coefficients_ = {.current = program_.at(current_extension - program_begin),
                   .previous = program_.at(previous_extension - program_begin),
                   .feedback = program_.at(feedback_extension - program_begin)};
  auto &bus = machine_.bus();
  for (unsigned i = 0; i < program_.size(); ++i) {
    put(bus, DspSpace::p, program_begin + i, program_.at(i));
  }
  stop_ = caller(bus);
  machine_.cpu().reset_spi_boot();
  baseline_.emplace(machine_.checkpoint());
}

void Fixture::seed(const Input &input, const Samples &source, const unsigned missing) {
  auto &bus = machine_.bus();
  constexpr unsigned gain_factor = 512;
  if (missing != 3) { put(bus, DspSpace::x, gain_slot, gain_factor * input.volume * input.volume); }
  put(bus, DspSpace::x, status_slot, input.scale | (input.twos ? rounding : 0));
  put(bus, DspSpace::x, phase_slot, input.phase);
  for (unsigned i = 0; i < source.mixed.size(); ++i) {
    if (missing == 2 && i + 1 == source.mixed.size()) { continue; }
    put(bus, DspSpace::x, input_base + i, raw(source.mixed.at(i)));
  }
  if (!input.dependent) {
    output_.fill(guard);
    for (unsigned i = 0; i < output_.size(); ++i) { put(bus, DspSpace::x, output_base + i, guard); }
    if (missing != 1) { put(bus, DspSpace::y, output_base, unused); }
    for (unsigned lane = 0; lane < 2; ++lane) {
      if (missing != 4 || lane != 1) {
        put(bus, DspSpace::x, history + lane, raw(source.history.at(lane)));
      }
      if (missing != 5 || lane != 1) {
        put(bus, DspSpace::y, history + lane, raw(source.history.at(lane + 2)));
      }
    }
  }
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : guards) { put(bus, space, address, guard); }
  }
}

void Fixture::observe(const DspNativeAccess &access) {
  if (!selected(access.pc) || access.stage != DspNativeAccessStage::after ||
      (access.kind != DspNativeAccessKind::read && access.kind != DspNativeAccessKind::write)) {
    return;
  }
  if (!access.value) {
    throw Fault("A completed native data operation must retain its actual word");
  }
  effects_.data.push_back({.kind = access.kind,
                           .space = access.target.space,
                           .address = access.target.address,
                           .value = *access.value});
}

void Fixture::execute(const bool capture) {
  unsigned steps = 0;
  do {
    require(++steps <= instruction_limit,
            "The original output routine must return through its real RTS");
    const auto pc = machine_.cpu().state().pc;
    require(selected(pc) || pc >= boot,
            "The native output call cannot jump to an unrelated original callee");
    if (capture && effects_.steps == midpoint && !middle_) {
      middle_.emplace(Saved{.machine = machine_.checkpoint(), .effects = effects_});
    }
    machine_.cpu().step({.observe = [this](const auto &access) { observe(access); }});
    if (selected(pc)) { ++effects_.steps; }
  } while (machine_.cpu().state().pc != stop_);
}

namespace {
struct AccessCounts {
  unsigned reads = 0;
  unsigned writes = 0;
  unsigned input_reads = 0;
  unsigned output_writes = 0;
  unsigned unused_reads = 0;
  unsigned x_history = 0;
  unsigned y_history = 0;
  unsigned x_coefficient = 0;
  unsigned y_coefficient = 0;
};

void verify_read(const Access &effect, const Samples &source, AccessCounts &count) {
  ++count.reads;
  const bool scratch_history =
      effect.address == 1 || effect.address == history || effect.address == history + 1;
  if (effect.space == DspSpace::x) {
    const bool mixed = effect.address >= input_base && effect.address < input_base + input_words;
    require(mixed || scratch_history || effect.address == gain_slot,
            "Unexpected original X data read");
    if (mixed) {
      require(effect.address == input_base + count.input_reads &&
                  effect.value == raw(source.mixed.at(count.input_reads)),
              "All actual mix reads retain order, including the final prefetch");
      ++count.input_reads;
    }
  } else {
    require(scratch_history || effect.address == output_base, "Unexpected original Y data read");
    if (effect.address == output_base) {
      require(effect.value == unused, "The unused initial Y read must remain real and distinct");
      ++count.unused_reads;
    }
  }
}

void verify_history_write(const Access &effect,
                          const Samples &source,
                          const Prediction &expected,
                          AccessCounts &count) {
  if (effect.space == DspSpace::y) {
    require(effect.address == history + (count.y_history % 2) &&
                effect.value == raw(source.mixed.at(count.y_history)),
            "Input history stores differ");
    ++count.y_history;
    return;
  }
  require(count.x_history < 2 && effect.address == history + count.x_history &&
              effect.value == raw(expected.output.at(block_words - 2 + count.x_history)),
          "The two output history stores must execute after the hardware loop");
  ++count.x_history;
}

void verify_write(const Access &effect,
                  const Input &input,
                  const Samples &source,
                  const Prediction &expected,
                  const Coefficients &coefficients,
                  AccessCounts &count) {
  ++count.writes;
  if (effect.address == 1) {
    const auto value = effect.space == DspSpace::x ? expected.previous : coefficients.feedback;
    require(effect.value == raw(value), "Native coefficient stores differ");
    if (effect.space == DspSpace::x) {
      ++count.x_coefficient;
    } else {
      ++count.y_coefficient;
    }
    return;
  }
  if (effect.address == history || effect.address == history + 1) {
    verify_history_write(effect, source, expected, count);
    return;
  }
  require(effect.space == DspSpace::x && count.output_writes < expected.output.size() &&
              effect.address == output_base + input.phase + count.output_writes &&
              effect.value == raw(expected.output.at(count.output_writes)),
          "Every actual output store must match the independent recurrence");
  ++count.output_writes;
}
}

void Fixture::verify_accesses(const Input &input,
                              const Samples &source,
                              const Prediction &expected) const {
  AccessCounts count;
  for (const auto &effect : effects_.data) {
    require(effect.space == DspSpace::x || effect.space == DspSpace::y,
            "Original master output never writes program words");
    if (effect.kind == DspNativeAccessKind::read) {
      verify_read(effect, source, count);
    } else {
      verify_write(effect, input, source, expected, coefficients_, count);
    }
  }
  constexpr unsigned expected_reads = 134;
  constexpr unsigned expected_writes = (2 * block_words) + 4;
  require(count.reads == expected_reads && count.writes == expected_writes &&
              count.input_reads == input_words && count.output_writes == block_words &&
              count.unused_reads == 1 && count.x_history == 2 && count.y_history == block_words &&
              count.x_coefficient == 1 && count.y_coefficient == 1,
          "Complete original data access footprint differs");
}

void Fixture::verify(const Input &input, const Samples &source, const Prediction &expected) {
  const auto state = machine_.cpu().state();
  require(effects_.steps == native_steps && state.pc == stop_ && state.sp == 0 &&
              state.r[0] == input_base + input_words && state.r[1] == output_base + input.phase &&
              state.r[2] == history + 1 && state.r[4] == 1 &&
              state.r[5] == output_base + input.phase + 32 && state.n[1] == input.phase,
          "Native return/pointers differ: steps=" + std::to_string(effects_.steps) +
              " r0=" + hex(state.r[0]) + " r1=" + hex(state.r[1]) + " r2=" + hex(state.r[2]) +
              " r4=" + hex(state.r[4]) + " r5=" + hex(state.r[5]) + " sp=" + hex(state.sp));
  require(state.a == accumulator(expected.a) && state.b == accumulator(expected.b) &&
              state.x == ((static_cast<std::uint64_t>(raw(expected.current)) << word_bits) |
                          raw(expected.output.at(block_words - 2))) &&
              state.y == ((static_cast<std::uint64_t>(raw(source.mixed.at(block_words - 2)))
                           << word_bits) |
                          raw(source.mixed.back())),
          "Full accumulators, initially scaled right history and final unused prefetch differ");
  constexpr unsigned status_mask = down | up | rounding;
  require((state.sr & status_mask) == (input.scale | (input.twos ? rounding : 0)) &&
              (state.sr & limit_flag) == (expected.clipped != 0 ? limit_flag : 0),
          "Caller scaling/rounding and sticky limiting differ");
  auto &bus = machine_.bus();
  for (unsigned i = 0; i < output_.size(); ++i) {
    const bool active = i >= input.phase && i < input.phase + expected.output.size();
    const auto word = active ? raw(expected.output.at(i - input.phase)) : output_.at(i);
    require(get(bus, DspSpace::x, output_base + i) == word,
            "Both output halves preserve their exact ownership");
  }
  for (unsigned lane = 0; lane < 2; ++lane) {
    require(get(bus, DspSpace::x, history + lane) == raw(expected.history.at(lane)) &&
                get(bus, DspSpace::y, history + lane) == raw(expected.history.at(lane + 2)),
            "Final input/output histories differ from independent lane state");
  }
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto address : guards) {
      require(get(bus, space, address) == guard, "Native output changed a guard");
    }
  }
  verify_accesses(input, source, expected);
}

void Fixture::restore(const Saved &saved) {
  machine_.restore(saved.machine);
  effects_ = saved.effects;
  ++restores_;
}

void Fixture::run(const Input &input,
                  const Samples &source,
                  const Prediction &expected,
                  const bool replay) {
  try {
    seed(input, source);
    effects_ = {};
    middle_.reset();
    execute(replay);
    verify(input, source, expected);
    const auto effects = effects_;
    const auto endpoint = machine_.cpu().debug_state();
    if (replay) {
      if (!middle_) { throw Fault("Selected original output calls require a mid-DO checkpoint"); }
      const auto saved = *middle_;
      for (unsigned branch = 0; branch < 2; ++branch) {
        restore(saved);
        execute(false);
        verify(input, source, expected);
        require(
            effects_ == effects && machine_.cpu().debug_state() == endpoint,
            "Both native mid-DO restores reproduce all accesses, output words and full CPU state");
      }
    }
    for (unsigned i = 0; i < expected.output.size(); ++i) {
      output_.at(input.phase + i) = raw(expected.output.at(i));
    }
    ++calls_;
  } catch (const Fault &error) {
    throw Fault("Original master volume=" + std::to_string(input.volume) +
                " scale=" + std::to_string(input.scale) + " phase=" + std::to_string(input.phase) +
                " pattern=" + std::to_string(input.pattern) + ": " + error.what());
  }
}

void Fixture::intact() {
  for (unsigned i = 0; i < program_.size(); ++i) {
    require(get(machine_.bus(), DspSpace::p, program_begin + i) == program_.at(i),
            "Original output program must remain unchanged");
  }
}

void Fixture::failures() {
  if (!baseline_) {
    throw Fault("Original output fixture must retain its untouched native baseline");
  }
  const auto baseline = *baseline_;
  constexpr std::array<unsigned, 5> addresses{
      output_base, input_base + 32, gain_slot, history + 1, history + 1};
  for (unsigned missing = 1; missing <= addresses.size(); ++missing) {
    machine_.restore(baseline);
    ++restores_;
    const Input input{.volume = 127, .phase = 32, .pattern = 2};
    seed(input, samples(input.pattern), missing);
    effects_ = {};
    bool rejected = false;
    try {
      execute(false);
    } catch (const Fault &error) {
      const std::string_view text(error.what());
      const char *const space = missing == 1 || missing == 5 ? "Y:" : "X:";
      require(text.contains(std::string(space) + hex(addresses.at(missing - 1), 6)) &&
                  text.contains("pc="),
              "Missing original data must retain its actual address, space and native PC");
      rejected = true;
    }
    require(rejected,
            "Unused reads and histories cannot be replaced by successful peripheral/data stubs");
    bool quarantined = false;
    try {
      machine_.cpu().step();
    } catch (const Fault &) { quarantined = true; }
    require(quarantined, "A failed original access cannot silently resume");
    ++failed_reads_;
    intact();
  }
  machine_.restore(baseline);
  ++restores_;
}
}
