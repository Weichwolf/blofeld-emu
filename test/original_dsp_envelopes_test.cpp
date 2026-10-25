// SPDX-License-Identifier: GPL-3.0-only
// Independent scalar envelope models; only pinned original firmware executes.
// Models adapted from Weichwolf/blofeld-src (GPL-3.0-only); provenance is recorded
// in doc/dsp-envelope-reference.json. No manufacturer program words are embedded.
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
constexpr unsigned memory_mode = 0x600389;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned descriptor = 0x480;
constexpr unsigned descriptor_pointer = 0x481;
constexpr unsigned setup_entry = 0xea5;
constexpr unsigned setup_stop = 0xeb7;
constexpr unsigned caller_entry = 0x7cb;
constexpr unsigned entries_base = 0x436;
constexpr unsigned setup_base = 0xdab;
constexpr unsigned retire_entry = 0x1609;
constexpr unsigned idle_entry = 0x160e;
constexpr unsigned release_entry = 0x160f;
constexpr unsigned loop_release_entry = 0x161c;
constexpr unsigned decay1_entry = 0x1629;
constexpr unsigned adsr_decay_entry = 0x1639;
constexpr unsigned decay2_entry = 0x1646;
constexpr unsigned advance_decay_entry = 0x164c;
constexpr unsigned adsr_attack_entry = 0x165e;
constexpr unsigned multistage_attack_entry = 0x166d;
constexpr unsigned start_entry = 0x167b;
constexpr unsigned refresh_entry = 0x1687;
constexpr unsigned guard_value = 0x13579b;
constexpr unsigned initial_value = 0x123456;
constexpr unsigned initial_peak = 0x654321;
constexpr unsigned initial_coefficient = 0x234567;
constexpr unsigned initial_sustain = 0x765432;
constexpr unsigned prefetch_value = 0xabcdef;
constexpr unsigned invalid_pointer = 0x12345;
constexpr unsigned positive_limit = 0x7fffff;
constexpr unsigned level127 = 0x7f0000;
constexpr unsigned level63 = 0x3f0000;
constexpr unsigned fraction_mask = 65535;
constexpr unsigned parameter_max = 127;
constexpr unsigned first_dispatch = 0x7d4;
constexpr unsigned second_dispatch = 0x7d9;
constexpr unsigned third_dispatch = 0x7de;
constexpr unsigned fourth_dispatch = 0x7e4;
constexpr unsigned program_base = retire_entry;
constexpr unsigned program_words = 148;
constexpr unsigned table_base = 0x1640;
constexpr unsigned table_words = 266;
constexpr unsigned curve_anchor = 0x16c8;
constexpr unsigned slot_base = 0x400;
constexpr unsigned slot_words = 6;
constexpr unsigned slots = 4;
constexpr unsigned controls = 0x440;
constexpr unsigned parameters = 0x600;
constexpr unsigned first_mode = parameters + 196;
constexpr unsigned parameter_stride = 12;
constexpr unsigned mask = 0xffffff;
constexpr unsigned flags_initial = 0x900001;
constexpr std::int64_t unit = 1LL << 24;
constexpr std::int64_t half = 1LL << 23;
constexpr unsigned step_limit = 256;
constexpr unsigned caller_controls = 0x26;
constexpr unsigned word_bits = 24;
constexpr unsigned level_stride = 31;
constexpr unsigned byte_mask = 255;
constexpr unsigned current_descriptor = 0x14;
constexpr unsigned state_words = 24;
constexpr unsigned parameter_words = 48;
constexpr std::array<unsigned, 8> value_corners{
    0, 1, 0x10000, 0x3fffff, 0x7effff, positive_limit, 0x800000, 0xffffff};
constexpr std::array<unsigned, 3> rate_fractions{0, 1, fraction_mask};
constexpr std::array<unsigned, 8> arithmetic_targets{release_entry,
                                                     loop_release_entry,
                                                     decay1_entry,
                                                     adsr_decay_entry,
                                                     decay2_entry,
                                                     advance_decay_entry,
                                                     adsr_attack_entry,
                                                     multistage_attack_entry};

void require(bool ok, const std::string &message) {
  if (!ok) { throw blofeld::Fault(message); }
}

std::int64_t signed_word(unsigned value) {
  return value < half ? value : static_cast<std::int64_t>(value) - unit;
}

std::int64_t floor_div(std::int64_t value, std::int64_t divisor) {
  return value >= 0 ? value / divisor : -((-value + divisor - 1) / divisor);
}

unsigned raw_word(std::int64_t value) {
  return static_cast<unsigned>(static_cast<std::uint64_t>(value) & mask);
}

unsigned stored(std::int64_t full) {
  return raw_word(std::clamp(floor_div(full, unit), -half, half - 1));
}

std::int64_t magnitude(std::int64_t value) {
  return value < 0 ? -value : value;
}

struct Source {
  std::vector<unsigned> program;
  std::vector<unsigned> table;
  std::vector<unsigned> entries;
  std::vector<unsigned> setup;
  bool chain = false;
  bool setup_window = false;
};

void harness(blofeld::DspBus &bus, const Source &source, unsigned &stop) {
  blofeld::test::DspProgram code;
  code.immediate("x1", memory_mode);
  code.emit("move x1,omr");
  for (unsigned i = 0; i < 3; ++i) { code.emit("nop"); }
  const auto begin = code.pc();
  code.immediate("sr", 0);
  for (unsigned i = 0; i < 8; ++i) { code.immediate("m" + std::to_string(i), mask); }
  if (source.setup_window) {
    code.immediate("r0", parameters);
    code.immediate("r2", descriptor);
    code.word(absolute_jump); // DSP56300FM long absolute JMP, followed by its target.
    code.word(setup_entry);
    stop = setup_stop; // Observe the original prefix before chain construction.
  } else {
    if (source.chain) {
      code.immediate("r5", descriptor_pointer);
      code.immediate("r7", caller_entry);
    } else {
      code.immediate("r0", slot_base + 1);
      code.immediate("r3", slot_base);
      code.immediate("r6", controls);
      code.immediate("n0", 3);
      code.immediate("x1", curve_anchor);
      code.emit("tfr x1,a");
      code.emit("move x:>" + blofeld::test::number(slot_base) + ",r7");
    }
    code.emit("jsr (r7)");
    stop = code.pc();
    code.emit("nop");
    code.word(absolute_jump); // Own loop; no original callback or return is patched.
    code.word(begin);
  }
  code.install(bus);
}

blofeld::DspBus &prepare(blofeld::DspBus &bus, const Source &source, unsigned &stop) {
  harness(bus, source, stop);
  bus.observe_omr(memory_mode);
  bus.advance_instruction_cycles(3);
  for (unsigned i = 0; i < source.program.size(); ++i) {
    bus.write({.address = program_base + i, .value = source.program[i]});
  }
  for (unsigned i = 0; i < source.entries.size(); ++i) {
    bus.write({.address = entries_base + i, .value = source.entries[i]});
  }
  for (unsigned i = 0; i < source.setup.size(); ++i) {
    bus.write({.address = setup_base + i, .value = source.setup[i]});
  }
  for (unsigned i = 0; i < source.table.size(); ++i) {
    bus.write({.space = blofeld::DspSpace::y, .address = table_base + i, .value = source.table[i]});
  }
  return bus;
}

struct Fixture {
  const Source &source;
  blofeld::DspBus bus;
  unsigned stop = 0;
  blofeld::DspCore cpu;
  unsigned long long cases = 0;
  unsigned long long steps = 0;
  std::set<unsigned> visited;

  explicit Fixture(const Source &input) : source(input), cpu(prepare(bus, input, stop)) {}

  void x(unsigned address, unsigned value) {
    bus.write({.space = blofeld::DspSpace::x, .address = address, .value = value});
  }

  unsigned x(unsigned address) {
    return bus.read({.space = blofeld::DspSpace::x, .address = address});
  }

  void y(unsigned address, unsigned value) {
    bus.write({.space = blofeld::DspSpace::y, .address = address, .value = value});
  }

  void run(unsigned target) {
    if (!source.chain && !source.setup_window) {
      require(x(slot_base) == target, "Actual callback differs from expected entry");
    }
    unsigned completed = 0;
    unsigned dispatches = 0;
    bool entered = false;
    while (cpu.state().pc != stop || completed == 0) {
      require(completed < step_limit, "Envelope call instruction bound");
      const auto state = cpu.state();
      if (state.pc >= program_base && state.pc < program_base + program_words) {
        visited.insert(state.pc);
      }
      if (state.pc == target) {
        require(state.sp == (source.setup_window ? 0U : 1U) &&
                    (source.setup_window || state.stack_pc == stop),
                "Original entry retains the genuine own caller frame");
        entered = true;
      }
      if (source.chain && (state.pc == first_dispatch || state.pc == second_dispatch ||
                           state.pc == third_dispatch || state.pc == fourth_dispatch)) {
        require(dispatches < slots, "Extra envelope dispatch");
        require(state.sp == 1 && state.stack_pc == stop &&
                    state.r[0] == slot_base + dispatches * slot_words + 1 &&
                    state.r[6] == caller_controls + dispatches * 4 &&
                    state.r[7] == x(slot_base + (dispatches * slot_words)) && state.n[0] == 3 &&
                    state.n[4] == slot_words &&
                    state.a == static_cast<std::uint64_t>(curve_anchor) << word_bits,
                "Original envelope caller frame differs from derived convention");
        ++dispatches;
      }
      cpu.step();
      bus.advance_instruction_cycles(1);
      ++completed;
    }
    require(entered && cpu.state().sp == 0,
            "Missing original entry or declared return/prefix boundary");
    require(!source.chain || dispatches == slots, "Missing actual envelope dispatch");
    ++cases;
    steps += completed;
  }
};

void intact(Fixture &fixture) {
  const auto &source = fixture.source;
  for (unsigned i = 0; i < source.table.size(); ++i) {
    require(fixture.bus.read({.space = blofeld::DspSpace::y, .address = table_base + i}) ==
                source.table[i],
            "Original envelope table changed");
  }
  for (const auto &[base, words] : std::array{std::pair{program_base, &source.program},
                                              std::pair{entries_base, &source.entries},
                                              std::pair{setup_base, &source.setup}}) {
    for (unsigned i = 0; i < words->size(); ++i) {
      require(fixture.bus.peek_program(base + i) == words->at(i),
              "Original program record changed");
    }
  }
}

using State = std::array<unsigned, state_words>;
using Parameters = std::array<unsigned, parameter_words>;
using Control = std::array<unsigned, 4>;

State initial_state(unsigned callback) {
  State state{};
  for (unsigned i = 0; i < slots; ++i) {
    const unsigned offset = i * slot_words;
    state[offset] = i == 0 ? callback : idle_entry;
    state[offset + 1] = initial_value + i;
    state[offset + 2] = first_mode + i * parameter_stride;
    state[offset + 3] = initial_peak;
    state[offset + 4] = initial_coefficient;
    state[offset + 5] = initial_sustain;
  }
  return state;
}

Parameters initial_parameters(unsigned mode = 0) {
  Parameters result{};
  for (unsigned i = 0; i < slots; ++i) {
    const unsigned offset = i * parameter_stride;
    for (unsigned j = 0; j < parameter_stride; ++j) {
      result[offset + j] = (13 * i + 7 * j) & parameter_max;
    }
    result[offset] = mode;
  }
  return result;
}

void seed(Fixture &fixture, const State &state, const Parameters &params, const Control &control) {
  for (unsigned i = 0; i < state.size(); ++i) { fixture.x(slot_base + i, state[i]); }
  for (unsigned i = 0; i < params.size(); ++i) { fixture.x(first_mode + i, params[i]); }
  for (unsigned i = 0; i < control.size(); ++i) { fixture.y(controls + i, control[i]); }
  for (const unsigned address : {slot_base - 1,
                                 slot_base + (slots * slot_words),
                                 first_mode - 1,
                                 first_mode + (slots * parameter_stride)}) {
    fixture.x(address, guard_value);
  }
  fixture.x(6, flags_initial);
}

void check(Fixture &fixture,
           const State &expected,
           const Parameters &params,
           const Control &control,
           unsigned flags) {
  for (unsigned i = 0; i < expected.size(); ++i) {
    require(fixture.x(slot_base + i) == expected[i],
            "State word " + std::to_string(i) + ": got " +
                std::to_string(fixture.x(slot_base + i)) + ", expected " +
                std::to_string(expected[i]));
  }
  for (unsigned i = 0; i < params.size(); ++i) {
    require(fixture.x(first_mode + i) == params[i], "Parameter changed");
  }
  for (unsigned i = 0; i < control.size(); ++i) {
    require(fixture.bus.read({.space = blofeld::DspSpace::y, .address = controls + i}) ==
                control[i],
            "Control changed");
  }
  for (const unsigned address : {slot_base - 1,
                                 slot_base + (slots * slot_words),
                                 first_mode - 1,
                                 first_mode + (slots * parameter_stride)}) {
    require(fixture.x(address) == guard_value, "State/parameter guard changed");
  }
  require(fixture.x(6) == flags, "Collection flag changed unexpectedly");
}

unsigned coefficient(const Source &source, unsigned raw, bool attack) {
  const auto delta =
      attack ? floor_div(signed_word(raw), 65536) : floor_div(-signed_word(raw), 65536);
  const auto address =
      static_cast<std::int64_t>(curve_anchor) +
      (attack ? std::max<std::int64_t>(delta, 0) : std::min<std::int64_t>(delta, 0));
  require(address >= table_base &&
              address < table_base + static_cast<std::int64_t>(source.table.size()),
          "Lookup escaped declared source table");
  return source.table[static_cast<unsigned>(address) - table_base];
}

struct RefreshInput {
  unsigned corner;
  unsigned field;
  unsigned value;
};

void initialize_case(Fixture &fixture, const RefreshInput input) {
  const auto [corner, field, value] = input;
  auto params = initial_parameters();
  auto state = initial_state(refresh_entry);
  auto expected = state;
  for (unsigned i = 0; i < slots; ++i) {
    const unsigned p = i * parameter_stride;
    std::array<unsigned, 3> values{(corner & 1) != 0 ? parameter_max : 0U,
                                   (corner & 2) != 0 ? parameter_max : 0U,
                                   (corner & 4) != 0 ? parameter_max : 0U};
    values[field] = (value + i * level_stride) & parameter_max;
    params[p + 4] = values[0];
    params[p + 7] = values[1];
    params[p + 8] = values[2];
    const unsigned s = i * slot_words;
    expected[s + 3] = values[0] * 65536;
    expected[s + 4] = fixture.source.table[curve_anchor - values[1] - table_base];
    expected[s + 5] = values[2] * 65536;
  }
  constexpr Control control{0, 1, 2, 3};
  seed(fixture, state, params, control);
  fixture.run(refresh_entry);
  check(fixture, expected, params, control, flags_initial);
  const auto cpu = fixture.cpu.state();
  require(cpu.r[1] == first_mode + slots * parameter_stride &&
              cpu.r[3] == slot_base + 3 + slots * slot_words && cpu.n[1] == 4 && cpu.n[3] == 4,
          "Initializer cursor/stride mismatch");
}

void initialize_cases(Fixture &fixture) {
  const auto before = fixture.cases;
  const auto before_steps = fixture.steps;
  for (unsigned corner = 0; corner < 8; ++corner) {
    for (unsigned field = 0; field < 3; ++field) {
      for (unsigned value = 0; value < 128; ++value) {
        initialize_case(fixture, {.corner = corner, .field = field, .value = value});
      }
    }
  }
  std::cout << "Envelope initializer: " << fixture.cases - before << " calls, "
            << fixture.steps - before_steps
            << " instructions; all four six-word slots, 0..127 field sweeps and eight "
               "corners\n";
}

struct Result {
  State state;
  unsigned cursor = slot_base + 1;
  unsigned control_cursor = controls;
  unsigned flags = flags_initial;
};

Result model(const Source &source,
             State state,
             const Parameters &params,
             const Control &control,
             unsigned flags = flags_initial) {
  Result result{.state = state, .flags = flags};
  auto &next = result.state;
  auto target = state[0];
  const auto old = signed_word(state[1]);
  if (target == start_entry) {
    target = (params[0] & 7) == 0 ? adsr_attack_entry : multistage_attack_entry;
    next[0] = target;
  }
  switch (target) {
    case retire_entry:
      next[0] = idle_entry;
      next[1] = 0;
      result.cursor = slot_base;
      result.flags |= 2;
      break;
    case idle_entry: break;
    case release_entry:
    case loop_release_entry: {
      const auto c = signed_word(coefficient(source, control[3], false));
      const auto full = old * (unit - 2 * c);
      next[1] = stored(full);
      if (full <= 0) { next[0] = target == release_entry ? retire_entry : multistage_attack_entry; }
      break;
    }
    case decay1_entry:
    case adsr_decay_entry: {
      const auto c = signed_word(coefficient(source, control[1], false));
      const auto sustain = signed_word(control[2]);
      const auto full = (old * unit) + (2 * c * (sustain - old));
      next[1] = stored(full);
      if (target == decay1_entry &&
          magnitude(full - (sustain * unit)) < advance_decay_entry * unit) {
        next[0] = advance_decay_entry;
      }
      result.cursor = slot_base;
      result.control_cursor = controls + 3;
      break;
    }
    case decay2_entry:
    case advance_decay_entry: {
      const auto c = signed_word(state[4]);
      const auto sustain = signed_word(state[5]);
      const auto full = (old * unit) + (2 * c * (sustain - old));
      next[1] = stored(full);
      if (target == advance_decay_entry && magnitude(full - (sustain * unit)) < state[0] * unit) {
        next[0] = source.table[params[0] & 7];
        result.cursor = slot_base;
      }
      break;
    }
    case adsr_attack_entry: {
      const auto increment = signed_word(coefficient(source, control[0], true));
      const auto full = (old + increment) * unit;
      next[1] = stored(full);
      if (full < -half * unit || full >= half * unit) {
        next[0] = adsr_decay_entry;
        result.cursor = slot_base;
      }
      result.control_cursor = controls + 1;
      break;
    }
    case multistage_attack_entry: {
      const auto increment = signed_word(coefficient(source, control[0], true));
      const auto peak = signed_word(state[3]);
      const auto full = old + increment;
      next[1] = stored(std::min(full, peak) * unit);
      if (full >= peak) { next[0] = decay1_entry; }
      result.cursor = slot_base;
      break;
    }
    default: throw blofeld::Fault("Unsupported independent model entry");
  }
  return result;
}

void dynamic_case(Fixture &fixture, State state, const Parameters &params, const Control &control) {
  const unsigned target = state[0];
  const auto expected = model(fixture.source, state, params, control);
  seed(fixture, state, params, control);
  fixture.run(target);
  try {
    check(fixture, expected.state, params, control, expected.flags);
    const auto cpu = fixture.cpu.state();
    require(cpu.r[0] == expected.cursor && cpu.r[6] == expected.control_cursor,
            "Callback cursor mismatch: R0=" + std::to_string(cpu.r[0]) +
                ", R6=" + std::to_string(cpu.r[6]));
    require(cpu.a == static_cast<std::uint64_t>(curve_anchor) << word_bits,
            "Callback did not retain/reset the curve anchor in A");
  } catch (const std::exception &error) {
    throw blofeld::Fault("Entry " + std::to_string(target) + ", value " + std::to_string(state[1]) +
                         ", control " + std::to_string(control[0]) + ": " + error.what());
  }
}

void rate_cases(Fixture &fixture) {
  for (const unsigned callback : arithmetic_targets) {
    for (unsigned rate = 0; rate < 128; ++rate) {
      for (const unsigned fraction : rate_fractions) {
        for (const unsigned value : value_corners) {
          auto state = initial_state(callback);
          auto params = initial_parameters(rate & 7);
          state[1] = value;
          state[3] = (rate & 1) != 0 ? level127 : 0x400000;
          state[4] = coefficient(fixture.source, rate * 65536, false);
          state[5] = (rate & 1) != 0 ? 0 : level63;
          const unsigned raw = (rate * 65536) + fraction;
          const Control control{raw, raw, (rate & 1) != 0 ? level127 : 0U, raw};
          dynamic_case(fixture, state, params, control);
        }
      }
    }
  }
}

void mode_cases(Fixture &fixture) {
  for (unsigned mode = 0; mode < 256; ++mode) {
    for (const unsigned value : value_corners) {
      auto state = initial_state(start_entry);
      state[1] = value;
      state[3] = level127;
      const auto params = initial_parameters(mode);
      dynamic_case(fixture, state, params, {0, level127, level63, 0});
    }
  }
}

void signed_cases(Fixture &fixture) {
  for (const unsigned callback : arithmetic_targets) {
    for (const unsigned raw : {0xffffffU, 0xff0000U, 0x800000U}) {
      for (const unsigned value : value_corners) {
        auto state = initial_state(callback);
        state[1] = value;
        state[3] = level127;
        dynamic_case(fixture, state, initial_parameters(), {raw, raw, 0, raw});
      }
    }
  }
}

void retirement_cases(Fixture &fixture) {
  for (const unsigned callback : {retire_entry, idle_entry}) {
    for (const unsigned value : value_corners) {
      auto state = initial_state(callback);
      state[1] = value;
      dynamic_case(fixture, state, initial_parameters(), {0, 0, 0, 0});
    }
  }
}

void threshold_cases(Fixture &fixture) {
  // Strict magnitude comparisons at, below and above the original callback
  // threshold; both signed sides, including fractional values before stores.
  for (unsigned mode = 0; mode < 256; ++mode) {
    for (const unsigned callback : {decay1_entry, advance_decay_entry}) {
      for (const unsigned distance : {0U, 0x164bU, advance_decay_entry, 0x164dU}) {
        for (const int sign : {-1, 1}) {
          for (const unsigned c : {0U, 50U, positive_limit}) {
            auto state = initial_state(callback);
            state[1] = raw_word(static_cast<std::int64_t>(sign) * distance);
            state[4] = c;
            state[5] = 0;
            const auto params = initial_parameters(mode);
            dynamic_case(fixture, state, params, {0, positive_limit, 0, 0});
          }
        }
      }
    }
  }
}

void dynamics(Fixture &fixture) {
  const auto before = fixture.cases;
  const auto before_steps = fixture.steps;
  rate_cases(fixture);
  mode_cases(fixture);
  signed_cases(fixture);
  retirement_cases(fixture);
  threshold_cases(fixture);
  std::cout << "Envelope callbacks: " << fixture.cases - before << " calls, "
            << fixture.steps - before_steps
            << " instructions; eight arithmetic entries/all128 bins/fractions, 256 modes, signed "
               "controls and retirement\n";
}

void sequences(Fixture &fixture) {
  const auto before = fixture.cases;
  const auto before_steps = fixture.steps;
  constexpr unsigned calls = 16;
  for (unsigned mode = 0; mode < 256; ++mode) {
    for (const unsigned decay : {0U, 64U, parameter_max}) {
      auto state = initial_state(start_entry);
      state[1] = 0;
      state[3] = level127;
      state[4] = coefficient(fixture.source, decay * 65536, false);
      state[5] = 0;
      const auto params = initial_parameters(mode);
      const Control control{0, 0, level63, 0};
      unsigned flags = flags_initial;
      seed(fixture, state, params, control);
      for (unsigned call = 0; call < calls; ++call) {
        const auto expected = model(fixture.source, state, params, control, flags);
        fixture.run(state[0]);
        check(fixture, expected.state, params, control, expected.flags);
        require(fixture.cpu.state().r[0] == expected.cursor &&
                    fixture.cpu.state().r[6] == expected.control_cursor,
                "Successive callback cursor mismatch");
        state = expected.state;
        flags = expected.flags;
      }
    }
  }
  std::cout << "Envelope sequences: " << fixture.cases - before << " calls, "
            << fixture.steps - before_steps
            << " instructions; 768 sixteen-call histories through actual mutable callbacks\n";
}

void check_chain_controls(Fixture &fixture, const std::array<Control, slots> &control) {
  constexpr auto y_base = caller_controls;
  for (unsigned slot = 0; slot < slots; ++slot) {
    for (unsigned i = 0; i < control[slot].size(); ++i) {
      require(fixture.bus.read({.space = blofeld::DspSpace::y,
                                .address = y_base + (slot * 4) + i}) == control[slot][i],
              "Caller control word changed");
    }
  }
  for (const unsigned address : {y_base - 1, y_base + (slots * 4)}) {
    require(fixture.bus.read({.space = blofeld::DspSpace::y, .address = address}) == guard_value,
            "Caller control guard changed");
  }
}

void chains(Fixture &fixture) {
  constexpr auto y_base = caller_controls;
  constexpr std::array<unsigned, 11> targets{retire_entry,
                                             idle_entry,
                                             release_entry,
                                             loop_release_entry,
                                             decay1_entry,
                                             adsr_decay_entry,
                                             decay2_entry,
                                             advance_decay_entry,
                                             adsr_attack_entry,
                                             multistage_attack_entry,
                                             start_entry};
  constexpr std::array<unsigned, 8> values{
      0, 1, 0x10000, 0x3fffff, 0x7effff, positive_limit, 0x800000, 0xffffff};
  constexpr std::array<unsigned, 3> fractions{0, 1, fraction_mask};
  for (unsigned rate = 0; rate < 128; ++rate) {
    for (unsigned pattern = 0; pattern < values.size(); ++pattern) {
      auto state = initial_state(start_entry);
      auto params = initial_parameters();
      std::array<Control, slots> control{};
      for (unsigned slot = 0; slot < slots; ++slot) {
        const unsigned s = slot * slot_words;
        const unsigned p = slot * parameter_stride;
        state[s] = targets[(rate + pattern + slot) % targets.size()];
        state[s + 1] = values[(pattern + slot) % values.size()];
        state[s + 3] = level127;
        state[s + 4] = coefficient(fixture.source, rate * 65536, false);
        state[s + 5] = (pattern & 1) == 0 ? 0U : level63;
        params[p] = (2 * rate + pattern + slot) & byte_mask;
        const unsigned raw = (rate * 65536) + fractions[(pattern + slot) % fractions.size()];
        control[slot] = {raw, raw, (pattern & 1) == 0 ? 0U : level127, raw};
      }
      seed(fixture, state, params, control[0]);
      fixture.x(current_descriptor, descriptor);
      fixture.x(descriptor + 1, slot_base);
      fixture.y(descriptor + 2, prefetch_value); // Original caller's unused Y prefetch.
      fixture.y(y_base - 1, guard_value);
      fixture.y(y_base + (slots * 4), guard_value);
      auto expected = state;
      unsigned flags = flags_initial;
      unsigned final_cursor = 0;
      unsigned final_control_cursor = 0;
      for (unsigned slot = 0; slot < slots; ++slot) {
        const unsigned s = slot * slot_words;
        State substate = state;
        std::copy_n(state.begin() + s, slot_words, substate.begin());
        Parameters subparams = params;
        std::copy_n(params.begin() + (static_cast<std::ptrdiff_t>(slot) * parameter_stride),
                    parameter_stride,
                    subparams.begin());
        const auto result = model(fixture.source, substate, subparams, control[slot], flags);
        std::copy_n(result.state.begin(), slot_words, expected.begin() + s);
        flags = result.flags;
        final_cursor = result.cursor + s;
        final_control_cursor = y_base + slot * 4 + result.control_cursor - controls;
        for (unsigned i = 0; i < control[slot].size(); ++i) {
          fixture.y(y_base + (slot * 4) + i, control[slot][i]);
        }
      }
      fixture.run(caller_entry);
      check(fixture, expected, params, control[0], flags);
      const auto cpu = fixture.cpu.state();
      require(cpu.r[0] == final_cursor && cpu.r[6] == final_control_cursor &&
                  cpu.r[4] == slot_base + 3 * slot_words && cpu.r[5] == descriptor,
              "Original four-envelope caller final cursor mismatch");
      require(fixture.x(current_descriptor) == descriptor && fixture.x(descriptor + 1) == slot_base,
              "Caller descriptor changed");
      require(fixture.bus.read({.space = blofeld::DspSpace::y, .address = descriptor + 2}) ==
                  prefetch_value,
              "Original caller prefetch changed");
      check_chain_controls(fixture, control);
    }
  }
  std::cout << "Envelope actual caller: " << fixture.cases << " complete prefixes, "
            << fixture.cases * slots << " real callbacks, " << fixture.steps
            << " instructions; all four dispatch frames and final tail return\n";
}

void setup_windows(const Source &source, std::ostream &trace) {
  unsigned long long steps = 0;
  unsigned calls = 0;
  for (unsigned scenario = 0; scenario < 32; ++scenario) {
    Fixture fixture(source);
    auto state = initial_state(idle_entry);
    auto expected = state;
    auto params = initial_parameters();
    for (unsigned slot = 0; slot < slots; ++slot) {
      const unsigned s = slot * slot_words;
      const unsigned p = slot * parameter_stride;
      const unsigned value = (4 * scenario) + slot;
      params[p + 4] = value;
      params[p + 7] = (7 * value) & parameter_max;
      params[p + 8] = (13 * value) & parameter_max;
      expected[s] = start_entry;
      expected[s + 1] = 0;
      expected[s + 2] = first_mode + p;
      expected[s + 3] = params[p + 4] * 65536;
      expected[s + 4] = source.table[curve_anchor - params[p + 7] - table_base];
      expected[s + 5] = params[p + 8] * 65536;
      // Deliberately wrong old pointers: the actual initializer must replace
      // all four, rather than succeeding on already prepared slot metadata.
      state[s + 2] = invalid_pointer;
    }
    constexpr Control control{0, 1, 2, 3};
    seed(fixture, state, params, control);
    fixture.x(descriptor_pointer, slot_base);
    fixture.run(setup_entry);
    check(fixture, expected, params, control, flags_initial);
    const auto cpu = fixture.cpu.state();
    require(cpu.r[0] == parameters && cpu.r[2] == descriptor &&
                cpu.r[1] == first_mode + slots * parameter_stride &&
                cpu.r[3] == slot_base + 3 + slots * slot_words,
            "Original initializer final cursor mismatch");
    require(fixture.x(descriptor_pointer) == slot_base, "Initializer descriptor changed");
    intact(fixture);
    steps += fixture.steps;
    calls += static_cast<unsigned>(fixture.cases);
  }
  trace << "setup " << calls << "\nsetup_instructions " << steps << '\n';
  std::cout << "Envelope setup window: 32 original initialization prefixes, " << steps
            << " instructions; four original pointer/clear writes and real coefficient call, stop "
               "before chain construction\n";
}

void group(Fixture &fixture,
           void (*run)(Fixture &),
           std::ostream &trace,
           const std::string_view name) {
  const auto before = fixture.cases;
  run(fixture);
  trace << name << ' ' << fixture.cases - before << '\n';
}

void missing_curve(Source source, std::ostream &trace) {
  source.table.resize(curve_anchor - table_base);
  Fixture fixture(source);
  auto state = initial_state(release_entry);
  state[1] = 1;
  seed(fixture, state, initial_parameters(), {0, 0, 0, 0});
  std::string error;
  try {
    fixture.run(release_entry);
  } catch (const blofeld::Fault &failure) { error = failure.what(); }
  require(error.contains("Unknown DSP memory read at Y:" + blofeld::hex(curve_anchor, 6)) &&
              error.contains("pc=") && error.contains("omr=" + blofeld::hex(memory_mode, 6)) &&
              fixture.cpu.state().sp == 1 && fixture.x(slot_base) == release_entry &&
              fixture.x(slot_base + 1) == 1 && fixture.cases == 0,
          "Missing original curve input fails with context before storing a result");
  bool failure_latched = false;
  try {
    fixture.cpu.step();
  } catch (const blofeld::Fault &) { failure_latched = true; }
  require(failure_latched, "A failed original callback cannot resume with guessed state");
  trace << "missing_curve 1\n";
  std::cout << "Original missing-curve boundary: " << error << '\n';
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Usage: original-dsp-envelopes-test firmware.zip private.trace");
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    const auto record = [&firmware](const unsigned space, const unsigned address) {
      const auto found = std::ranges::find_if(firmware.dsp, [space, address](const auto &value) {
        return value.space == space && value.address == address;
      });
      require(found != firmware.dsp.end(), "Missing original envelope record");
      return found->words;
    };
    const Source source{.program = record(0, program_base),
                        .table = record(2, table_base),
                        .entries = record(0, entries_base),
                        .setup = record(0, setup_base)};
    require(source.program.size() == program_words && source.table.size() == table_words,
            "Original envelope extent changed");
    Fixture fixture(source);
    std::ofstream trace(arguments[2]);
    require(trace.is_open(), "Cannot write private envelope trace");
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    group(fixture, initialize_cases, trace, "refresh");
    group(fixture, dynamics, trace, "callbacks");
    group(fixture, sequences, trace, "sequences");
    auto chain_source = source;
    chain_source.chain = true;
    Fixture chain_fixture(chain_source);
    chains(chain_fixture);
    auto setup_source = source;
    setup_source.setup_window = true;
    setup_windows(setup_source, trace);
    missing_curve(source, trace);
    fixture.visited.insert(chain_fixture.visited.begin(), chain_fixture.visited.end());
    intact(fixture);
    intact(chain_fixture);
    trace << "direct " << fixture.cases << "\nchain " << chain_fixture.cases << "\ninstructions "
          << fixture.steps + chain_fixture.steps << "\nvisited" << std::hex;
    for (const auto pc : fixture.visited) { trace << ' ' << pc; }
    trace << std::dec << '\n';
    require(trace.good(), "Cannot finish private envelope trace");
    std::cout << "Original envelope models checked; full MIDI, voice lifecycle and board audio "
                 "remain unproven.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
