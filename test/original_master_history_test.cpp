// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "firmware/firmware.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
using Word = dsp56300::Word;
constexpr unsigned word_bits = 24, word_mask = 0xffffff;
constexpr std::int64_t half = std::int64_t{1} << (word_bits - 1);
constexpr std::int64_t feedback = 0x7bc01a;
constexpr std::int64_t fixed_minimum = -((half - 1) / (half - feedback));
constexpr std::int64_t expected_fixed_minimum = -30;
constexpr unsigned scaling_mask = 0xc00;
constexpr unsigned record_index = 44, program_begin = 0x11d, program_words = 793;
constexpr unsigned entry = 0x27b, coefficient_address = 0x285;
constexpr unsigned gain = 0x17, history = 0x46, input = 0x620, output = 0x660;
constexpr unsigned block_words = 32, stop = 6;
constexpr unsigned original_instructions = 14 + (16 * 8) + 3;
constexpr unsigned caller_instructions = 3,
                   block_instructions = original_instructions + caller_instructions;
constexpr unsigned settle_updates = 389, settle_blocks = 25;
constexpr unsigned move_r0 = 0x60f400, move_r1 = 0x61f400;
constexpr unsigned jsr_absolute = 0x0bf080, jmp_absolute = 0x0af080;
// Authored FM MOVE-immediate and absolute JSR/JMP encodings. The routine itself
// comes exclusively from the local hash-pinned original firmware.
constexpr std::array caller{move_r0, input, move_r1, output, jsr_absolute, entry, jmp_absolute, 0U};
constexpr std::array volumes{0U, 1U, 32U, 64U, 96U, 127U};

void require(const bool accepted, const std::string_view reason) {
  if (!accepted) { throw std::runtime_error(std::string(reason)); }
}

Word raw(const std::int64_t value) {
  return static_cast<Word>(static_cast<std::uint64_t>(value) & word_mask);
}

std::int64_t next(const std::int64_t value) {
  const auto product = feedback * value;
  // Mathematical floor, independently of host signed-shift and DSP helpers.
  return (product / half) - static_cast<std::int64_t>(product < 0 && product % half != 0);
}

void prove_bound() {
  require(fixed_minimum == expected_fixed_minimum,
          "The integer inequality determines all negative fixed words");
  for (std::int64_t value = fixed_minimum; value <= 0; ++value) {
    require(next(value) == value, "Every derived nonpositive fixed word remains exact");
  }
  std::int64_t lower = -half;
  std::int64_t upper = half - 1;
  for (unsigned index = 1; index <= settle_updates; ++index) {
    lower = next(lower);
    upper = next(upper);
    if (index + 1 == settle_updates) {
      require(lower == expected_fixed_minimum - 1 && upper == 1,
              "Both extremes witness the sharp predecessor bound");
    }
  }
  require(lower == fixed_minimum && upper == 0,
          "Monotonic endpoint images bound every signed24 initial history after389 updates");
}

class Fixture {
public:
  explicit Fixture(const blofeld::Firmware &firmware)
      : cpu_(dsp56300::Buffers{.program = program_, .x = x_, .y = y_}) {
    require(firmware.dsp.size() > record_index, "Original master program record must exist");
    const auto &record = firmware.dsp[record_index];
    require(record.space == 0 && record.address == program_begin &&
                record.words.size() == program_words,
            "The pinned original program has its actual Harvard space, address and extent");
    cpu_.load(dsp56300::Space::program, program_begin, record.words);
    cpu_.load(dsp56300::Space::program, 0, caller);
    require(program_.at(coefficient_address) == static_cast<Word>(feedback),
            "The documented feedback coefficient matches the unchanged original operand");
    original_ = program_;
  }

  void start(const unsigned volume, const std::array<std::int64_t, 2> initial) {
    cpu_.reset();
    x_.fill(0);
    y_.fill(0);
    x_.at(gain) = 512 * volume * volume;
    for (unsigned lane = 0; lane < initial.size(); ++lane) {
      x_.at(history + lane) = raw(initial[lane]);
    }
    auto state = cpu_.registers();
    state.pc = 0;
    state.sr = 0; // Ordinary unscaled word transfers; no rounding MAC occurs.
    state.m.fill(word_mask);
    cpu_.set_registers(state);
  }

  std::array<std::int64_t, 2> block(std::array<std::int64_t, 2> expected) {
    const auto before = cpu_.progress().instructions;
    const auto result = cpu_.run(block_instructions);
    require(result.fault.kind == dsp56300::FaultKind::none &&
                result.dispatches == block_instructions,
            "Actual original instructions must retire without a synthesized continuation");
    const auto state = cpu_.registers();
    require(state.pc == stop && state.sp == 0 && (state.sr & scaling_mask) == 0,
            "Actual JSR/RTS and unscaled transfers retain the declared caller context");
    require(
        cpu_.progress().instructions - before == block_instructions,
        "Each call executes the original145 instructions and three authored caller instructions");
    for (unsigned index = 0; index < block_words; ++index) {
      const auto lane = index % 2;
      expected[lane] = next(expected[lane]);
      require(x_.at(output + index) == raw(expected[lane]),
              "Every original output word follows the independent zero-input recurrence");
    }
    for (unsigned lane = 0; lane < expected.size(); ++lane) {
      require(x_.at(history + lane) == raw(expected[lane]) && y_.at(history + lane) == 0,
              "Actual output histories persist and actual zero-input histories remain zero");
    }
    require(std::all_of(x_.begin() + input,
                        x_.begin() + input + block_words + 1,
                        [](const Word value) { return value == 0; }),
            "The complete input block and genuine final prefetch neighbor remain zero");
    require(program_ == original_, "No original program or authored caller word may change");
    const auto returned = cpu_.run(1);
    require(returned.fault.kind == dsp56300::FaultKind::none && returned.dispatches == 1 &&
                cpu_.registers().pc == 0 && cpu_.registers().sp == 0,
            "The authored JMP, not a host PC repair, starts the next call");
    ++calls_;
    return expected;
  }

  [[nodiscard]] unsigned calls() const { return calls_; }

private:
  std::array<Word, 4096> program_{};
  std::array<Word, 4096> original_{};
  std::array<Word, 2048> x_{};
  std::array<Word, 2048> y_{};
  dsp56300::Core cpu_;
  unsigned calls_ = 0;
};

void run(const blofeld::Firmware &firmware) {
  prove_bound();
  Fixture fixture(firmware);
  for (const auto volume : volumes) {
    for (std::int64_t value = fixed_minimum; value <= 0; ++value) {
      const std::array initial{value, fixed_minimum - value};
      fixture.start(volume, initial);
      require(fixture.block(initial) == initial,
              "All31 fixed histories survive in both lanes at every selected master gain");
    }
    std::array extreme{-half, half - 1};
    fixture.start(volume, extreme);
    for (unsigned block = 0; block < settle_blocks; ++block) { extreme = fixture.block(extreme); }
    require(
        extreme == std::array{fixed_minimum, std::int64_t{0}},
        "Original repeated calls converge from both extreme histories without host history stores");
  }
  constexpr unsigned expected_calls = static_cast<unsigned>(volumes.size()) * (31 + settle_blocks);
  require(fixture.calls() == expected_calls,
          "Complete fixed-band and retained extreme-history matrix");
  std::cout << "MASTER_HISTORY calls=" << fixture.calls()
            << " output-words=" << fixture.calls() * block_words
            << " fixed-words=31 endpoint-updates=389 gains=" << volumes.size()
            << " original-instructions=" << fixture.calls() * original_instructions << '\n';
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Usage: original-master-history firmware.zip");
    run(blofeld::Firmware::load(arguments[1]));
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
