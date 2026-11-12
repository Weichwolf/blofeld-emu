// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/clock.hpp"
#include "dsp/execute.hpp"
#include "dsp/program.hpp"
#include "io/dsp_reset.hpp"
#include "io/shi.hpp"
#include "isa.hpp"

#include <array>
#include <cstddef>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string_view>

namespace {
using namespace dsp56300;
namespace vi = blofeld::vi;
template <isa::Opcode instruction> constexpr Word op = static_cast<Word>(instruction);
constexpr Word boot = vi::dsp::BootRom::main_first;
constexpr Word mode = 0x600389, low_x_words = 0x3000, low_y_words = 0x2000;
constexpr Word high_first = 0xc000, high_x_words = 0x4000, high_y_words = 0xa000;
constexpr Word immediate = 0x34, absolute = 0x30, postincrement = 0x18;
constexpr Word read = 0x8000, x1 = 1U << 16U, x0_register = 4, omr_register = 26;
constexpr Word literal = op<isa::Opcode::move_x_effective> | (immediate << 8U) | read;
constexpr Word control = op<isa::Opcode::movec_effective> | (immediate << 8U) | read;
constexpr Word clock_read = op<isa::Opcode::move_x_effective> | (absolute << 8U) | read;
constexpr Word clock_write = op<isa::Opcode::move_x_effective> | (absolute << 8U);
constexpr std::array<Word, 15> offsets{0, 2, 4, 5, 6, 8, 9, 10, 11, 13, 14, 15, 17, 19, 20};
constexpr std::size_t producer_instructions = 13;
constexpr Word handoff = boot + offsets[producer_instructions];

struct Kernel {
  explicit Kernel(const Word x_count)
      : words{control | omr_register,
              mode,
              control,
              word_mask,
              op<isa::Opcode::move_immediate_x0> | x1,
              op<isa::Opcode::move_immediate_r0>,
              literal,
              x_count,
              op<isa::Opcode::rep_register> | (x0_register << 8U),
              op<isa::Opcode::move_x_effective> | x1 | (postincrement << 8U),
              op<isa::Opcode::move_immediate_r0>,
              literal,
              low_y_words,
              op<isa::Opcode::rep_register> | (x0_register << 8U),
              op<isa::Opcode::move_y_effective> | x1 | (postincrement << 8U),
              clock_read,
              vi::dsp::Clock::address,
              clock_write,
              vi::dsp::Clock::address,
              op<isa::Opcode::nop>,
              op<isa::Opcode::stop>} {}

  std::array<Word, offsets.back() + 1> words;
};

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

vi::dsp::Program prepare(const Kernel &kernel) {
  std::array<CodeSpan, offsets.size()> code;
  std::array<Word, offsets.size()> addresses{};
  for (std::size_t index = 0; index < offsets.size(); ++index) {
    const auto end = index + 1 == offsets.size() ? kernel.words.size() : offsets[index + 1];
    addresses[index] = boot + offsets[index];
    code[index] = {.address = addresses[index],
                   .words = std::span(kernel.words).subspan(offsets[index], end - offsets[index])};
  }
  constexpr std::array initial{boot};
  constexpr std::array target{handoff};
  const std::array variants{CodeVariant{.id = 0,
                                        .code = std::span(code).first(producer_instructions),
                                        .entries = initial,
                                        .failed_fetches = {}},
                            CodeVariant{.id = 1,
                                        .code = std::span(code).subspan(producer_instructions),
                                        .entries = target,
                                        .failed_fetches = {}}};
  const std::array choices{
      vi::dsp::ProgramChoice{
          .variant = 0, .resumes = std::span(addresses).first(producer_instructions), .words = {}},
      vi::dsp::ProgramChoice{.variant = 1, .resumes = target, .words = {}}};
  constexpr std::array grants{
      RegionLayout{.space = Space::x, .first = 0, .size = low_x_words},
      RegionLayout{.space = Space::x, .first = high_first, .size = high_x_words, .tracked = true},
      RegionLayout{.space = Space::y, .first = 0, .size = low_y_words},
      RegionLayout{.space = Space::y, .first = high_first, .size = high_y_words, .tracked = true}};
  const std::array layouts{VariantLayout{.id = 1, .memory = grants}};
  constexpr std::array transfers{CodeTransition{.from = 0, .to = 1, .entry = handoff}};
  return blofeld::app::aot::prepare_dsp_program(
      {.variants = variants}, choices, layouts, {}, transfers);
}

void publication() {
  const Kernel kernel(low_x_words);
  const auto program = prepare(kernel);
  vi::Shi shi;
  vi::DspReset reset;
  const auto result = vi::dsp::execute({.main = kernel.words, .eeprom = {}},
                                       shi,
                                       reset,
                                       {},
                                       nullptr,
                                       {},
                                       nullptr,
                                       nullptr,
                                       nullptr,
                                       nullptr,
                                       &program);
  require(!result.failure && !result.exception && !result.preparation_diagnostic &&
              result.instructions == producer_instructions + low_x_words + low_y_words &&
              result.stopped_pc == handoff + 2 &&
              result.prepared_native_bytes == program.report().native_bytes,
          "Actual native clears and PLL readback/write publish both low grants before handoff");
}

void incomplete() {
  const Kernel kernel(low_x_words - 1);
  const auto program = prepare(kernel);
  vi::Shi shi;
  vi::DspReset reset;
  bool refused = false;
  try {
    static_cast<void>(vi::dsp::execute({.main = kernel.words, .eeprom = {}},
                                       shi,
                                       reset,
                                       {},
                                       nullptr,
                                       {},
                                       nullptr,
                                       nullptr,
                                       nullptr,
                                       nullptr,
                                       &program));
  } catch (const std::runtime_error &error) {
    const std::string_view diagnostic(error.what());
    refused = diagnostic.contains("No prepared DSP image admits P:$ff0013") &&
              diagnostic.contains("OMR=$600389");
  }
  require(refused, "One unknown X word refuses untracked admission at the real completed PLL PC");
}
}

int main() try {
  publication();
  incomplete();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
