// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/execute.hpp"
#include "dsp/program.hpp"
#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/midi_port.hpp"
#include "io/shi.hpp"
#include "isa.hpp"
#include "storage/flash_image.hpp"
#include "vm_context.hpp"

#include <array>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {
using namespace dsp56300;
namespace vi = blofeld::vi;
constexpr Word boot = vi::dsp::BootRom::main_first;
constexpr Word literal = static_cast<Word>(isa::Opcode::move_x_effective) | (0x34U << 8) | 0x8000U;
constexpr Word store = static_cast<Word>(isa::Opcode::movem_effective) | (0x30U << 8) | 4U;
constexpr Word jump = static_cast<Word>(isa::Opcode::jmp_absolute_short);
constexpr Word stop = static_cast<Word>(isa::Opcode::stop);
constexpr std::array initial{literal, stop, store, Word{0}, jump};
constexpr std::array changed{stop};

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::vector<Word> bootstrap() {
  std::vector<Word> result;
  for (Word address = 0; address < initial.size(); ++address) {
    result.insert(result.end(), {literal, initial[address], store, address});
  }
  result.push_back(jump);
  return result;
}

vi::dsp::Program prepare(const std::span<const Word> boot_words) {
  const std::array first_code{CodeSpan{.address = boot, .words = boot_words}};
  const std::array second_code{CodeSpan{.address = 0, .words = initial}};
  const std::array final_code{CodeSpan{.address = 0, .words = changed},
                              CodeSpan{.address = 2, .words = std::span(initial).subspan(2)}};
  constexpr std::array first_entries{boot};
  constexpr std::array second_entries{Word{0}};
  constexpr std::array final_entries{Word{0}, Word{4}};
  const std::array variants{
      CodeVariant{.id = 0, .code = first_code, .entries = first_entries, .failed_fetches = {}},
      CodeVariant{.id = 1, .code = second_code, .entries = second_entries, .failed_fetches = {}},
      CodeVariant{.id = 2, .code = final_code, .entries = final_entries, .failed_fetches = {}}};
  const std::array initial_guard{vi::dsp::ProgramWord{.address = 0, .value = literal}};
  const std::array final_guards{vi::dsp::ProgramWord{.address = 0, .value = stop},
                                vi::dsp::ProgramWord{.address = 1, .value = stop}};
  const std::array choices{
      vi::dsp::ProgramChoice{.variant = 0, .resumes = first_entries, .words = {}},
      vi::dsp::ProgramChoice{.variant = 1, .resumes = second_entries, .words = initial_guard},
      vi::dsp::ProgramChoice{.variant = 2, .resumes = final_entries, .words = final_guards}};
  constexpr std::array transfers{CodeTransition{.from = 0, .to = 1, .entry = 0}};
  return blofeld::app::aot::prepare_dsp_program({.variants = variants}, choices, {}, {}, transfers);
}

void normal_owner() {
  const auto words = bootstrap();
  auto program = std::make_shared<vi::dsp::Program>(prepare(words));
  for (unsigned instance = 0; instance < 2; ++instance) {
    vi::Shi shi;
    vi::DspReset reset;
    const auto report = vi::dsp::execute({.main = words, .eeprom = {}},
                                         shi,
                                         reset,
                                         {},
                                         nullptr,
                                         {},
                                         nullptr,
                                         nullptr,
                                         nullptr,
                                         nullptr,
                                         program.get());
    require(!report.failure && !report.exception && !report.preparation_diagnostic &&
                report.instructions == 15 && report.stopped_pc == 1 &&
                report.prepared_native_bytes == program->report().native_bytes,
            "The normal DSP owner loads RAM, admits a transfer and selects a real changed-code "
            "image without guest replay");
  }
  const std::weak_ptr<const vi::dsp::Program> retained = program;
  {
    auto image = std::make_unique<vi::FlashImage>();
    const vi::VmContext vm(std::move(image), words, {}, {}, vi::MidiPort::InputMode::live, program);
    program.reset();
    require(!retained.expired(), "The VM owns prepared code independently of the producer");
  }
  require(retained.expired(), "VM destruction releases its prepared-code ownership");
}
}

int main() try { normal_owner(); } catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
