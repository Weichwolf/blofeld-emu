// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/execute.hpp"
#include "dsp/program.hpp"
#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/shi.hpp"
#include "isa.hpp"

#include <array>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <vector>

namespace {
using namespace dsp56300;
namespace vi = blofeld::vi;
constexpr Word boot = vi::dsp::BootRom::main_first;
constexpr Word literal = static_cast<Word>(isa::Opcode::move_x_effective) | (0x34U << 8) | 0x8000U;
constexpr Word store_p = static_cast<Word>(isa::Opcode::movem_effective) | (0x30U << 8) | 4U;
constexpr Word store_x = static_cast<Word>(isa::Opcode::move_x_effective) | (0x30U << 8);
constexpr Word read_p = static_cast<Word>(isa::Opcode::movem_effective) | (0x20U << 8) | 0x8004U;
constexpr Word receive = static_cast<Word>(isa::Opcode::movep_register_low) | 0x14U;
constexpr Word stop = static_cast<Word>(isa::Opcode::stop);
constexpr Word nop = static_cast<Word>(isa::Opcode::nop);
constexpr Word jump = static_cast<Word>(isa::Opcode::jmp_absolute_short);
constexpr Word source = 128, data = 32, target = 19, marker = 0xc000;
constexpr Word kernel_words = 24, fault_pc = 21, fault_instructions = 62, stop_instructions = 61;
constexpr Word source_register = 0xffffef, destination_register = 0xffffee;
constexpr Word count_register = 0xffffed, control_register = 0xffffec;
// Software-triggered single word, fixed P-space source and destination.
constexpr Word software_pp = 0x980000 | 2 | (2 << 2) | (4 << 4) | (4 << 7);
constexpr std::array kernel{receive | (4U << 8),
                            store_p,
                            source,
                            receive | (4U << 8),
                            store_x,
                            destination_register,
                            receive | (16U << 8),
                            literal,
                            source,
                            store_x,
                            source_register,
                            literal,
                            Word{0},
                            store_x,
                            count_register,
                            literal,
                            software_pp,
                            store_x,
                            control_register,
                            nop,
                            read_p,
                            store_p,
                            marker,
                            stop};
static_assert(kernel.size() == kernel_words && kernel[target] == nop);

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::vector<Word> bootstrap() {
  std::vector<Word> result;
  for (Word address = 0; address < kernel.size(); ++address) {
    result.insert(result.end(), {literal, kernel[address], store_p, address});
  }
  result.push_back(jump);
  return result;
}

vi::dsp::Program prepare(const std::span<const Word> words) {
  auto changed = kernel;
  changed[target] = stop;
  const std::array rom{CodeSpan{.address = boot, .words = words}};
  const std::array ram{CodeSpan{.address = 0, .words = kernel}};
  const std::array patched{CodeSpan{.address = 0, .words = changed}};
  constexpr std::array rom_entries{boot};
  constexpr std::array ram_entries{Word{0}};
  constexpr std::array patched_entries{target};
  const std::array variants{
      CodeVariant{.id = 0, .code = rom, .entries = rom_entries, .failed_fetches = {}},
      CodeVariant{.id = 1, .code = ram, .entries = ram_entries, .failed_fetches = {}},
      CodeVariant{.id = 2, .code = patched, .entries = patched_entries, .failed_fetches = {}}};
  constexpr std::array initial_guard{vi::dsp::ProgramWord{.address = target, .value = nop}};
  constexpr std::array changed_guard{vi::dsp::ProgramWord{.address = target, .value = stop}};
  const std::array choices{
      vi::dsp::ProgramChoice{.variant = 0, .resumes = rom_entries, .words = {}},
      vi::dsp::ProgramChoice{.variant = 1, .resumes = ram_entries, .words = initial_guard},
      vi::dsp::ProgramChoice{.variant = 2, .resumes = patched_entries, .words = changed_guard}};
  constexpr std::array transfers{CodeTransition{.from = 0, .to = 1, .entry = 0}};
  return blofeld::app::aot::prepare_dsp_program({.variants = variants}, choices, {}, {}, transfers);
}

struct Transfer {
  Word destination, value;
};

vi::ExecutionReport execute(const vi::dsp::Program &program,
                            const std::span<const Word> words,
                            const Transfer transfer) {
  vi::Shi shi;
  vi::DspReset reset;
  constexpr Word receive_control = 0xffff91;
  constexpr Word fifo24 = 0x29;
  require(shi.write({.address = receive_control, .value = fifo24}) &&
              shi.select(true).has_value() && shi.transfer(transfer.value).accepted &&
              shi.transfer(transfer.destination).accepted &&
              shi.transfer(transfer.destination).accepted,
          "Supply payload and destination through the real receive FIFO");
  return vi::dsp::execute({.main = words, .eeprom = {}},
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
}

void data_and_equal_writes(const vi::dsp::Program &program, const std::span<const Word> words) {
  constexpr Word payload = 0x5a;
  for (const auto transfer : {Transfer{.destination = data, .value = payload},
                              Transfer{.destination = target, .value = nop}}) {
    const auto report = execute(program, words, transfer);
    const bool valid =
        report.failure && !report.exception && !report.preparation_diagnostic &&
        report.failure->operation == vi::Access::write && report.failure->space == 'P' &&
        report.failure->pc == fault_pc && report.failure->address == marker &&
        report.failure->value == transfer.value && report.instructions == fault_instructions;
    if (!valid && report.failure) {
      const auto &failure = *report.failure;
      std::cerr << "DMA destination=" << transfer.destination << " retired=" << report.instructions
                << " pc=" << failure.pc << " address=" << failure.address
                << " value=" << failure.value << " fault=" << failure.code
                << " operation=" << static_cast<unsigned>(failure.operation) << '\n';
    }
    require(valid,
            "Inactive P data and equal code writes complete DMA and preserve the running image; "
            "the guest reads back the actual transferred word before its intentional write fault");
  }
}

void changed_code(const vi::dsp::Program &program, const std::span<const Word> words) {
  const auto report = execute(program, words, {.destination = target, .value = stop});
  require(!report.failure && !report.exception && !report.preparation_diagnostic &&
              report.instructions == stop_instructions && report.stopped_pc == target + 1,
          "A changed executable DMA store installs the prepared version at the real next PC; "
          "the new STOP executes before the old image's intentional fault");
}
}

int main() try {
  const auto words = bootstrap();
  const auto program = prepare(words);
  data_and_equal_writes(program, words);
  changed_code(program, words);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
