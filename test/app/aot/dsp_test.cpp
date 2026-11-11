// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/code_source.hpp"
#include "dsp/io.hpp"
#include "io/dsp_reset.hpp"
#include "io/pcm_output.hpp"
#include "io/shi.hpp"
#include "isa.hpp"
#include "storage/dsp_records.hpp"
#include "support/dsp_records.hpp"

#include <array>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
#include <stop_token>

namespace {
using namespace dsp56300;
namespace vi = blofeld::vi;
constexpr Word entry = vi::dsp::BootRom::main_first;
constexpr Word control = 0xffff91;
constexpr Word clock_register = 0xffff90;
constexpr Word receive = 0xffff94;
constexpr Word x0 = 4;
constexpr Word x1 = 5;
constexpr Word receive_port = 0x14;
constexpr Word clock_port = 0x10;
constexpr Word pctl_port = 0x3d;
constexpr Word write_direction = 0x8000;
constexpr unsigned high_word = 24;
constexpr Word receive_word =
    static_cast<Word>(isa::Opcode::movep_register_low) | (x0 << 8U) | receive_port;
constexpr Word pctl_word =
    static_cast<Word>(isa::Opcode::movep_register_high) | (x1 << 8U) | pctl_port;
constexpr Word write_clock_word =
    static_cast<Word>(isa::Opcode::movep_register_low) | (x0 << 8U) | clock_port | write_direction;
constexpr Word read_clock_word =
    static_cast<Word>(isa::Opcode::movep_register_low) | (x1 << 8U) | clock_port;
constexpr auto read_words = isa::program(std::array{receive_word, pctl_word}, isa::stop);
constexpr auto write_words = isa::program(std::array{write_clock_word, read_clock_word}, isa::stop);
enum class Action : std::uint8_t { read, empty, write };

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

NativeImage prepare() {
  blofeld::test::DspRecords transport;
  transport.add(0, entry, read_words);
  transport.add(0, entry, write_words);
  const auto bytes = transport.finish();
  const auto records = vi::DspRecords::read(bytes);
  constexpr std::array ranges{vi::dsp::CodeRange{.address = entry, .words = read_words.size()}};
  const vi::dsp::CodeSource read(records.records().first(1), ranges);
  const vi::dsp::CodeSource write(records.records(), ranges);
  constexpr std::array entries{entry};
  const std::array variants{
      CodeVariant{.id = 0, .code = read.code(), .entries = entries, .failed_fetches = {}},
      CodeVariant{.id = 1, .code = write.code(), .entries = entries, .failed_fetches = {}}};
  auto native = blofeld::app::aot::prepare_dsp({.variants = variants});
  require(native.report().prepared_entries == read_words.size() + write_words.size() &&
              native.report().native_bytes != 0 && native.report().failure.kind == FaultKind::none,
          "The application prepares every authored variant through its built-in concrete binding");
  return native;
}

void execute(const NativeImage &native, const Action action, const Word payload) {
  vi::Shi shi;
  vi::DspReset reset;
  const std::stop_token stop;
  const bool write = action == Action::write;
  const auto words = write ? std::span(write_words) : std::span(read_words);
  const auto io = std::make_unique<vi::dsp::Io>(vi::dsp::BootRom{.main = words, .eeprom = {}},
                                                shi,
                                                reset,
                                                stop,
                                                nullptr,
                                                vi::PcmOutput{},
                                                nullptr,
                                                nullptr,
                                                nullptr,
                                                nullptr);
  Core core(Memory::external(), io.get());
  io->bind(core, 0);
  core.reset();
  auto registers = core.registers();
  constexpr Word reset_mode = 0x305;
  constexpr Word reset_status = 0xc00300;
  registers.pc = entry;
  registers.omr = reset_mode;
  registers.sr = reset_status;
  registers.x = payload;
  core.set_registers(registers);
  core.install(native, write ? 1 : 0);
  if (action == Action::read) {
    constexpr Word single_word = 9;
    require(shi.write({.address = control, .value = single_word}) && shi.select(true).has_value() &&
                shi.transfer(payload).accepted,
            "Publish a real native transport word without a binding-specific device copy");
  }
  const auto result = core.run();
  if (action == Action::empty) {
    require(result.fault.kind == FaultKind::io_read && result.fault.pc == entry &&
                result.fault.space == Space::x && result.fault.address == receive &&
                result.retired == 0,
            "A real empty imported FIFO preserves the exact failed-access prefix");
    return;
  }
  constexpr std::uint64_t reset_pctl = 0x04601d;
  constexpr std::uint64_t clock_mask = 0x7ff;
  const auto high = write ? payload & clock_mask : reset_pctl;
  require(result.fault.kind == FaultKind::none && result.retired == words.size() &&
              result.state == ExecutionState::stopped &&
              core.registers().x == (high << high_word | payload),
          "Prepared native code retains the actual register effects and complete guest result");
  Word value = 0;
  if (write) {
    require(shi.read(clock_register, value) && value == (payload & clock_mask),
            "The prepared write changed the same native register read by the device owner");
  } else {
    require(!shi.read(receive, value), "The original native FIFO payload is consumed exactly once");
  }
}

void incomplete_input() {
  bool rejected = false;
  try {
    static_cast<void>(blofeld::app::aot::prepare_dsp({.variants = {}}));
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "The application cannot turn an absent code catalogue into prepared success");

  constexpr Word missing = 0x20;
  constexpr auto call = static_cast<Word>(isa::Opcode::jsr_absolute_short) | missing;
  constexpr auto words = isa::program(std::array{call}, isa::stop);
  const std::array spans{CodeSpan{.address = entry, .words = words}};
  constexpr std::array entries{entry};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  rejected = false;
  try {
    static_cast<void>(blofeld::app::aot::prepare_dsp({.variants = variants}));
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "A missing native successor fails before playback without lazy translation");
}
}

int main() try {
  const auto native = prepare();
  constexpr Word first = 0x53a2f1;
  constexpr Word second = 0xa53971;
  constexpr Word clock_fields = 0xffffff;
  execute(native, Action::read, first);
  execute(native, Action::read, second);
  execute(native, Action::read, 0);
  execute(native, Action::empty, 0);
  execute(native, Action::write, clock_fields);
  incomplete_input();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
