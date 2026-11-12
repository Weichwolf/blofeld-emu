// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/execute.hpp"
#include "io/dsp_reset.hpp"
#include "io/shi.hpp"
#include "isa.hpp"
#include "storage/firmware.hpp"
#include "support/dsp_records.hpp"

#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/FormatVariadic.h"
#include "llvm/Support/JSON.h"
#include "llvm/Support/SHA256.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
using dsp56300::Word;
using Object = llvm::json::Object;
using Array = llvm::json::Array;
namespace vi = blofeld::vi;
constexpr Word boot = vi::dsp::BootRom::main_first;
constexpr Word literal =
    static_cast<Word>(dsp56300::isa::Opcode::move_x_effective) | (0x34U << 8) | 0x8000U;
constexpr Word store =
    static_cast<Word>(dsp56300::isa::Opcode::movem_effective) | (0x30U << 8) | 4U;
constexpr Word jump = static_cast<Word>(dsp56300::isa::Opcode::jmp_absolute_short);
constexpr Word stop = static_cast<Word>(dsp56300::isa::Opcode::stop);
constexpr std::array initial{literal, stop, store, Word{0}, jump};
constexpr std::array eeprom{stop};

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::vector<std::uint8_t> packed(const std::span<const Word> words) {
  std::vector<std::uint8_t> result;
  for (const auto word : words) {
    result.insert(result.end(),
                  {static_cast<std::uint8_t>(word >> 16U),
                   static_cast<std::uint8_t>(word >> 8U),
                   static_cast<std::uint8_t>(word)});
  }
  return result;
}

std::string hash(const std::span<const std::uint8_t> bytes) {
  return llvm::toHex(llvm::SHA256::hash({bytes.data(), bytes.size()}), true);
}

struct Fixture {
  vi::FirmwareUpdate firmware;
  std::vector<Word> rom;

  Fixture() {
    firmware.blocks[0] = packed(initial);
    blofeld::test::DspRecords records;
    records.add(0, 0, initial);
    constexpr std::array data{Word{0}};
    records.add(1, 0, data);
    records.add(2, 0, data);
    firmware.nand_program = records.finish();
    for (Word address = 0; address < initial.size(); ++address) {
      rom.insert(rom.end(), {literal, initial[address], store, address});
    }
    rom.push_back(jump);
  }

  [[nodiscard]] vi::dsp::BootRom boot_rom() const { return {.main = rom, .eeprom = eeprom}; }
};

Array code_range(Array range) {
  Array result;
  result.push_back(std::move(range));
  return result;
}

Object catalogue(const Fixture &fixture) {
  Array blocks;
  for (const auto &bytes : fixture.firmware.blocks) { blocks.push_back(hash(bytes)); }
  Object inputs{{.K = "blocks", .V = std::move(blocks)},
                {.K = "nand", .V = hash(fixture.firmware.nand_program)},
                {.K = "boot-main", .V = hash(packed(fixture.rom))},
                {.K = "boot-eeprom", .V = hash(packed(eeprom))}};
  Object rom{
      {.K = "id", .V = 0},
      {.K = "writes",
       .V = Array{Object{{.K = "kind", .V = "boot-main"},
                         {.K = "address", .V = boot},
                         {.K = "word_offset", .V = 0},
                         {.K = "words", .V = fixture.rom.size()}},
                  Object{{.K = "kind", .V = "boot-eeprom"},
                         {.K = "address", .V = vi::dsp::BootRom::eeprom_first},
                         {.K = "word_offset", .V = 0},
                         {.K = "words", .V = eeprom.size()}}}},
      {.K = "code",
       .V = Array{Array{boot, fixture.rom.size()},
                  Array{vi::dsp::BootRom::eeprom_first, eeprom.size()}}},
      {.K = "entries", .V = Array{boot}},
      {.K = "resumes", .V = Array{boot}},
      {.K = "guards", .V = Array{Object{{.K = "address", .V = 0}, {.K = "value", .V = nullptr}}}}};
  Object loaded{
      {.K = "id", .V = 1},
      {.K = "writes", .V = Array{Object{{.K = "kind", .V = "records"}, {.K = "count", .V = 3}}}},
      {.K = "code", .V = code_range(Array{0, initial.size()})},
      {.K = "entries", .V = Array{0}},
      {.K = "resumes", .V = Array{0}},
      {.K = "guards", .V = Array{Object{{.K = "address", .V = 0}, {.K = "value", .V = literal}}}}};
  Object changed{{.K = "id", .V = 2},
                 {.K = "writes",
                  .V = Array{Object{{.K = "kind", .V = "block"},
                                    {.K = "block", .V = 0},
                                    {.K = "byte_offset", .V = 0},
                                    {.K = "address", .V = 0},
                                    {.K = "words", .V = initial.size()}},
                             Object{{.K = "kind", .V = "copy"},
                                    {.K = "from", .V = 1},
                                    {.K = "address", .V = 0},
                                    {.K = "words", .V = 1}}}},
                 {.K = "code", .V = Array{Array{0, 1}, Array{2, initial.size() - 2}}},
                 {.K = "entries", .V = Array{0, 4}},
                 {.K = "resumes", .V = Array{0, 4}},
                 {.K = "mode_mask", .V = 0x80},
                 {.K = "mode", .V = 0},
                 {.K = "guards",
                  .V = Array{Object{{.K = "address", .V = 0}, {.K = "value", .V = stop}},
                             Object{{.K = "address", .V = 1}, {.K = "value", .V = stop}}}},
                 {.K = "failed_fetches", .V = Array{}}};
  Object result;
  result["format"] = 1;
  result["inputs"] = std::move(inputs);
  result["variants"] = Array{std::move(rom), std::move(loaded), std::move(changed)};
  result["interrupts"] = Array{};
  result["transitions"] =
      Array{Object{{.K = "from", .V = 0}, {.K = "to", .V = 1}, {.K = "entry", .V = 0}}};
  return result;
}

std::string document(Object input) {
  return llvm::formatv("{0}", llvm::json::Value(std::move(input))).str();
}

void normal_owner(const Fixture &fixture) {
  const auto program = blofeld::app::aot::prepare_dsp_catalogue(
      document(catalogue(fixture)), fixture.firmware, fixture.boot_rom());
  vi::Shi shi;
  vi::DspReset reset;
  const auto result = vi::dsp::execute(fixture.boot_rom(),
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
              result.instructions == 15 && result.stopped_pc == 1 &&
              result.prepared_native_bytes == program.report().native_bytes,
          "An input-bound catalogue runs its prepared variants through the normal DSP owner");
}

template <class Change> void rejects(const Fixture &fixture, Change change) {
  auto input = catalogue(fixture);
  change(input);
  bool rejected = false;
  try {
    const auto program = blofeld::app::aot::prepare_dsp_catalogue(
        document(std::move(input)), fixture.firmware, fixture.boot_rom());
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Malformed or differently bound catalogues fail before execution");
}

Object &variant(Object &input, const std::size_t index = 0) {
  return *(*input.getArray("variants"))[index].getAsObject();
}

Object &write(Object &input) {
  return *(*variant(input).getArray("writes"))[0].getAsObject();
}

void invalid_catalogues(const Fixture &fixture) {
  rejects(fixture, [](auto &input) { input["format"] = 2; });
  rejects(fixture, [](auto &input) { input.getObject("inputs")->erase("nand"); });
  rejects(fixture, [](auto &input) { input.getObject("inputs")->operator[]("nand") = "0"; });
  rejects(fixture, [](auto &input) { input.getObject("inputs")->operator[]("blocks") = Array{}; });
  rejects(fixture, [](auto &input) { write(input)["kind"] = "unknown"; });
  rejects(fixture, [](auto &input) { write(input)["word_offset"] = -1; });
  rejects(fixture, [](auto &input) { write(input)["words"] = 0x1000000; });
  rejects(fixture,
          [](auto &input) { variant(input)["code"] = code_range(Array{dsp56300::word_mask, 2}); });
  rejects(fixture, [](auto &input) { variant(input)["code"] = code_range(Array{0, 1}); });
  rejects(fixture, [](auto &input) { variant(input)["failed_fetches"] = false; });
  rejects(fixture, [](auto &input) { variant(input)["memory"] = false; });
  rejects(fixture, [](auto &input) { variant(input)["memory"] = Array{}; });
  rejects(fixture, [](auto &input) {
    variant(input)["memory"] =
        Array{Object{{.K = "space", .V = "p"}, {.K = "first", .V = 0}, {.K = "words", .V = 1}}};
  });
  rejects(fixture, [](auto &input) {
    variant(input)["guards"] =
        Array{Object{{.K = "address", .V = 0x1000000}, {.K = "value", .V = nullptr}}};
  });
}

void rejects_input(const std::string &document,
                   const vi::FirmwareUpdate &firmware,
                   const vi::dsp::BootRom rom) {
  bool rejected = false;
  try {
    const auto program = blofeld::app::aot::prepare_dsp_catalogue(document, firmware, rom);
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Every decoded input and each separate ROM region is bound before lowering");
}

void changed_inputs(const Fixture &fixture) {
  const auto input = document(catalogue(fixture));
  for (std::size_t index = 0; index < fixture.firmware.blocks.size(); ++index) {
    auto changed = fixture.firmware;
    changed.blocks[index].push_back(0);
    rejects_input(input, changed, fixture.boot_rom());
  }
  auto changed = fixture.firmware;
  changed.nand_program.push_back(0);
  rejects_input(input, changed, fixture.boot_rom());
  auto main = fixture.rom;
  main[0] ^= 1;
  rejects_input(input, fixture.firmware, {.main = main, .eeprom = eeprom});
  auto other = eeprom;
  other[0] ^= 1;
  rejects_input(input, fixture.firmware, {.main = fixture.rom, .eeprom = other});
  rejects_input(input, fixture.firmware, {.main = fixture.rom, .eeprom = {}});
}
}

int main() try {
  const Fixture fixture;
  normal_owner(fixture);
  invalid_catalogues(fixture);
  changed_inputs(fixture);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
