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

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
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

void normal_owner(const Fixture &fixture, const bool fixed = false) {
  auto input = catalogue(fixture);
  if (fixed) {
    constexpr Word low_x_words = 0x3000;
    constexpr Word high_first = 0xc000;
    constexpr Word high_x_words = 0x6000;
    constexpr Word high_y_words = 0xa000;
    const auto region = [](const char *space, const Word first, const Word count) {
      return Object{{.K = "space", .V = space},
                    {.K = "first", .V = first},
                    {.K = "words", .V = count},
                    {.K = "tracked", .V = true}};
    };
    for (const unsigned index : {1U, 2U}) {
      (*(*input.getArray("variants"))[index].getAsObject())["memory"] =
          Array{region("x", 0, low_x_words),
                region("x", high_first, high_x_words),
                region("y", 0, 0x2000),
                region("y", high_first, high_y_words)};
    }
  }
  const auto program = blofeld::app::aot::prepare_dsp_catalogue(
      document(std::move(input)), fixture.firmware, fixture.boot_rom());
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

Object software_catalogue(Fixture &fixture, const bool long_vector, const Word instruction) {
  constexpr Word trap = static_cast<Word>(dsp56300::isa::Opcode::trap);
  const Word vector = instruction == trap ? 8 : 4;
  constexpr Word callee = 16;
  constexpr Word nop = static_cast<Word>(dsp56300::isa::Opcode::nop);
  constexpr Word call = static_cast<Word>(dsp56300::isa::Opcode::jsr_effective) | (0x30U << 8U);
  constexpr Word rti = static_cast<Word>(dsp56300::isa::Opcode::rti);
  std::vector<Word> words(long_vector ? callee + 1 : vector + 2, nop);
  words[0] = instruction;
  words[1] = stop;
  if (long_vector) {
    words[vector] = call;
    words[vector + 1] = callee;
    words[callee] = rti;
  }
  fixture.firmware.blocks[0] = packed(words);
  fixture.rom.clear();
  for (Word address = 0; address < words.size(); ++address) {
    fixture.rom.insert(fixture.rom.end(), {literal, words[address], store, address});
  }
  fixture.rom.push_back(jump);
  auto result = catalogue(fixture);
  auto &variants = *result.getArray("variants");
  auto &loaded = *variants[1].getAsObject();
  loaded["writes"] = Array{Object{{.K = "kind", .V = "block"},
                                  {.K = "block", .V = 0},
                                  {.K = "byte_offset", .V = 0},
                                  {.K = "address", .V = 0},
                                  {.K = "words", .V = words.size()}}};
  Array code{Array{0, 2}, Array{vector, 2}};
  if (long_vector) { code.push_back(Array{callee, 1}); }
  loaded["code"] = std::move(code);
  loaded["guards"] = Array{Object{{.K = "address", .V = 0}, {.K = "value", .V = instruction}}};
  result["variants"] = Array{std::move(variants[0]), std::move(variants[1])};
  result["interrupts"] = Array{Object{{.K = "vector", .V = vector}, {.K = "level", .V = 3}}};
  return result;
}

void software_owner(const Fixture &fixture, Object input, const Word instructions) {
  const auto program = blofeld::app::aot::prepare_dsp_catalogue(
      document(std::move(input)), fixture.firmware, fixture.boot_rom());
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
              result.instructions == instructions && result.stopped_pc == 2,
          "A prepared level-three software exception executes its actual vectors and returns "
          "through the normal owner despite the initial interrupt mask");
}

void software_interrupts() {
  constexpr Word trap = static_cast<Word>(dsp56300::isa::Opcode::trap);
  constexpr Word undefined = 0x000143; // Authored reserved-domain member, not firmware code.
  constexpr Word long_instructions = 39;
  for (const Word instruction : {trap, undefined}) {
    const Word fast_instructions = instruction == trap ? 25 : 17;
    for (const bool long_vector : {false, true}) {
      Fixture fixture;
      auto input = software_catalogue(fixture, long_vector, instruction);
      software_owner(
          fixture, std::move(input), long_vector ? long_instructions : fast_instructions);
    }
  }
}

void derived_word() {
  constexpr unsigned shift = 22;
  constexpr Word trap = static_cast<Word>(dsp56300::isa::Opcode::trap);
  Fixture fixture;
  auto input = software_catalogue(fixture, false, trap);
  auto &loaded = *(*input.getArray("variants"))[1].getAsObject();
  // (TRAP & bit2) << 22 wraps to the NOP independently loaded at vector8 by guest MOVEM.
  loaded.getArray("writes")->push_back(Object{{.K = "kind", .V = "field"},
                                              {.K = "from", .V = 0},
                                              {.K = "address", .V = 8},
                                              {.K = "mask", .V = 4},
                                              {.K = "shift", .V = shift}});
  constexpr Word instructions = 25;
  software_owner(fixture, std::move(input), instructions);
}

void nested_software() {
  constexpr Word trap = static_cast<Word>(dsp56300::isa::Opcode::trap);
  constexpr Word undefined = 0x000143;
  constexpr Word vector = 8;
  constexpr unsigned loader_words = 4;
  Fixture fixture;
  auto input = software_catalogue(fixture, false, trap);
  const auto bytes = packed(std::array{undefined});
  std::ranges::copy(
      bytes,
      std::span(fixture.firmware.blocks[0]).subspan(static_cast<std::size_t>(vector) * 3).begin());
  fixture.rom[(vector * loader_words) + 1] = undefined;
  auto &identity = *input.getObject("inputs");
  (*identity.getArray("blocks"))[0] = hash(fixture.firmware.blocks[0]);
  identity["boot-main"] = hash(packed(fixture.rom));
  auto &loaded = *(*input.getArray("variants"))[1].getAsObject();
  loaded.getArray("code")->push_back(Array{4, 2});
  input.getArray("interrupts")->push_back(Object{{.K = "vector", .V = 4}, {.K = "level", .V = 3}});
  constexpr Word instructions = 26;
  software_owner(fixture, std::move(input), instructions);
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
  rejects(fixture, [](auto &input) {
    variant(input)["memory"] =
        Array{Object{{.K = "space", .V = "x"}, {.K = "first", .V = 0}, {.K = "words", .V = 0}}};
  });
  rejects(fixture, [](auto &input) {
    input["interrupts"] = Array{Object{{.K = "vector", .V = 8}, {.K = "level", .V = 4}}};
  });
  rejects(fixture, [](auto &input) {
    variant(input)["memory"] =
        Array{Object{{.K = "space", .V = "p"}, {.K = "first", .V = 0}, {.K = "words", .V = 1}}};
  });
  rejects(fixture, [](auto &input) {
    variant(input)["guards"] =
        Array{Object{{.K = "address", .V = 0x1000000}, {.K = "value", .V = nullptr}}};
  });
}

void duplicate_loaded(Object &input) {
  auto &variants = *input.getArray("variants");
  auto duplicate = variants[1];
  (*duplicate.getAsObject())["id"] = 3;
  variants.push_back(std::move(duplicate));
}

void ambiguous_admission(const Fixture &fixture) {
  rejects(fixture, [](auto &input) { duplicate_loaded(input); });
  rejects(fixture, [](auto &input) {
    duplicate_loaded(input);
    variant(input, 3)["memory"] = Array{};
  });
  rejects(fixture, [](auto &input) {
    duplicate_loaded(input);
    variant(input, 1)["mode_mask"] = 0x80;
    variant(input, 1)["mode"] = 0;
    variant(input, 3)["mode_mask"] = 0x100;
    variant(input, 3)["mode"] = 0;
  });
  rejects(fixture, [](auto &input) {
    duplicate_loaded(input);
    for (auto &value : *input.getArray("variants")) { (*value.getAsObject())["memory"] = Array{}; }
    Object x{{.K = "space", .V = "x"}, {.K = "first", .V = 0}, {.K = "words", .V = 8}};
    Object y{{.K = "space", .V = "y"}, {.K = "first", .V = 0}, {.K = "words", .V = 8}};
    const llvm::json::Value first(std::move(x));
    const llvm::json::Value second(std::move(y));
    variant(input, 1)["memory"] = Array{first, second};
    variant(input, 3)["memory"] = Array{second, first};
  });
}

void disjoint_admission(const Fixture &fixture) {
  for (const bool memory : {false, true}) {
    auto input = catalogue(fixture);
    duplicate_loaded(input);
    if (memory) {
      for (auto &value : *input.getArray("variants")) {
        (*value.getAsObject())["memory"] = Array{};
      }
      variant(input, 1)["memory"] = Array{Object{{.K = "space", .V = "x"},
                                                 {.K = "first", .V = 0},
                                                 {.K = "words", .V = 8},
                                                 {.K = "tracked", .V = true}}};
      variant(input, 3)["memory"] = Array{Object{{.K = "space", .V = "x"},
                                                 {.K = "first", .V = 0},
                                                 {.K = "words", .V = 8},
                                                 {.K = "tracked", .V = false}}};
    } else {
      variant(input, 1)["mode_mask"] = 0x80;
      variant(input, 1)["mode"] = 0;
      variant(input, 3)["mode_mask"] = 0x80;
      variant(input, 3)["mode"] = 0x80;
    }
    const auto program = blofeld::app::aot::prepare_dsp_catalogue(
        document(std::move(input)), fixture.firmware, fixture.boot_rom());
    require(program.report().prepared_entries != 0 && program.report().unresolved_entries == 0,
            "Disjoint OMR predicates and initialized-memory layouts retain valid prepared choices");
  }
}

void invalid_fields(const Fixture &fixture) {
  for (const unsigned example : {0U, 1U, 2U, 3U}) {
    Object field{{.K = "kind", .V = "field"},
                 {.K = "from", .V = 0},
                 {.K = "address", .V = 0},
                 {.K = "mask", .V = dsp56300::word_mask},
                 {.K = "shift", .V = 0}};
    constexpr unsigned bits = 24;
    if (example == 0) { field["shift"] = bits; }
    if (example == 1) { field["shift"] = -1; }
    if (example == 2) { field["mask"] = dsp56300::address_words; }
    constexpr Word missing = 32;
    if (example == 3) { field["from"] = missing; }
    rejects(fixture, [&](auto &input) {
      variant(input, 1).getArray("writes")->push_back(std::move(field));
    });
  }
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

int main(const int argc, const char *const *argv) try {
  require(argc <= 3, "Supply an optional test directory and catalogue test group");
  const std::string_view group = argc == 3 ? argv[2] : "inputs";
  if (group == "inputs") {
    const Fixture fixture;
    ambiguous_admission(fixture);
    disjoint_admission(fixture);
    normal_owner(fixture);
    invalid_catalogues(fixture);
    invalid_fields(fixture);
    changed_inputs(fixture);
  } else if (group == "layouts") {
    normal_owner(Fixture{}, true);
  } else if (group == "derived") {
    derived_word();
  } else if (group == "software") {
    software_interrupts();
  } else if (group == "nested") {
    nested_software();
  } else {
    throw std::invalid_argument("Unknown catalogue test group");
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
