// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/code_source.hpp"
#include "dsp/program.hpp"
#include "storage/dsp_records.hpp"
#include "storage/firmware.hpp"
#include "storage/update_reader.hpp"

#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/JSON.h"
#include "llvm/Support/SHA256.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <format>
#include <optional>
#include <span>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <vector>

namespace blofeld::app::aot {
namespace {
using dsp56300::Word;
using Object = llvm::json::Object;
using Array = llvm::json::Array;

[[noreturn]] void invalid(const std::string_view field) {
  throw std::invalid_argument(std::format("Invalid DSP catalogue field: {}", field));
}

const Object &object(const llvm::json::Value &value) {
  const auto *result = value.getAsObject();
  if (result == nullptr) { invalid("object"); }
  return *result;
}

const Array &array(const Object &value, const llvm::StringRef field) {
  const auto *result = value.getArray(field);
  if (result == nullptr) { invalid(field.str()); }
  return *result;
}

std::uint64_t number(const llvm::json::Value &value) {
  const auto result = value.getAsInteger();
  if (!result || *result < 0) { invalid("unsigned integer"); }
  return static_cast<std::uint64_t>(*result);
}

std::uint64_t number(const Object &value, const llvm::StringRef field) {
  const auto *result = value.get(field);
  if (result == nullptr) { invalid(field.str()); }
  return number(*result);
}

Word word(const std::uint64_t value) {
  if (value > dsp56300::word_mask) { invalid("24-bit word"); }
  return static_cast<Word>(value);
}

Word word(const Object &value, const llvm::StringRef field) {
  return word(number(value, field));
}

std::string_view text(const Object &value, const llvm::StringRef field) {
  const auto result = value.getString(field);
  if (!result) { invalid(field.str()); }
  return *result;
}

bool boolean(const Object &value, const llvm::StringRef field, const bool fallback) {
  if (value.get(field) == nullptr) { return fallback; }
  const auto result = value.getBoolean(field);
  if (!result) { invalid(field.str()); }
  return *result;
}

void identity(const std::span<const std::uint8_t> bytes, const std::string_view expected) {
  const auto digest = llvm::SHA256::hash({bytes.data(), bytes.size()});
  if (llvm::toHex(digest, true) != expected) {
    throw std::invalid_argument("DSP catalogue source SHA256 mismatch");
  }
}

void identity(const std::span<const Word> words, const std::string_view expected) {
  std::vector<std::uint8_t> bytes;
  bytes.reserve(words.size() * 3);
  for (const auto value : words) {
    if (value > dsp56300::word_mask) { invalid("ROM word"); }
    bytes.insert(bytes.end(),
                 {static_cast<std::uint8_t>(value >> 16U),
                  static_cast<std::uint8_t>(value >> 8U),
                  static_cast<std::uint8_t>(value)});
  }
  identity(bytes, expected);
}

void inputs(const Object &root, const vi::FirmwareUpdate &firmware, const vi::dsp::BootRom boot) {
  if (!boot.valid()) { invalid("boot regions"); }
  const auto *source = root.getObject("inputs");
  if (source == nullptr) { invalid("inputs"); }
  const auto &blocks = array(*source, "blocks");
  if (blocks.size() != firmware.blocks.size()) { invalid("block identities"); }
  for (std::size_t index = 0; index < blocks.size(); ++index) {
    const auto hash = blocks[index].getAsString();
    if (!hash) { invalid("block SHA256"); }
    identity(firmware.blocks[index], *hash);
  }
  identity(firmware.nand_program, text(*source, "nand"));
  identity(boot.main, text(*source, "boot-main"));
  identity(boot.eeprom, text(*source, "boot-eeprom"));
}

std::vector<Word> addresses(const Array &input) {
  std::vector<Word> result;
  for (const auto &value : input) { result.push_back(word(number(value))); }
  return result;
}

std::vector<vi::dsp::CodeRange> ranges(const Array &input) {
  std::vector<vi::dsp::CodeRange> result;
  for (const auto &value : input) {
    const auto *pair = value.getAsArray();
    if (pair == nullptr || pair->size() != 2) { invalid("instruction range"); }
    result.push_back({.address = word(number((*pair)[0])), .words = number((*pair)[1])});
  }
  return result;
}

class Stored {
public:
  void bytes(const Word address, const std::span<const std::uint8_t> bytes) {
    if (bytes.empty() || bytes.size() % 3 != 0) { invalid("stored program bytes"); }
    vi::update::Reader reader(bytes);
    auto &words = words_.emplace_back();
    while (!reader.empty()) { words.push_back(reader.big(3)); }
    writes_.push_back({.address = address, .words = words});
  }

  void write(const dsp56300::CodeSpan input) {
    auto &words = words_.emplace_back(input.words.begin(), input.words.end());
    writes_.push_back({.address = input.address, .words = words});
  }

  void copy(const Object &input) {
    const std::array source{
        vi::dsp::CodeRange{.address = word(input, "from"), .words = number(input, "words")}};
    const vi::dsp::CodeSource snapshot(writes_, source);
    write({.address = word(input, "address"), .words = snapshot.code()[0].words});
  }

  void field(const Object &input) {
    constexpr std::uint64_t bits = 24;
    const auto shift = number(input, "shift");
    if (shift >= bits) { invalid("field shift"); }
    const std::array source{vi::dsp::CodeRange{.address = word(input, "from"), .words = 1}};
    const vi::dsp::CodeSource snapshot(writes_, source);
    const std::array value{((snapshot.code()[0].words[0] & word(input, "mask")) << shift) &
                           dsp56300::word_mask};
    write({.address = word(input, "address"), .words = value});
  }

  [[nodiscard]] vi::dsp::CodeSource classify(const Array &code) const {
    return {writes_, ranges(code)};
  }

private:
  std::vector<std::vector<Word>> words_;
  std::vector<dsp56300::CodeSpan> writes_;
};

void stored_write(Stored &stored,
                  const Object &input,
                  const vi::FirmwareUpdate &firmware,
                  const vi::dsp::BootRom boot) {
  const auto kind = text(input, "kind");
  if (kind == "copy") {
    stored.copy(input);
  } else if (kind == "field") {
    stored.field(input);
  } else if (kind == "records") {
    const auto records = vi::DspRecords::read(firmware.nand_program);
    const auto count = number(input, "count");
    if (count > records.records().size()) { invalid("record prefix"); }
    for (const auto &record : records.records().first(count)) {
      if (record.space == 0) { stored.bytes(record.address, record.payload); }
    }
  } else if (kind == "block") {
    const auto block = number(input, "block");
    if (block >= firmware.blocks.size()) { invalid("block index"); }
    const auto bytes = std::span(firmware.blocks[block]);
    const auto offset = number(input, "byte_offset");
    const auto count = number(input, "words");
    if (offset > bytes.size() || count > (bytes.size() - offset) / 3) { invalid("block extent"); }
    stored.bytes(word(input, "address"), bytes.subspan(offset, count * 3));
  } else if (kind == "boot-main" || kind == "boot-eeprom") {
    const auto words = kind == "boot-main" ? boot.main : boot.eeprom;
    const auto offset = number(input, "word_offset");
    const auto count = number(input, "words");
    if (offset > words.size() || count > words.size() - offset) { invalid("ROM extent"); }
    stored.write({.address = word(input, "address"), .words = words.subspan(offset, count)});
  } else {
    invalid("stored write kind");
  }
}

struct Variant {
  std::uint64_t id;
  vi::dsp::CodeSource code;
  std::vector<Word> entries, resumes, failed_fetches;
  std::vector<vi::dsp::ProgramWord> guards;
  std::vector<dsp56300::RegionLayout> memory;
  Word mode_mask = 0, mode = 0;
  bool fixed_memory = false;
};

void admission(Variant &variant, const Object &input) {
  variant.entries = addresses(array(input, "entries"));
  variant.resumes = addresses(array(input, "resumes"));
  if (input.get("failed_fetches") != nullptr) {
    variant.failed_fetches = addresses(array(input, "failed_fetches"));
  }
  if (input.get("mode_mask") != nullptr) { variant.mode_mask = word(input, "mode_mask"); }
  if (input.get("mode") != nullptr) { variant.mode = word(input, "mode"); }
  for (const auto &value : array(input, "guards")) {
    const auto &guard = object(value);
    const auto *expected = guard.get("value");
    if (expected == nullptr) { invalid("guard value"); }
    const auto word_value = expected->kind() == llvm::json::Value::Null ? std::optional<Word>{}
                                                                        : word(number(*expected));
    variant.guards.push_back({.address = word(guard, "address"), .value = word_value});
  }
  variant.fixed_memory = input.get("memory") != nullptr;
  if (variant.fixed_memory) {
    for (const auto &value : array(input, "memory")) {
      const auto &region = object(value);
      const auto space = text(region, "space");
      if (space != "x" && space != "y") { invalid("external memory space"); }
      variant.memory.push_back({.space = space == "x" ? dsp56300::Space::x : dsp56300::Space::y,
                                .first = word(region, "first"),
                                .size = number(region, "words"),
                                .writable = boolean(region, "writable", true),
                                .tracked = boolean(region, "tracked", false),
                                .aliases_program = boolean(region, "aliases_program", false)});
    }
  }
}

bool compatible_admission(const Variant &left, const Variant &right) {
  if (((left.mode ^ right.mode) & left.mode_mask & right.mode_mask) != 0) { return false; }
  for (const auto &guard : left.guards) {
    const auto found =
        std::ranges::find(right.guards, guard.address, &vi::dsp::ProgramWord::address);
    if (found != right.guards.end() && found->value != guard.value) { return false; }
  }
  const auto layout = [](const dsp56300::RegionLayout &region) {
    return std::tuple{region.space,
                      region.first,
                      region.size,
                      region.writable,
                      region.tracked,
                      region.aliases_program};
  };
  return !left.fixed_memory || !right.fixed_memory ||
         std::ranges::is_permutation(left.memory, right.memory, {}, layout, layout);
}

void validate_admissions(const std::span<const Variant> variants) {
  for (std::size_t first = 0; first < variants.size(); ++first) {
    const auto &left = variants[first];
    for (const auto &right : variants.subspan(first + 1)) {
      if (!compatible_admission(left, right)) { continue; }
      const auto shared = std::ranges::find_first_of(left.resumes, right.resumes);
      if (shared != left.resumes.end()) {
        throw std::invalid_argument(
            std::format("Ambiguous DSP catalogue admission at P:${:06x}: variants {} and {}",
                        *shared,
                        left.id,
                        right.id));
      }
    }
  }
}

vi::dsp::Program
prepare(const Object &root, const vi::FirmwareUpdate &firmware, const vi::dsp::BootRom boot) {
  std::vector<Variant> owned;
  std::vector<dsp56300::VariantLayout> layouts;
  for (const auto &value : array(root, "variants")) {
    const auto &input = object(value);
    Stored stored;
    for (const auto &write : array(input, "writes")) {
      stored_write(stored, object(write), firmware, boot);
    }
    auto &variant = owned.emplace_back(number(input, "id"), stored.classify(array(input, "code")));
    admission(variant, input);
  }
  validate_admissions(owned);
  std::vector<dsp56300::CodeVariant> source;
  std::vector<vi::dsp::ProgramChoice> choices;
  for (const auto &variant : owned) {
    source.push_back({.id = variant.id,
                      .code = variant.code.code(),
                      .entries = variant.entries,
                      .failed_fetches = variant.failed_fetches});
    choices.push_back({.variant = variant.id,
                       .resumes = variant.resumes,
                       .words = variant.guards,
                       .mode_mask = variant.mode_mask,
                       .mode = variant.mode});
    if (variant.fixed_memory) { layouts.push_back({.id = variant.id, .memory = variant.memory}); }
  }
  std::vector<dsp56300::InterruptRequest> interrupts;
  for (const auto &value : array(root, "interrupts")) {
    const auto &request = object(value);
    const auto level = number(request, "level");
    if (level > 3) { invalid("interrupt level"); }
    interrupts.push_back(
        {.vector = word(request, "vector"), .level = static_cast<unsigned>(level)});
  }
  std::vector<dsp56300::CodeTransition> transitions;
  for (const auto &value : array(root, "transitions")) {
    const auto &transfer = object(value);
    transitions.push_back({.from = number(transfer, "from"),
                           .to = number(transfer, "to"),
                           .entry = word(transfer, "entry")});
  }
  return prepare_dsp_program({.variants = source}, choices, layouts, interrupts, transitions);
}
}

vi::dsp::Program prepare_dsp_catalogue(const std::string_view document,
                                       const vi::FirmwareUpdate &firmware,
                                       const vi::dsp::BootRom boot) {
  auto parsed = llvm::json::parse(document);
  if (!parsed) { throw std::invalid_argument(llvm::toString(parsed.takeError())); }
  const auto &root = object(*parsed);
  if (number(root, "format") != 1) { invalid("format"); }
  inputs(root, firmware, boot);
  return prepare(root, firmware, boot);
}
}
