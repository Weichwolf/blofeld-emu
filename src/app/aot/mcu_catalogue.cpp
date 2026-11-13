// SPDX-License-Identifier: GPL-3.0-only
#include "aot/mcu.hpp"
#include "mcu/program.hpp"
#include "storage/firmware.hpp"

#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/JSON.h"
#include "llvm/Support/SHA256.h"

#include <algorithm>
#include <bitset>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <format>
#include <set>
#include <span>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace blofeld::app::aot {
namespace {
using Object = llvm::json::Object;

[[noreturn]] void invalid(const std::string_view field) {
  throw std::invalid_argument(std::format("Invalid MCU catalogue field: {}", field));
}

const Object &object(const llvm::json::Value &value) {
  const auto *result = value.getAsObject();
  if (result == nullptr) { invalid("object"); }
  return *result;
}

const llvm::json::Array &array(const Object &value, const llvm::StringRef field) {
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

std::string_view text(const Object &value, const llvm::StringRef field) {
  const auto result = value.getString(field);
  if (!result) { invalid(field.str()); }
  return *result;
}

cpu12::Address address(const std::uint64_t value) {
  if (value >= cpu12::address_bytes) { invalid("16-bit address"); }
  return static_cast<cpu12::Address>(value);
}

void identity(const std::span<const std::uint8_t> bytes, const std::string_view expected) {
  if (llvm::toHex(llvm::SHA256::hash({bytes.data(), bytes.size()}), true) != expected) {
    throw std::invalid_argument("MCU catalogue source SHA256 mismatch");
  }
}

std::span<const std::uint8_t> source(const std::span<const McuSource> sources,
                                     const std::string_view name) {
  const auto found = std::ranges::find(sources, name, &McuSource::name);
  if (found == sources.end()) { invalid("missing named source"); }
  return found->bytes;
}

void inputs(const Object &root,
            const vi::FirmwareUpdate &firmware,
            const std::span<const McuSource> sources) {
  const auto *input = root.getObject("inputs");
  if (input == nullptr) { invalid("inputs"); }
  identity(firmware.mcu_flash, text(*input, "mcu"));
  std::set<std::string_view> supplied;
  std::set<std::string_view> declared;
  for (const auto &value : sources) {
    if (value.name.empty() || value.name == "mcu" || !supplied.insert(value.name).second) {
      invalid("source name");
    }
  }
  for (const auto &value : array(*input, "blobs")) {
    const auto &blob = object(value);
    const auto name = text(blob, "name");
    if (name.empty() || name == "mcu" || !declared.insert(name).second) { invalid("source name"); }
    identity(source(sources, name), text(blob, "sha256"));
  }
  if (declared != supplied) { invalid("source identities"); }
}

struct Variant {
  std::uint64_t id;
  std::vector<cpu12::CodeSpan> code;
  std::vector<cpu12::Address> entries;
  std::bitset<cpu12::address_bytes> bytes;
};

Variant variant(const Object &input,
                const vi::FirmwareUpdate &firmware,
                const std::span<const McuSource> sources) {
  Variant result{.id = number(input, "id"), .code = {}, .entries = {}, .bytes = {}};
  const auto name = text(input, "source");
  const auto bytes =
      name == "mcu" ? std::span<const std::uint8_t>(firmware.mcu_flash) : source(sources, name);
  const auto base = address(number(input, "address"));
  const auto offset = number(input, "byte_offset");
  const auto size = number(input, "bytes");
  if (offset > bytes.size() || size == 0 || size > bytes.size() - offset ||
      size > cpu12::address_bytes - base) {
    invalid("source extent");
  }
  std::set<cpu12::Address> starts;
  for (const auto &value : array(input, "instructions")) {
    const auto *pair = value.getAsArray();
    if (pair == nullptr || pair->size() != 2) { invalid("instruction range"); }
    const auto pc = address(number((*pair)[0]));
    const auto count = number((*pair)[1]);
    if (pc < base || std::uint64_t{pc} - base >= size || count == 0 ||
        count > cpu12::maximum_capture_bytes || count > size - (pc - base) ||
        !starts.insert(pc).second) {
      invalid("instruction extent");
    }
    result.code.push_back({.address = pc, .words = bytes.subspan(offset + pc - base, count)});
    result.entries.push_back(pc);
    for (std::uint64_t index = 0; index < count; ++index) { result.bytes.set(pc + index); }
  }
  if (result.entries.empty()) { invalid("instruction entries"); }
  return result;
}
}

vi::mcu::Program prepare_mcu_catalogue(const std::string_view document,
                                       const vi::FirmwareUpdate &firmware,
                                       const std::span<const McuSource> sources) {
  auto parsed = llvm::json::parse(document);
  if (!parsed) { throw std::invalid_argument(llvm::toString(parsed.takeError())); }
  const auto &root = object(*parsed);
  if (number(root, "format") != 1) { invalid("format"); }
  inputs(root, firmware, sources);
  std::vector<Variant> owned;
  std::set<std::uint64_t> ids;
  for (const auto &value : array(root, "variants")) {
    owned.push_back(variant(object(value), firmware, sources));
    if (!ids.insert(owned.back().id).second) { invalid("duplicate variant ID"); }
  }
  if (owned.empty()) { invalid("variants"); }
  const auto cross = root.getBoolean("cross_domain_entries");
  if (!cross || !*cross) { invalid("explicit cross_domain_entries contract"); }
  std::vector<cpu12::CodeVariant> variants;
  std::vector<cpu12::CodeTransition> transitions;
  for (const auto &from : owned) {
    variants.push_back({.id = from.id, .code = from.code, .entries = from.entries});
    for (const auto &to : owned) {
      if (from.id == to.id) { continue; }
      for (const auto entry : to.entries) {
        if (!from.bytes[entry]) {
          transitions.push_back({.from = from.id, .to = to.id, .entry = entry});
        }
      }
    }
  }
  return prepare_mcu_program({.variants = variants}, transitions);
}
}
