// SPDX-License-Identifier: GPL-3.0-only
#include "aot/artifact.hpp"
#include "aot/code.hpp"
#include "aot/llvm.hpp"
#include "files/directory.hpp"
#include "files/read.hpp"

#include "llvm-c/Core.h"
#include "llvm-c/TargetMachine.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/BLAKE3.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace blofeld::host::aot::detail {
namespace {
using Digest = std::array<std::uint8_t, 32>;
constexpr std::size_t maximum_bytes = std::size_t{1} << 30U, header_bytes = 2 * Digest{}.size();

std::optional<std::vector<std::uint8_t>> load(const std::filesystem::path &root,
                                              const std::string &name) {
  const Directory locked(root, Directory::Lock::wait);
  if (!std::filesystem::exists(root / name) ||
      std::filesystem::file_size(root / name) > maximum_bytes) {
    return {};
  }
  return read_file(root / name, maximum_bytes);
}

std::span<const std::uint8_t> payload(const std::span<const std::uint8_t> record,
                                      const Digest &key) {
  if (record.size() < header_bytes || !std::ranges::equal(record.first(key.size()), key)) {
    throw std::invalid_argument("AOT cache has an invalid input identity");
  }
  const auto result = record.subspan(header_bytes);
  const auto digest = llvm::BLAKE3::hash(llvm::ArrayRef(result.data(), result.size()));
  if (!std::ranges::equal(record.subspan(key.size(), key.size()), digest)) {
    throw std::invalid_argument("AOT cache has a corrupt payload");
  }
  return result;
}

void store(const std::filesystem::path &root,
           const std::string &name,
           const Digest &key,
           const std::span<const std::uint8_t> payload) {
  const auto digest = llvm::BLAKE3::hash(llvm::ArrayRef(payload.data(), payload.size()));
  if (payload.size() > maximum_bytes - header_bytes) {
    throw std::length_error("AOT cache exceeds its artifact size limit");
  }
  std::vector<std::uint8_t> record;
  record.reserve(header_bytes + payload.size());
  record.insert(record.end(), key.begin(), key.end());
  record.insert(record.end(), digest.begin(), digest.end());
  record.insert(record.end(), payload.begin(), payload.end());
  const Directory locked(root, Directory::Lock::wait);
  locked.replace(name, record, Directory::Durability::durable);
}

}

Artifact native_object(const std::filesystem::path &root,
                       const std::span<const Module> modules,
                       const Imports &imports) {
  if (root.empty()) { return {{}, {}, Artifact::Kind::native}; }
  llvm::BLAKE3 hash;
  const auto count = [&](const std::uint64_t size) {
    hash.update(llvm::StringRef(reinterpret_cast<const char *>(&size), sizeof(size)));
  };
  const auto bytes = [&](const llvm::StringRef input) {
    count(input.size());
    hash.update(input);
  };
  count(modules.size());
  for (const auto &module : modules) {
    bytes("module");
    bytes({reinterpret_cast<const char *>(module.bitcode.data()), module.bitcode.size()});
    count(module.exports.size());
    for (const auto &entry : module.exports) {
      bytes({entry.name.data(), entry.name.size()});
      bytes({entry.abi.data(), entry.abi.size()});
    }
  }
  count(imports.size());
  for (const auto &[name, entry] : imports) {
    bytes(name);
    bytes({entry.abi.data(), entry.abi.size()});
  }
  const auto key = hash.final();
  return {root, std::as_bytes(std::span(key)), Artifact::Kind::native};
}
}

namespace blofeld::host::aot {
Artifact::Artifact(std::filesystem::path root,
                   const std::span<const std::byte> inputs,
                   const Kind kind)
    : root_(std::move(root)) {
  if (root_.empty()) { return; }
  if (!root_.is_absolute()) { throw std::invalid_argument("AOT cache must be an absolute path"); }
  llvm::BLAKE3 hash;
  const auto text = [&](const std::string_view value) {
    hash.update(llvm::StringRef(value.data(), value.size()));
    hash.update(llvm::StringRef("\0", 1));
  };
  text(kind == Kind::native ? "native-object-2" : "preparation-artifact-2");
  text(BLOFELD_AOT_PIPELINE);
  const detail::Owner<char *, LLVMDisposeMessage> triple(LLVMGetDefaultTargetTriple());
  const detail::Owner<char *, LLVMDisposeMessage> cpu(LLVMGetHostCPUName());
  const detail::Owner<char *, LLVMDisposeMessage> features(LLVMGetHostCPUFeatures());
  text(triple.get());
  text(cpu.get());
  text(features.get());
  hash.update(llvm::ArrayRef(reinterpret_cast<const std::uint8_t *>(inputs.data()), inputs.size()));
  key_ = hash.final();
  name_ = llvm::toHex(llvm::ArrayRef(key_), true) + (kind == Kind::native ? ".o" : ".artifact");
  std::filesystem::create_directories(root_);
}

std::optional<std::vector<std::byte>> Artifact::load() const {
  if (root_.empty()) { return {}; }
  const auto record = detail::load(root_, name_);
  if (!record) { return {}; }
  try {
    const auto bytes = std::as_bytes(detail::payload(*record, key_));
    return std::vector(bytes.begin(), bytes.end());
  } catch (const std::invalid_argument &) { return {}; }
}

void Artifact::store(const std::span<const std::byte> bytes) const {
  if (root_.empty()) { return; }
  detail::store(
      root_, name_, key_, {reinterpret_cast<const std::uint8_t *>(bytes.data()), bytes.size()});
}
}
