// SPDX-License-Identifier: GPL-3.0-only
#include "aot/code.hpp"
#include "aot/llvm.hpp"
#include "files/directory.hpp"
#include "files/read.hpp"

#include "llvm-c/BitWriter.h"
#include "llvm-c/Core.h"
#include "llvm-c/TargetMachine.h"
#include "llvm-c/Types.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/SHA256.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::host::aot::detail {
namespace {
using Buffer = Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer>;
using Digest = std::array<std::uint8_t, 32>;
constexpr std::size_t maximum_bytes = std::size_t{1} << 30U, header_bytes = 2 * Digest{}.size();

std::span<const std::uint8_t> bytes(const Buffer &buffer) {
  return {reinterpret_cast<const std::uint8_t *>(LLVMGetBufferStart(buffer.get())),
          LLVMGetBufferSize(buffer.get())};
}

Digest identity(LLVMModuleRef module,
                LLVMTargetMachineRef machine,
                const Exports &exports,
                const Imports &imports) {
  llvm::SHA256 hash;
  const auto text = [&](const std::string_view value) {
    hash.update(llvm::StringRef(value.data(), value.size()));
    hash.update(llvm::StringRef("\0", 1));
  };
  text(BLOFELD_AOT_PIPELINE);
  const Owner<char *, LLVMDisposeMessage> triple(LLVMGetTargetMachineTriple(machine));
  const Owner<char *, LLVMDisposeMessage> cpu(LLVMGetTargetMachineCPU(machine));
  const Owner<char *, LLVMDisposeMessage> features(LLVMGetTargetMachineFeatureString(machine));
  text(triple.get());
  text(cpu.get());
  text(features.get());
  text(std::to_string(exports.size()));
  for (const auto &[name, abi] : exports) {
    text(name);
    text(abi);
  }
  text("imports");
  text(std::to_string(imports.size()));
  for (const auto &[name, entry] : imports) {
    text(name);
    text(entry.abi);
  }
  const Buffer input(LLVMWriteBitcodeToMemoryBuffer(module));
  const auto input_bytes = bytes(input);
  hash.update(llvm::ArrayRef(input_bytes.data(), input_bytes.size()));
  return hash.final();
}

std::optional<std::vector<std::uint8_t>> load(const std::filesystem::path &root,
                                              const std::string &name) {
  const Directory locked(root, Directory::Lock::wait);
  if (!std::filesystem::exists(root / name)) { return {}; }
  return read_file(root / name, maximum_bytes);
}

void restore(Owner<LLVMModuleRef, LLVMDisposeModule> &module,
             const std::span<const std::uint8_t> record,
             const Digest &key,
             const Exports &exports) {
  if (record.size() < header_bytes || !std::ranges::equal(record.first(key.size()), key)) {
    throw std::invalid_argument("AOT cache has an invalid input identity");
  }
  const auto payload = record.subspan(header_bytes);
  const auto digest = llvm::SHA256::hash(llvm::ArrayRef(payload.data(), payload.size()));
  if (!std::ranges::equal(record.subspan(key.size(), key.size()), digest)) {
    throw std::invalid_argument("AOT cache has a corrupt bitcode payload");
  }
  std::vector<Export> expected;
  for (const auto &[name, abi] : exports) { expected.push_back({.name = name, .abi = abi}); }
  Exports actual;
  module.reset(parse(LLVMGetModuleContext(module.get()),
                     {.bitcode = std::as_bytes(payload), .exports = expected},
                     actual));
  validate_target(module.get());
}

void store(const std::filesystem::path &root,
           const std::string &name,
           const Digest &key,
           LLVMModuleRef module) {
  const Buffer output(LLVMWriteBitcodeToMemoryBuffer(module));
  const auto payload = bytes(output);
  const auto digest = llvm::SHA256::hash(llvm::ArrayRef(payload.data(), payload.size()));
  if (payload.size() > maximum_bytes - header_bytes) {
    throw std::length_error("AOT cache exceeds its artifact size limit");
  }
  std::vector<std::uint8_t> record;
  record.reserve(header_bytes + payload.size());
  record.insert(record.end(), key.begin(), key.end());
  record.insert(record.end(), digest.begin(), digest.end());
  record.insert(record.end(), payload.begin(), payload.end());
  const Directory locked(root, Directory::Lock::wait);
  locked.replace(name, record);
}
}

void optimize(Owner<LLVMModuleRef, LLVMDisposeModule> &module,
              LLVMTargetMachineRef machine,
              const Exports &exports,
              const Imports &imports,
              const std::filesystem::path &cache) {
  if (cache.empty()) {
    optimize(module.get(), machine, exports);
    return;
  }
  if (!cache.is_absolute()) { throw std::invalid_argument("AOT cache must be an absolute path"); }
  std::filesystem::create_directories(cache);
  const auto key = identity(module.get(), machine, exports, imports);
  const auto name = llvm::toHex(llvm::ArrayRef(key), true) + ".bc";
  const auto cached = load(cache, name);
  if (cached) {
    restore(module, *cached, key, exports);
  } else {
    optimize(module.get(), machine, exports);
  }
  validate_imports(module.get(), imports);
  auto active = imports;
  std::erase_if(active, [&](const auto &entry) {
    return LLVMGetNamedFunction(module.get(), entry.first.c_str()) == nullptr;
  });
  validate_symbols(module.get(), active);
  if (!cached) { store(cache, name, key, module.get()); }
}
}
