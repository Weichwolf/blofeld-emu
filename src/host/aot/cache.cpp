// SPDX-License-Identifier: GPL-3.0-only
#include "aot/artifact.hpp"
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
#include "llvm/ExecutionEngine/ObjectCache.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/SHA256.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace blofeld::host::aot::detail {
namespace {
using Buffer = Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer>;
using Digest = std::array<std::uint8_t, 32>;
constexpr std::size_t maximum_bytes = std::size_t{1} << 30U, header_bytes = 2 * Digest{}.size();

bool omit_dsp_frame_pointers(LLVMModuleRef module) {
#if defined(__linux__) && defined(__x86_64__) && defined(NDEBUG) &&                                \
    !__has_feature(address_sanitizer) && !__has_feature(thread_sanitizer) &&                       \
    !__has_feature(undefined_behavior_sanitizer)
  bool changed = false;
  for (auto &function : *llvm::unwrap(module)) {
    if (function.isDeclaration() || !function.hasFnAttribute("dsp56300-image-part") ||
        function.getFnAttribute("frame-pointer").getValueAsString() == "none") {
      continue;
    }
    // Keep RBP available for the allocator. Fault attribution uses the saved PC.
    function.addFnAttr("frame-pointer", "none");
    changed = true;
  }
  return changed;
#else
  static_cast<void>(module);
  return false;
#endif
}

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
  const auto digest = llvm::SHA256::hash(llvm::ArrayRef(result.data(), result.size()));
  if (!std::ranges::equal(record.subspan(key.size(), key.size()), digest)) {
    throw std::invalid_argument("AOT cache has a corrupt payload");
  }
  return result;
}

void restore(Owner<LLVMModuleRef, LLVMDisposeModule> &module,
             const std::span<const std::uint8_t> record,
             const Digest &key,
             const Exports &exports) {
  const auto bitcode = payload(record, key);
  std::vector<Export> expected;
  for (const auto &[name, abi] : exports) { expected.push_back({.name = name, .abi = abi}); }
  Exports actual;
  Owner<LLVMModuleRef, LLVMDisposeModule> restored(
      parse(LLVMGetModuleContext(module.get()),
            {.bitcode = std::as_bytes(bitcode), .exports = expected},
            actual));
  validate_target(restored.get());
  std::size_t size = 0;
  const auto *identifier = LLVMGetModuleIdentifier(module.get(), &size);
  LLVMSetModuleIdentifier(restored.get(), identifier, size);
  module = std::move(restored);
}

void store(const std::filesystem::path &root,
           const std::string &name,
           const Digest &key,
           const std::span<const std::uint8_t> payload) {
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
  locked.replace(name, record, Directory::Durability::durable);
}

class Objects final : public llvm::ObjectCache {
public:
  Objects(std::filesystem::path root, const Digest key, const llvm::Module &module)
      : root_(std::move(root)),
        key_(key),
        name_(llvm::toHex(llvm::ArrayRef(key), true) + ".o"),
        module_(module.getModuleIdentifier()) {}

  void notifyObjectCompiled(const llvm::Module *module,
                            const llvm::MemoryBufferRef object) override {
    if (module->getModuleIdentifier() != module_) { return; }
    const auto data = object.getBuffer();
    store(root_, name_, key_, {reinterpret_cast<const std::uint8_t *>(data.data()), data.size()});
  }

  std::unique_ptr<llvm::MemoryBuffer> getObject(const llvm::Module *module) override {
    if (module->getModuleIdentifier() != module_) { return {}; }
    const auto record = load(root_, name_);
    if (!record) { return {}; }
    try {
      const auto data = payload(*record, key_);
      auto object = llvm::MemoryBuffer::getMemBufferCopy(
          {reinterpret_cast<const char *>(data.data()), data.size()}, name_);
      static_cast<void>(text_bytes(llvm::wrap(object.get())));
      return object;
    } catch (const std::invalid_argument &) { return {}; } catch (const std::runtime_error &) {
      return {};
    }
  }

private:
  const std::filesystem::path root_;
  const Digest key_;
  const std::string name_, module_;
};
}

std::unique_ptr<llvm::ObjectCache> optimize(Owner<LLVMModuleRef, LLVMDisposeModule> &module,
                                            LLVMTargetMachineRef machine,
                                            const Exports &exports,
                                            const Imports &imports,
                                            const std::filesystem::path &cache) {
  if (cache.empty()) {
    optimize(module.get(), machine, exports);
    static_cast<void>(omit_dsp_frame_pointers(module.get()));
    return {};
  }
  if (!cache.is_absolute()) { throw std::invalid_argument("AOT cache must be an absolute path"); }
  std::filesystem::create_directories(cache);
  const auto key = identity(module.get(), machine, exports, imports);
  const auto name = llvm::toHex(llvm::ArrayRef(key), true) + ".bc";
  auto cached = load(cache, name);
  if (cached) {
    try {
      restore(module, *cached, key, exports);
    } catch (const std::invalid_argument &) { cached.reset(); }
  }
  if (!cached) { optimize(module.get(), machine, exports); }
  validate_imports(module.get(), imports);
  auto active = imports;
  std::erase_if(active, [&](const auto &entry) {
    return LLVMGetNamedFunction(module.get(), entry.first.c_str()) == nullptr;
  });
  validate_symbols(module.get(), active);
  if (!cached) {
    const Buffer output(LLVMWriteBitcodeToMemoryBuffer(module.get()));
    store(cache, name, key, bytes(output));
  }
  auto object_key = key;
  if (omit_dsp_frame_pointers(module.get())) {
    llvm::SHA256 hash;
    hash.update(object_key);
    hash.update("generated-dsp-frame-pointer-none");
    object_key = hash.final();
  }
  return std::make_unique<Objects>(cache, object_key, *llvm::unwrap(module.get()));
}
}

namespace blofeld::host::aot {
Artifact::Artifact(std::filesystem::path root, const std::span<const std::byte> inputs)
    : root_(std::move(root)) {
  if (root_.empty()) { return; }
  if (!root_.is_absolute()) { throw std::invalid_argument("AOT cache must be an absolute path"); }
  llvm::SHA256 hash;
  const auto text = [&](const std::string_view value) {
    hash.update(llvm::StringRef(value.data(), value.size()));
    hash.update(llvm::StringRef("\0", 1));
  };
  text("preparation-artifact-1");
  text(BLOFELD_AOT_PIPELINE);
  const detail::Owner<char *, LLVMDisposeMessage> triple(LLVMGetDefaultTargetTriple());
  const detail::Owner<char *, LLVMDisposeMessage> cpu(LLVMGetHostCPUName());
  const detail::Owner<char *, LLVMDisposeMessage> features(LLVMGetHostCPUFeatures());
  text(triple.get());
  text(cpu.get());
  text(features.get());
  hash.update(llvm::ArrayRef(reinterpret_cast<const std::uint8_t *>(inputs.data()), inputs.size()));
  key_ = hash.final();
  name_ = llvm::toHex(llvm::ArrayRef(key_), true) + ".artifact";
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
