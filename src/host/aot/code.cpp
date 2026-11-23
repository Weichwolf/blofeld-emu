// SPDX-License-Identifier: GPL-3.0-only
#include "aot/code.hpp"

#include "aot/artifact.hpp"
#include "aot/link_graph.hpp"
#include "aot/llvm.hpp"
#if defined(__APPLE__) && defined(__aarch64__)
#include "aot/jit_memory_darwin.hpp"
#endif
#include "native_sites.hpp"

#include "llvm-c/Core.h"
#include "llvm-c/Error.h"
#include "llvm-c/LLJIT.h"
#include "llvm-c/Object.h"
#include "llvm-c/Orc.h"
#include "llvm-c/TargetMachine.h"
#include "llvm-c/Types.h"
#include "llvm/ExecutionEngine/ObjectCache.h"
#include "llvm/ExecutionEngine/Orc/CompileUtils.h"
#include "llvm/ExecutionEngine/Orc/IRCompileLayer.h"
#include "llvm/ExecutionEngine/Orc/LLJIT.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/MemoryBuffer.h"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace blofeld::host::aot {
namespace {
using detail::Owner;
using Buffer = Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer>;
constexpr const char *module_name = "blofeld-native";

class Capture final : public llvm::ObjectCache {
public:
  void notifyObjectCompiled(const llvm::Module *module,
                            const llvm::MemoryBufferRef object) override {
    if (module->getModuleIdentifier() != module_name) { return; }
    const auto data = object.getBuffer();
    const auto bytes = std::as_bytes(std::span(data.data(), data.size()));
    data_.assign(bytes.begin(), bytes.end());
  }

  std::unique_ptr<llvm::MemoryBuffer> getObject(const llvm::Module * /*module*/) override {
    return {};
  }

  void publish(const Artifact &artifact) {
    if (data_.empty()) { throw std::runtime_error("AOT did not emit its native object"); }
    const auto data = std::move(data_);
    artifact.store(data);
  }

private:
  std::vector<std::byte> data_;
};

Buffer load_object(const Artifact &artifact) {
  const auto record = artifact.load();
  if (!record || record->empty()) { return {}; }
  auto object = Buffer(LLVMCreateMemoryBufferWithMemoryRangeCopy(
      reinterpret_cast<const char *>(record->data()), record->size(), module_name));
  try {
    if (detail::text_bytes(object.get()) != 0) { return object; }
  } catch (const std::runtime_error &) { return {}; }
  return {};
}

void checked(LLVMErrorRef error) {
  if (error == nullptr) { return; }
  const Owner<char *, LLVMDisposeErrorMessage> message(LLVMGetErrorMessage(error));
  throw std::runtime_error(message.get());
}

void dispose(LLVMOrcLLJITRef jit) noexcept {
  if (auto *error = LLVMOrcDisposeLLJIT(jit)) {
    const Owner<char *, LLVMDisposeErrorMessage> message(LLVMGetErrorMessage(error));
    std::cerr << message.get() << '\n';
    std::terminate();
  }
}

llvm::orc::LLJIT &native(LLVMOrcLLJITRef jit) {
  return *reinterpret_cast<llvm::orc::LLJIT *>(jit);
}

void platform(LLVMOrcLLJITBuilderRef builder,
              llvm::ObjectCache *cache,
              llvm_native_sites::SourceMap &source,
              Code::ExecutableMemory memory) {
  auto &native_builder = *reinterpret_cast<llvm::orc::LLJITBuilder *>(builder);
  if (memory == Code::ExecutableMemory::jit) {
#if defined(__APPLE__) && defined(__aarch64__)
    detail::use_jit_memory(native_builder);
#else
    throw std::invalid_argument("JIT memory requires macOS ARM64");
#endif
  }
  native_builder.setNumCompileThreads(0).setSupportConcurrentCompilation(false);
  native_builder.setPrePlatformSetup([cache, &source](llvm::orc::LLJIT &jit) {
    source.attach(jit);
    // LLVM19 selects SimpleCompiler when concurrent compilation is disabled.
    static_cast<llvm::orc::SimpleCompiler &>(jit.getIRCompileLayer().getCompiler())
        .setObjectCache(cache);
    return llvm::Error::success();
  });
}

void bind_imports(LLVMOrcLLJITRef jit, const detail::Imports &imports) {
  if (imports.empty()) { return; }
  std::vector<LLVMOrcCSymbolMapPair> symbols;
  symbols.reserve(imports.size());
  constexpr auto flags = LLVMJITSymbolGenericFlagsExported | LLVMJITSymbolGenericFlagsCallable;
  for (const auto &[name, entry] : imports) {
    symbols.push_back({.Name = LLVMOrcLLJITMangleAndIntern(jit, name.c_str()),
                       .Sym = {.Address = std::bit_cast<LLVMOrcExecutorAddress>(entry.address),
                               .Flags = {.GenericFlags = flags, .TargetFlags = 0}}});
  }
  Owner<LLVMOrcMaterializationUnitRef, LLVMOrcDisposeMaterializationUnit> unit(
      LLVMOrcAbsoluteSymbols(symbols.data(), symbols.size()));
  auto *pending = unit.get();
  auto *error = LLVMOrcJITDylibDefine(LLVMOrcLLJITGetMainJITDylib(jit), unit.release());
  if (error != nullptr) { unit.reset(pending); }
  checked(error);
}
}

class Code::Impl {
public:
  enum class Phase : std::uint8_t { assembling, initialized };

  ~Impl() {
    if (phase == Phase::initialized) {
      auto &owner = native(jit.get());
      if (auto error = owner.deinitialize(owner.getMainJITDylib())) {
        std::cerr << llvm::toString(std::move(error)) << '\n';
        std::terminate();
      }
    }
  }

  void open(LLVMTargetMachineRef machine,
            const detail::Imports &bindings,
            const ExecutableMemory memory,
            const bool capture) {
    Owner<LLVMOrcLLJITBuilderRef, LLVMOrcDisposeLLJITBuilder> builder(LLVMOrcCreateLLJITBuilder());
    platform(builder.get(), capture ? &object : nullptr, source, memory);
    if (machine != nullptr) {
      LLVMOrcLLJITBuilderSetJITTargetMachineBuilder(
          builder.get(), LLVMOrcJITTargetMachineBuilderCreateFromTargetMachine(machine));
    }
    LLVMOrcLLJITRef raw = nullptr;
    auto *failure = LLVMOrcCreateLLJIT(&raw, builder.release());
    jit.reset(raw);
    checked(failure);
    detail::named_dependencies(native(raw));
    bind_imports(raw, bindings);
  }

  void compile(const std::span<const Module> modules,
               const detail::Imports &bindings,
               const ExecutableMemory memory,
               const bool capture) {
    const Owner<LLVMOrcThreadSafeContextRef, LLVMOrcDisposeThreadSafeContext> context(
        LLVMOrcCreateNewThreadSafeContext());
    auto *llvm_context = LLVMOrcThreadSafeContextGetContext(context.get());
    Owner<LLVMModuleRef, LLVMDisposeModule> module(
        LLVMModuleCreateWithNameInContext(module_name, llvm_context));
    detail::Exports exports;
    std::vector<Owner<LLVMModuleRef, LLVMDisposeModule>> inputs;
    for (const auto &input : modules) {
      inputs.emplace_back(detail::parse(llvm_context, input, exports));
      detail::validate_target(inputs.back().get());
      detail::validate_imports(inputs.back().get(), bindings);
    }
    if (exports.empty()) { throw std::invalid_argument("AOT requires declared native exports"); }
    for (const auto &expected : inputs) {
      for (const auto &incoming : inputs) {
        if (expected != incoming) { detail::validate_contracts(expected.get(), incoming.get()); }
      }
    }
    for (auto &input : inputs) { detail::merge(module.get(), std::move(input)); }
    detail::validate_exports(module.get(), exports);
    detail::validate_imports(module.get(), bindings);
    detail::validate_symbols(module.get(), bindings);
    Owner<LLVMTargetMachineRef, LLVMDisposeTargetMachine> machine(detail::target(module.get()));
    detail::optimize(module.get(), machine.get(), exports);
    detail::validate_imports(module.get(), bindings);
    auto active = bindings;
    std::erase_if(active, [&](const auto &entry) {
      return LLVMGetNamedFunction(module.get(), entry.first.c_str()) == nullptr;
    });
    detail::validate_symbols(module.get(), active);
    detail::select_dsp_frame_pointers(module.get());
    open(machine.release(), bindings, memory, capture);
    auto *prepared = LLVMOrcCreateNewThreadSafeModule(module.release(), context.get());
    checked(
        LLVMOrcLLJITAddLLVMIRModule(jit.get(), LLVMOrcLLJITGetMainJITDylib(jit.get()), prepared));
  }

  void resolve(const std::span<const Module> modules) {
    for (const auto &input : modules) {
      for (const auto entry : input.exports) {
        LLVMOrcExecutorAddress address = 0;
        const std::string name(entry.name);
        checked(LLVMOrcLLJITLookup(jit.get(), &address, name.c_str()));
        symbols.emplace(name, std::bit_cast<const void *>(address));
      }
    }
    auto &owner = native(jit.get());
    if (auto initialization = owner.initialize(owner.getMainJITDylib())) {
      throw std::runtime_error(llvm::toString(std::move(initialization)));
    }
    phase = Phase::initialized;
  }

  Capture object;
  llvm_native_sites::SourceMap source;
  Owner<LLVMOrcLLJITRef, dispose> jit;
  std::map<std::string, const void *, std::less<>> symbols;
  Phase phase = Phase::assembling;
};

Code::Code(const std::span<const Module> modules,
           const std::span<const Import> imports,
           const std::filesystem::path &cache,
           const ExecutableMemory memory,
           const Preparation preparation)
    : impl_(std::make_unique<Impl>()) {
  const auto bindings = detail::imports(imports);
  detail::initialize();
  const auto artifact = detail::native_object(cache, modules, bindings);
  auto object = load_object(artifact);
  const bool compiled = !object;
  if (object) {
    impl_->open(nullptr, bindings, memory, false);
    checked(LLVMOrcLLJITAddObjectFile(
        impl_->jit.get(), LLVMOrcLLJITGetMainJITDylib(impl_->jit.get()), object.release()));
  } else {
    if (preparation == Preparation::load_only) {
      throw std::runtime_error(
          "Missing or corrupt native AOT object. Run blofeld-headless --prepare.");
    }
    impl_->compile(modules, bindings, memory, !cache.empty());
  }
  impl_->resolve(modules);
  if (compiled && !cache.empty()) { impl_->object.publish(artifact); }
}

Code::~Code() = default;

const void *Code::address(const std::string_view name) const {
  const auto found = impl_->symbols.find(name);
  if (found == impl_->symbols.end()) {
    throw std::invalid_argument("Unknown AOT export: " + std::string(name));
  }
  return found->second;
}

Code::FaultMap Code::fault_map() const noexcept {
  return impl_->source.map();
}

std::uint64_t Code::native_bytes() const noexcept {
  std::uint64_t bytes = 0;
  for (const auto &range : impl_->source.text) { bytes += range.size; }
  return bytes;
}
}

namespace blofeld::host::aot::detail {
std::uint64_t text_bytes(LLVMMemoryBufferRef object) {
  char *raw_error = nullptr;
  const Owner<LLVMBinaryRef, LLVMDisposeBinary> binary(
      LLVMCreateBinary(object, nullptr, &raw_error));
  const Owner<char *, LLVMDisposeMessage> error(raw_error);
  if (!binary) { throw std::runtime_error(error.get()); }
  const Owner<LLVMSectionIteratorRef, LLVMDisposeSectionIterator> sections(
      LLVMObjectFileCopySectionIterator(binary.get()));
  std::uint64_t bytes = 0;
  if (sections) {
    while (LLVMObjectFileIsSectionIteratorAtEnd(binary.get(), sections.get()) == 0) {
      const auto *raw_name = LLVMGetSectionName(sections.get());
      const std::string_view name(raw_name == nullptr ? "" : raw_name);
      if (name == "__text" || name == ".text" || name.starts_with(".text.") || name == ".ltext" ||
          name.starts_with(".ltext.")) {
        bytes += LLVMGetSectionSize(sections.get());
      }
      LLVMMoveToNextSection(sections.get());
    }
  }
  return bytes;
}
}
