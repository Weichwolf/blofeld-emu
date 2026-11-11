// SPDX-License-Identifier: GPL-3.0-only
#include "aot/code.hpp"

#include "aot/llvm.hpp"

#include "llvm-c/Core.h"
#include "llvm-c/Error.h"
#include "llvm-c/LLJIT.h"
#include "llvm-c/Object.h"
#include "llvm-c/Orc.h"
#include "llvm-c/TargetMachine.h"
#include "llvm-c/Types.h"
#include "llvm/ExecutionEngine/Orc/LLJIT.h"
#include "llvm/Support/Error.h"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
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

LLVMErrorRef measure(void *owner, LLVMMemoryBufferRef *buffer) noexcept {
  try {
    *static_cast<std::uint64_t *>(owner) += detail::text_bytes(*buffer);
    return nullptr;
  } catch (const std::exception &error) {
    LLVMDisposeMemoryBuffer(*buffer);
    *buffer = nullptr;
    return LLVMCreateStringError(error.what());
  } catch (...) {
    LLVMDisposeMemoryBuffer(*buffer);
    *buffer = nullptr;
    return LLVMCreateStringError("Cannot measure native AOT object");
  }
}

void platform(LLVMOrcLLJITBuilderRef builder, std::uint64_t &bytes) {
  auto &native_builder = *reinterpret_cast<llvm::orc::LLJITBuilder *>(builder);
  native_builder.setPrePlatformSetup([&bytes](llvm::orc::LLJIT &jit) {
    auto *handle = reinterpret_cast<LLVMOrcLLJITRef>(&jit);
    LLVMOrcObjectTransformLayerSetTransform(
        LLVMOrcLLJITGetObjTransformLayer(handle), measure, &bytes);
    return llvm::Error::success();
  });
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

  std::uint64_t bytes = 0;
  Owner<LLVMOrcLLJITRef, dispose> jit;
  std::map<std::string, const void *, std::less<>> symbols;
  Phase phase = Phase::assembling;
};

Code::Code(const std::span<const Module> modules) : impl_(std::make_unique<Impl>()) {
  detail::initialize();
  const Owner<LLVMOrcThreadSafeContextRef, LLVMOrcDisposeThreadSafeContext> context(
      LLVMOrcCreateNewThreadSafeContext());
  auto *llvm_context = LLVMOrcThreadSafeContextGetContext(context.get());
  Owner<LLVMModuleRef, LLVMDisposeModule> module(
      LLVMModuleCreateWithNameInContext("blofeld-native", llvm_context));
  detail::Exports exports;
  std::vector<Owner<LLVMModuleRef, LLVMDisposeModule>> inputs;
  for (const auto &input : modules) {
    inputs.emplace_back(detail::parse(llvm_context, input, exports));
    detail::validate_target(inputs.back().get());
  }
  if (exports.empty()) { throw std::invalid_argument("AOT requires declared native exports"); }
  for (const auto &expected : inputs) {
    for (const auto &incoming : inputs) {
      if (expected != incoming) { detail::validate_contracts(expected.get(), incoming.get()); }
    }
  }
  for (auto &input : inputs) { detail::merge(module.get(), std::move(input)); }
  detail::validate_exports(module.get(), exports);
  detail::validate_symbols(module.get());
  Owner<LLVMTargetMachineRef, LLVMDisposeTargetMachine> machine(detail::target(module.get()));
  detail::optimize(module.get(), machine.get(), exports);
  Owner<LLVMOrcLLJITBuilderRef, LLVMOrcDisposeLLJITBuilder> builder(LLVMOrcCreateLLJITBuilder());
  platform(builder.get(), impl_->bytes);
  LLVMOrcLLJITBuilderSetJITTargetMachineBuilder(
      builder.get(), LLVMOrcJITTargetMachineBuilderCreateFromTargetMachine(machine.release()));
  LLVMOrcLLJITRef raw_jit = nullptr;
  auto *failure = LLVMOrcCreateLLJIT(&raw_jit, builder.release());
  impl_->jit.reset(raw_jit);
  checked(failure);
  auto *prepared = LLVMOrcCreateNewThreadSafeModule(module.release(), context.get());
  checked(LLVMOrcLLJITAddLLVMIRModule(raw_jit, LLVMOrcLLJITGetMainJITDylib(raw_jit), prepared));
  for (const auto &[name, contract] : exports) {
    static_cast<void>(contract);
    LLVMOrcExecutorAddress address = 0;
    checked(LLVMOrcLLJITLookup(raw_jit, &address, name.c_str()));
    impl_->symbols.emplace(name, std::bit_cast<const void *>(address));
  }
  auto &jit = native(raw_jit);
  if (auto initialization = jit.initialize(jit.getMainJITDylib())) {
    throw std::runtime_error(llvm::toString(std::move(initialization)));
  }
  impl_->phase = Impl::Phase::initialized;
}

Code::~Code() = default;

const void *Code::address(const std::string_view name) const {
  const auto found = impl_->symbols.find(name);
  if (found == impl_->symbols.end()) {
    throw std::invalid_argument("Unknown AOT export: " + std::string(name));
  }
  return found->second;
}

std::uint64_t Code::native_bytes() const noexcept {
  return impl_->bytes;
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
