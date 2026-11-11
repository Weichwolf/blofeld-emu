// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "aot/code.hpp"

#include "llvm-c/Core.h"
#include "llvm-c/TargetMachine.h"
#include "llvm-c/Types.h"

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <type_traits>

namespace blofeld::host::aot::detail {
template <auto Dispose> struct Delete {
  void operator()(auto *value) const noexcept { Dispose(value); }
};
template <class Ref, auto Dispose>
using Owner = std::unique_ptr<std::remove_pointer_t<Ref>, Delete<Dispose>>;

using Exports = std::map<std::string, std::string>;
[[nodiscard]] std::string abi(LLVMValueRef value);
void validate_contracts(LLVMModuleRef expected, LLVMModuleRef incoming);
void validate_exports(LLVMModuleRef module, const Exports &exports);
void validate_target(LLVMModuleRef module);
void validate_symbols(LLVMModuleRef module);
[[nodiscard]] LLVMModuleRef parse(LLVMContextRef context, const Module &source, Exports &exports);
void merge(LLVMModuleRef destination, Owner<LLVMModuleRef, LLVMDisposeModule> module);
void optimize(LLVMModuleRef module, LLVMTargetMachineRef machine, const Exports &exports);
[[nodiscard]] LLVMTargetMachineRef target(LLVMModuleRef module);
[[nodiscard]] std::uint64_t text_bytes(LLVMMemoryBufferRef object);
void initialize();
}
