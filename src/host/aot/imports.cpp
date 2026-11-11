// SPDX-License-Identifier: GPL-3.0-only
#include "aot/code.hpp"
#include "aot/llvm.hpp"

#include "llvm-c/Core.h"
#include "llvm-c/Types.h"

#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace blofeld::host::aot::detail {
Imports imports(const std::span<const Import> entries) {
  Imports result;
  for (const auto entry : entries) {
    if (entry.name.empty() || entry.name.find('\0') != std::string_view::npos ||
        entry.name.starts_with("__") || entry.name.starts_with("llvm.") || entry.abi.empty() ||
        entry.address == nullptr || !result.emplace(entry.name, entry).second) {
      throw std::invalid_argument("AOT has an invalid, reserved or duplicate native import");
    }
  }
  return result;
}

void validate_imports(LLVMModuleRef module, const Imports &imports) {
  for (const auto &[name, entry] : imports) {
    auto *function = LLVMGetNamedFunction(module, name.c_str());
    if (function == nullptr) {
      if (LLVMGetNamedGlobal(module, name.c_str()) != nullptr ||
          LLVMGetNamedGlobalAlias(module, name.data(), name.size()) != nullptr) {
        throw std::invalid_argument(std::string("AOT native import is not a function: ") + name);
      }
      continue;
    }
    if (LLVMIsDeclaration(function) == 0 || abi(function) != entry.abi) {
      throw std::invalid_argument(
          std::string("AOT native import overrides IR or has an incompatible ABI: ") + name);
    }
    validate_import_calls(function);
  }
}
}
