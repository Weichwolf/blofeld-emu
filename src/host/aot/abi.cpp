// SPDX-License-Identifier: GPL-3.0-only
#include "aot/attributes.hpp"
#include "aot/llvm.hpp"

#include "llvm-c/Core.h"
#include "llvm-c/Types.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Value.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/raw_ostream.h"

#include <stdexcept>
#include <string>

namespace blofeld::host::aot::detail {
std::string abi(LLVMValueRef value) {
  const auto &function = *llvm::cast<llvm::Function>(llvm::unwrap(value));
  std::string result;
  llvm::raw_string_ostream text(result);
  text << "llvm19;cc=" << function.getCallingConv() << ";type=";
  function.getFunctionType()->print(text);
  for (unsigned index = 0; index <= function.arg_size(); ++index) {
    for (const auto kind : abi_attributes) {
      const auto attribute = function.getAttributes().getAttributeAtIndex(index, kind);
      if (attribute.isValid()) { text << ';' << index << ':' << attribute.getAsString(); }
    }
  }
  return result;
}

void validate_exports(LLVMModuleRef module, const Exports &exports) {
  for (const auto &[name, expected] : exports) {
    auto *function = LLVMGetNamedFunction(module, name.c_str());
    if (function == nullptr || LLVMIsDeclaration(function) != 0 || abi(function) != expected) {
      throw std::invalid_argument("AOT export is missing or has an incompatible ABI: " + name);
    }
  }
}
}
