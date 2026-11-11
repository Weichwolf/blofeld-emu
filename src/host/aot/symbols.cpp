// SPDX-License-Identifier: GPL-3.0-only
#include "aot/llvm.hpp"

#include "llvm-c/Core.h"
#include "llvm-c/Support.h"
#include "llvm-c/Types.h"

#include <cstddef>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace blofeld::host::aot::detail {
namespace {
void symbol(LLVMValueRef value) {
  if (LLVMIsDeclaration(value) == 0 || LLVMGetFirstUse(value) == nullptr ||
      (LLVMIsAFunction(value) != nullptr && LLVMGetIntrinsicID(value) != 0)) {
    return;
  }
  std::size_t size = 0;
  const auto *name = LLVMGetValueName2(value, &size);
  const std::string text(name, size);
  // The native ORC platform owns this DSO handle and its destructor registrations.
  if (text == "__dso_handle") { return; }
  if (LLVMSearchForAddressOfSymbol(text.c_str()) == nullptr) {
    throw std::invalid_argument("AOT unresolved native symbol: " + text);
  }
}
}

void validate_symbols(LLVMModuleRef module) {
  std::vector<LLVMValueRef> pending;
  for (auto *function = LLVMGetFirstFunction(module); function != nullptr;
       function = LLVMGetNextFunction(function)) {
    symbol(function);
    for (auto *block = LLVMGetFirstBasicBlock(function); block != nullptr;
         block = LLVMGetNextBasicBlock(block)) {
      for (auto *instruction = LLVMGetFirstInstruction(block); instruction != nullptr;
           instruction = LLVMGetNextInstruction(instruction)) {
        pending.push_back(instruction);
      }
    }
  }
  for (auto *global = LLVMGetFirstGlobal(module); global != nullptr;
       global = LLVMGetNextGlobal(global)) {
    symbol(global);
    pending.push_back(global);
  }
  std::set<LLVMValueRef> visited;
  while (!pending.empty()) {
    auto *value = pending.back();
    pending.pop_back();
    if (value == nullptr || !visited.insert(value).second || LLVMIsAUser(value) == nullptr) {
      continue;
    }
    const bool cast =
        (LLVMIsAConstantExpr(value) != nullptr && LLVMGetConstOpcode(value) == LLVMIntToPtr) ||
        (LLVMIsAInstruction(value) != nullptr && LLVMGetInstructionOpcode(value) == LLVMIntToPtr);
    if (cast) {
      auto *address = LLVMGetOperand(value, 0);
      if (LLVMIsConstant(address) != 0 && LLVMIsNull(address) == 0) {
        throw std::invalid_argument("AOT input embeds an absolute host pointer");
      }
    }
    for (int operand = 0; operand < LLVMGetNumOperands(value); ++operand) {
      pending.push_back(LLVMGetOperand(value, static_cast<unsigned>(operand)));
    }
  }
}
}
