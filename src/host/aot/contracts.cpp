// SPDX-License-Identifier: GPL-3.0-only
#include "aot/attributes.hpp"
#include "aot/llvm.hpp"

#include "llvm-c/Types.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/AssumptionCache.h"
#include "llvm/Analysis/BasicAliasAnalysis.h"
#include "llvm/Analysis/CaptureTracking.h"
#include "llvm/Analysis/MemoryLocation.h"
#include "llvm/Analysis/TargetLibraryInfo.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Value.h"
#include "llvm/Support/Casting.h"
#include "llvm/TargetParser/Triple.h"

#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace blofeld::host::aot::detail {
namespace {
using TypePairs = std::set<std::pair<const llvm::Type *, const llvm::Type *>>;

bool same_type(const llvm::Type *left, const llvm::Type *right, TypePairs &visited) {
  if (left == right) { return true; }
  if (left->getTypeID() != right->getTypeID()) { return false; }
  if (const auto *structure = llvm::dyn_cast<llvm::StructType>(left)) {
    const auto *other = llvm::cast<llvm::StructType>(right);
    if (structure->isOpaque() || other->isOpaque() || structure->isPacked() != other->isPacked()) {
      return false;
    }
  } else if (const auto *array = llvm::dyn_cast<llvm::ArrayType>(left)) {
    if (array->getNumElements() != llvm::cast<llvm::ArrayType>(right)->getNumElements()) {
      return false;
    }
  } else if (const auto *function = llvm::dyn_cast<llvm::FunctionType>(left)) {
    if (function->isVarArg() != llvm::cast<llvm::FunctionType>(right)->isVarArg()) { return false; }
  } else {
    return false;
  }
  if (left->getNumContainedTypes() != right->getNumContainedTypes()) { return false; }
  if (!visited.emplace(left, right).second) { return true; }
  for (unsigned index = 0; index < left->getNumContainedTypes(); ++index) {
    if (!same_type(left->getContainedType(index), right->getContainedType(index), visited)) {
      return false;
    }
  }
  return true;
}

bool same_attribute(const llvm::Attribute left, const llvm::Attribute right) {
  if (left == right) { return true; }
  if (!left.isTypeAttribute() || !right.isTypeAttribute() ||
      left.getKindAsEnum() != right.getKindAsEnum()) {
    return false;
  }
  TypePairs visited;
  return same_type(left.getValueAsType(), right.getValueAsType(), visited);
}

bool noalias(const llvm::Function &caller, const unsigned index) {
  return caller.hasParamAttribute(index, llvm::Attribute::NoAlias);
}

bool noalias(llvm::CallBase &caller, const unsigned index) {
  if (caller.getAttributes().hasParamAttr(index, llvm::Attribute::NoAlias)) { return true; }
  const auto *argument = caller.getArgOperand(index);
  const auto *object = llvm::dyn_cast<llvm::AllocaInst>(llvm::getUnderlyingObject(argument));
  if (object == nullptr) { return false; }
  auto &function = *caller.getFunction();
  llvm::DominatorTree dominators(function);
  if (llvm::PointerMayBeCapturedBefore(object, true, true, &caller, &dominators)) { return false; }
  const llvm::TargetLibraryInfoImpl library(llvm::Triple(function.getParent()->getTargetTriple()));
  const llvm::TargetLibraryInfo information(library);
  llvm::AssumptionCache assumptions(function);
  llvm::BasicAAResult basic(
      function.getParent()->getDataLayout(), function, information, assumptions, &dominators);
  llvm::AAResults aliases(information);
  aliases.addAAResult(basic);
  llvm::EarliestEscapeInfo captures(dominators);
  llvm::AAQueryInfo query(aliases, &captures);
  for (unsigned other = 0; other < caller.arg_size(); ++other) {
    const auto *value = caller.getArgOperand(other);
    if (other != index && value->getType()->isPointerTy() &&
        aliases.alias(llvm::MemoryLocation::getBeforeOrAfter(argument),
                      llvm::MemoryLocation::getBeforeOrAfter(value),
                      query,
                      &caller) != llvm::AliasResult::NoAlias) {
      return false;
    }
  }
  return true;
}

std::uint64_t integer(llvm::AttributeSet attributes, const llvm::Attribute::AttrKind kind) {
  const auto attribute = attributes.getAttribute(kind);
  return attribute.isValid() ? attribute.getValueAsInt() : 0;
}

void parameters(llvm::AttributeSet guaranteed,
                llvm::AttributeSet required,
                const std::string &name) {
  for (const auto kind : {llvm::Attribute::NonNull, llvm::Attribute::NoUndef}) {
    if (required.hasAttribute(kind) && !guaranteed.hasAttribute(kind)) {
      throw std::invalid_argument("AOT stronger parameter contract: " + name + ";" +
                                  llvm::Attribute::getNameFromAttrKind(kind).str());
    }
  }
  for (const auto kind : {llvm::Attribute::Alignment,
                          llvm::Attribute::Dereferenceable,
                          llvm::Attribute::DereferenceableOrNull}) {
    if (integer(required, kind) > integer(guaranteed, kind)) {
      throw std::invalid_argument("AOT stronger pointer guarantee: " + name);
    }
  }
  if (required.hasAttribute(llvm::Attribute::Range) &&
      required.getAttribute(llvm::Attribute::Range) !=
          guaranteed.getAttribute(llvm::Attribute::Range)) {
    throw std::invalid_argument("AOT incompatible parameter range: " + name);
  }
}

template <class Caller> void compatible(Caller &caller, const llvm::Function &target) {
  const auto name = target.getName().str();
  if (caller.getFunctionType() != target.getFunctionType() ||
      caller.getCallingConv() != target.getCallingConv()) {
    throw std::invalid_argument("AOT function ABI mismatch: " + name);
  }
  const auto expected = caller.getAttributes();
  const auto actual = target.getAttributes();
  for (unsigned index = 0; index <= target.arg_size(); ++index) {
    for (const auto kind : abi_attributes) {
      if (!same_attribute(expected.getAttributeAtIndex(index, kind),
                          actual.getAttributeAtIndex(index, kind))) {
        throw std::invalid_argument("AOT ABI attribute mismatch: " + name);
      }
    }
    if (index != 0) {
      if (actual.hasParamAttr(index - 1, llvm::Attribute::NoAlias) && !noalias(caller, index - 1)) {
        throw std::invalid_argument("AOT stronger parameter contract: " + name +
                                    ";noalias;index=" + std::to_string(index - 1));
      }
      parameters(expected.getParamAttrs(index - 1), actual.getParamAttrs(index - 1), name);
    }
  }
  for (const auto kind : {llvm::Attribute::NoUnwind,
                          llvm::Attribute::NoFree,
                          llvm::Attribute::NoSync,
                          llvm::Attribute::WillReturn}) {
    if (expected.hasFnAttr(kind) && !actual.hasFnAttr(kind)) {
      throw std::invalid_argument("AOT weaker function guarantee: " + name);
    }
  }
  const auto effects = target.getMemoryEffects();
  if ((effects & caller.getMemoryEffects()) != effects) {
    throw std::invalid_argument("AOT incompatible memory effects: " + name);
  }
}

void references(llvm::Function &declaration, const llvm::Function &definition) {
  if (declaration.getFunctionType() != definition.getFunctionType() ||
      declaration.getCallingConv() != definition.getCallingConv()) {
    throw std::invalid_argument("AOT function ABI mismatch: " + definition.getName().str());
  }
  bool escaped = declaration.use_empty();
  for (auto *user : declaration.users()) {
    if (auto *call = llvm::dyn_cast<llvm::CallBase>(user);
        call != nullptr && call->getCalledOperand() == &declaration) {
      compatible(*call, definition);
    } else {
      escaped = true;
    }
  }
  if (escaped) { compatible(declaration, definition); }
}
}

void validate_import_calls(LLVMValueRef function) {
  auto &declaration = *llvm::cast<llvm::Function>(llvm::unwrap(function));
  references(declaration, declaration);
}

void validate_contracts(LLVMModuleRef expected, LLVMModuleRef incoming) {
  auto &source = *llvm::unwrap(expected);
  const auto &definitions = *llvm::unwrap(incoming);
  for (auto &function : source) {
    if (!function.isDeclaration() || function.isIntrinsic()) { continue; }
    const auto *definition = definitions.getFunction(function.getName());
    if (definition != nullptr && !definition->isDeclaration() && !definition->hasLocalLinkage()) {
      references(function, *definition);
    }
  }
}
}
