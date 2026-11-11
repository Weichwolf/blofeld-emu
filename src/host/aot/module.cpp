// SPDX-License-Identifier: GPL-3.0-only
#include "aot/code.hpp"
#include "aot/llvm.hpp"

#include "llvm-c/Analysis.h"
#include "llvm-c/BitReader.h"
#include "llvm-c/Core.h"
#include "llvm-c/Linker.h"
#include "llvm-c/Types.h"

#include <exception>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace blofeld::host::aot::detail {
namespace {
class Diagnostics {
public:
  explicit Diagnostics(LLVMContextRef context)
      : context_(context),
        handler_(LLVMContextGetDiagnosticHandler(context)),
        owner_(LLVMContextGetDiagnosticContext(context)) {
    LLVMContextSetDiagnosticHandler(context_, receive, this);
  }

  ~Diagnostics() { LLVMContextSetDiagnosticHandler(context_, handler_, owner_); }

  Diagnostics(const Diagnostics &) = delete;
  Diagnostics &operator=(const Diagnostics &) = delete;

  void check(const bool failed) const {
    if (exception_) { std::rethrow_exception(exception_); }
    if (failed || !message_.empty()) {
      throw std::invalid_argument(message_.empty() ? "Cannot import AOT bitcode" : message_);
    }
  }

private:
  static void receive(LLVMDiagnosticInfoRef diagnostic, void *owner) noexcept {
    const auto severity = LLVMGetDiagInfoSeverity(diagnostic);
    if (severity != LLVMDSError && severity != LLVMDSWarning) { return; }
    auto &self = *static_cast<Diagnostics *>(owner);
    try {
      const Owner<char *, LLVMDisposeMessage> message(LLVMGetDiagInfoDescription(diagnostic));
      if (message) { self.message_ += message.get(); }
    } catch (...) { self.exception_ = std::current_exception(); }
  }

  LLVMContextRef context_;
  LLVMDiagnosticHandler handler_;
  void *owner_;
  std::string message_;
  std::exception_ptr exception_;
};

void verify(LLVMModuleRef module) {
  char *raw = nullptr;
  const auto failed = LLVMVerifyModule(module, LLVMReturnStatusAction, &raw);
  const Owner<char *, LLVMDisposeMessage> message(raw);
  if (failed != 0) { throw std::invalid_argument(message.get()); }
}
}

LLVMModuleRef parse(LLVMContextRef context, const Module &source, Exports &exports) {
  if (source.bitcode.empty()) { throw std::invalid_argument("AOT bitcode is empty"); }
  const Diagnostics diagnostics(context);
  const Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer> input(
      LLVMCreateMemoryBufferWithMemoryRangeCopy(
          reinterpret_cast<const char *>(source.bitcode.data()), source.bitcode.size(), "aot"));
  LLVMModuleRef raw = nullptr;
  const auto failed = LLVMParseBitcodeInContext2(context, input.get(), &raw);
  Owner<LLVMModuleRef, LLVMDisposeModule> module(raw);
  diagnostics.check(failed != 0 || !module);
  verify(module.get());
  Exports roots;
  for (const auto entry : source.exports) {
    if (entry.name.empty() || entry.name.find('\0') != std::string_view::npos ||
        entry.abi.empty() || exports.contains(std::string(entry.name)) ||
        !roots.emplace(entry.name, entry.abi).second) {
      throw std::invalid_argument("AOT has an invalid or duplicate export");
    }
  }
  validate_exports(module.get(), roots);
  exports.merge(roots);
  return module.release();
}

void merge(LLVMModuleRef destination, Owner<LLVMModuleRef, LLVMDisposeModule> module) {
  const auto copy = [&](const bool target) {
    const auto *source = target ? LLVMGetTarget(module.get()) : LLVMGetDataLayoutStr(module.get());
    const auto *current = target ? LLVMGetTarget(destination) : LLVMGetDataLayoutStr(destination);
    if (std::string_view(current).empty()) {
      if (target) {
        LLVMSetTarget(destination, source);
      } else {
        LLVMSetDataLayout(destination, source);
      }
    } else if (!target && !std::string_view(source).empty() &&
               std::string_view(source) != current) {
      throw std::invalid_argument("AOT modules have incompatible data layouts");
    }
  };
  copy(true);
  copy(false);
  LLVMSetTarget(module.get(), LLVMGetTarget(destination));
  LLVMSetDataLayout(module.get(), LLVMGetDataLayoutStr(destination));
  const Diagnostics diagnostics(LLVMGetModuleContext(destination));
  diagnostics.check(LLVMLinkModules2(destination, module.release()) != 0);
  verify(destination);
}
}
