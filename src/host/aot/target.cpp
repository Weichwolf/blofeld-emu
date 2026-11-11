// SPDX-License-Identifier: GPL-3.0-only
#include "llvm-c/Target.h"

#include "aot/llvm.hpp"
#include "aot/native_init.hpp"

#include "llvm-c/Core.h"
#include "llvm-c/Support.h"
#include "llvm-c/TargetMachine.h"
#include "llvm-c/Types.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/TargetParser/AArch64TargetParser.h"
#include "llvm/TargetParser/SubtargetFeature.h"
#include "llvm/TargetParser/Triple.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::host::aot::detail {
namespace {
std::string attribute(LLVMValueRef function, const std::string_view key) {
  auto *value =
      LLVMGetStringAttributeAtIndex(function,
                                    static_cast<LLVMAttributeIndex>(LLVMAttributeFunctionIndex),
                                    key.data(),
                                    static_cast<unsigned>(key.size()));
  if (value == nullptr) { return {}; }
  unsigned size = 0;
  const auto *text = LLVMGetStringAttributeValue(value, &size);
  return {text, size};
}

std::string features() {
  const Owner<char *, LLVMDisposeMessage> triple(LLVMGetDefaultTargetTriple());
  const Owner<char *, LLVMDisposeMessage> cpu(LLVMGetHostCPUName());
  const Owner<char *, LLVMDisposeMessage> flags(LLVMGetHostCPUFeatures());
  std::string result;
  if (llvm::Triple(triple.get()).isAArch64()) {
    const auto model = llvm::AArch64::parseCpu(cpu.get());
    if (!model) { throw std::runtime_error("Cannot resolve native AOT CPU extensions"); }
    llvm::AArch64::ExtensionSet extensions;
    extensions.addArchDefaults(model->Arch);
    extensions.addCPUDefaults(*model);
    std::vector<std::string> defaults;
    extensions.toLLVMFeatureList(defaults);
    for (const auto &feature : defaults) { result += feature + ","; }
  }
  result += flags.get();
  return result;
}

void validate_features(LLVMValueRef function,
                       const llvm::Target &target,
                       const std::string &triple) {
  const Owner<char *, LLVMDisposeMessage> cpu(LLVMGetHostCPUName());
  const std::unique_ptr<llvm::MCSubtargetInfo> native(
      target.createMCSubtargetInfo(triple, cpu.get(), features()));
  if (!native) { throw std::runtime_error("Cannot resolve native AOT feature model"); }
  const auto flags = attribute(function, "target-features");
  std::string_view remaining(flags);
  while (!remaining.empty()) {
    const auto end = remaining.find(',');
    const auto flag = remaining.substr(0, end);
    if (flag.size() < 2 || (flag.front() != '+' && flag.front() != '-') ||
        !std::ranges::any_of(native->getAllProcessorFeatures(), [&](const auto &candidate) {
          return std::string_view(candidate.Key) == flag.substr(1);
        })) {
      throw std::invalid_argument("AOT has an unknown target feature: " + std::string(flag));
    }
    if (end == std::string_view::npos) { break; }
    remaining.remove_prefix(end + 1);
  }
  auto required_cpu = attribute(function, "target-cpu");
  if (required_cpu.empty()) { required_cpu = cpu.get(); }
  if (!native->isCPUStringValid(required_cpu)) {
    throw std::invalid_argument("AOT has an unknown target CPU: " + required_cpu);
  }
  const std::unique_ptr<llvm::MCSubtargetInfo> required(
      target.createMCSubtargetInfo(triple, required_cpu, flags));
  if (!required || (required->getFeatureBits() & ~native->getFeatureBits()).any()) {
    throw std::invalid_argument("AOT requires unavailable native CPU features");
  }
}
}

void initialize() {
  weichwolf::llvm19::initialize_native();
  if (LLVMLoadLibraryPermanently(nullptr) != 0) {
    throw std::runtime_error("Cannot expose the native process to AOT relocation");
  }
}

void validate_target(LLVMModuleRef module) {
  const Owner<char *, LLVMDisposeMessage> host(LLVMGetDefaultTargetTriple());
  const llvm::Triple native(host.get());
  const std::string source(LLVMGetTarget(module));
  if (!source.empty()) {
    const llvm::Triple actual(source);
    const bool same_os =
        native.getOS() == actual.getOS() || (native.isOSDarwin() && actual.isOSDarwin());
    if (native.getArch() != actual.getArch() ||
        native.getObjectFormat() != actual.getObjectFormat() || !same_os ||
        native.getEnvironment() != actual.getEnvironment()) {
      throw std::invalid_argument("AOT input target is incompatible with the native host");
    }
  }
  std::string error;
  const std::string triple(host.get());
  const auto *target = llvm::TargetRegistry::lookupTarget(triple, error);
  if (target == nullptr) { throw std::runtime_error(error); }
  for (auto *function = LLVMGetFirstFunction(module); function != nullptr;
       function = LLVMGetNextFunction(function)) {
    if (LLVMIsDeclaration(function) == 0) { validate_features(function, *target, triple); }
  }
}

LLVMTargetMachineRef target(LLVMModuleRef module) {
  const Owner<char *, LLVMDisposeMessage> triple(LLVMGetDefaultTargetTriple());
  const Owner<char *, LLVMDisposeMessage> cpu(LLVMGetHostCPUName());
  LLVMTargetRef architecture = nullptr;
  char *raw_error = nullptr;
  const auto failed = LLVMGetTargetFromTriple(triple.get(), &architecture, &raw_error);
  const Owner<char *, LLVMDisposeMessage> error(raw_error);
  if (failed != 0) { throw std::runtime_error(error.get()); }
  Owner<LLVMTargetMachineRef, LLVMDisposeTargetMachine> machine(
      LLVMCreateTargetMachine(architecture,
                              triple.get(),
                              cpu.get(),
                              features().c_str(),
                              LLVMCodeGenLevelAggressive,
                              LLVMRelocPIC,
                              LLVMCodeModelSmall));
  if (!machine) { throw std::runtime_error("Cannot create the native AOT target"); }
  const Owner<LLVMTargetDataRef, LLVMDisposeTargetData> data(
      LLVMCreateTargetDataLayout(machine.get()));
  const Owner<char *, LLVMDisposeMessage> layout(LLVMCopyStringRepOfTargetData(data.get()));
  const std::string_view expected(LLVMGetDataLayoutStr(module));
  if (!expected.empty() && expected != layout.get()) {
    throw std::invalid_argument("AOT layout is incompatible with the native target");
  }
  LLVMSetTarget(module, triple.get());
  LLVMSetDataLayout(module, layout.get());
  return machine.release();
}
}
