// SPDX-License-Identifier: GPL-3.0-only
#include "calls.hpp"

#include "llvm-c/Core.h"
#include "llvm-c/Error.h"
#include "llvm-c/LLJIT.h"
#include "llvm-c/Object.h"
#include "llvm-c/Orc.h"
#include "llvm-c/TargetMachine.h"
#include "llvm-c/Types.h"

#include <atomic>
#include <bit>
#include <dlfcn.h>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {
std::atomic<unsigned> machines{0}, engines{0}, objects{0}, helpers{0}, modules{0};

struct Probe {
  LLVMOrcObjectTransformLayerTransformFunction transform;
  void *owner;
};

std::vector<std::unique_ptr<Probe>> probes;

template <typename Function> Function original(const char *name) {
  const auto *address = dlsym(RTLD_NEXT, name);
  if (address == nullptr) { throw std::runtime_error("Cannot resolve the packaged LLVM API"); }
  return std::bit_cast<Function>(address);
}

LLVMErrorRef inspect(void *owner, LLVMModuleRef module) noexcept {
  static_cast<void>(owner);
  for (const auto *const name : {"aot_host", "aot_first", "_Z12aot_snapshotPKj"}) {
    if (auto *function = LLVMGetNamedFunction(module, name); function != nullptr &&
                                                             LLVMIsDeclaration(function) == 0 &&
                                                             LLVMGetFirstUse(function) != nullptr) {
      ++helpers;
    }
  }
  return nullptr;
}

LLVMErrorRef observe(void *owner, LLVMMemoryBufferRef *buffer) noexcept {
  char *error = nullptr;
  auto *binary = LLVMCreateBinary(*buffer, nullptr, &error);
  if (error != nullptr) { LLVMDisposeMessage(error); }
  if (binary == nullptr) { return LLVMCreateStringError("Cannot inspect actual native object"); }
  auto *symbols = LLVMObjectFileCopySymbolIterator(binary);
  bool first = false;
  bool second = false;
  if (symbols != nullptr) {
    while (LLVMObjectFileIsSymbolIteratorAtEnd(binary, symbols) == 0) {
      const auto *raw = LLVMGetSymbolName(symbols);
      const std::string_view name(raw == nullptr ? "" : raw);
      first |= name == "aot_first" || name == "_aot_first";
      second |= name == "aot_second" || name == "_aot_second";
      LLVMMoveToNextSymbol(symbols);
    }
    LLVMDisposeSymbolIterator(symbols);
  }
  LLVMDisposeBinary(binary);
  if (first || second) {
    ++objects;
    if (!first || !second) { ++helpers; }
  }
  const auto &probe = *static_cast<Probe *>(owner);
  return probe.transform(probe.owner, buffer);
}
}

extern "C" LLVMTargetMachineRef LLVMCreateTargetMachine(LLVMTargetRef target,
                                                        const char *triple,
                                                        const char *cpu,
                                                        const char *features,
                                                        LLVMCodeGenOptLevel level,
                                                        LLVMRelocMode relocation,
                                                        LLVMCodeModel model) {
  ++machines;
  return original<decltype(&LLVMCreateTargetMachine)>("LLVMCreateTargetMachine")(
      target, triple, cpu, features, level, relocation, model);
}

extern "C" LLVMErrorRef LLVMOrcCreateLLJIT(LLVMOrcLLJITRef *result,
                                           LLVMOrcLLJITBuilderRef builder) {
  ++engines;
  return original<decltype(&LLVMOrcCreateLLJIT)>("LLVMOrcCreateLLJIT")(result, builder);
}

extern "C" LLVMErrorRef LLVMOrcLLJITAddLLVMIRModule(LLVMOrcLLJITRef J,
                                                    LLVMOrcJITDylibRef JD,
                                                    LLVMOrcThreadSafeModuleRef TSM) {
  ++modules;
  if (auto *error = LLVMOrcThreadSafeModuleWithModuleDo(TSM, inspect, nullptr)) { return error; }
  return original<decltype(&LLVMOrcLLJITAddLLVMIRModule)>("LLVMOrcLLJITAddLLVMIRModule")(
      J, JD, TSM);
}

extern "C" void LLVMOrcObjectTransformLayerSetTransform(
    LLVMOrcObjectTransformLayerRef ObjTransformLayer,
    LLVMOrcObjectTransformLayerTransformFunction TransformFunction,
    void *Ctx) {
  probes.push_back(std::make_unique<Probe>(Probe{.transform = TransformFunction, .owner = Ctx}));
  original<decltype(&LLVMOrcObjectTransformLayerSetTransform)>(
      "LLVMOrcObjectTransformLayerSetTransform")(ObjTransformLayer, observe, probes.back().get());
}

namespace blofeld::test::aot {
Calls calls() noexcept {
  return {.machines = machines.load(),
          .engines = engines.load(),
          .objects = objects.load(),
          .helpers = helpers.load(),
          .modules = modules.load()};
}
}
