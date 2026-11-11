// SPDX-License-Identifier: GPL-3.0-only
#include "aot/code.hpp"
#include "calls.hpp"
#include "fixture.hpp"

#include "llvm-c/BitReader.h"
#include "llvm-c/BitWriter.h"
#include "llvm-c/Core.h"
#include "llvm-c/Types.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace {
using namespace blofeld::host::aot;
using blofeld::test::aot::calls;
std::atomic<unsigned> started{0}, finished{0};
constexpr std::string_view function_abi = "llvm19;cc=0;type=i32 (i32, ptr)";
constexpr std::string_view getter_abi = "llvm19;cc=0;type=i32 ()";
constexpr const char *snapshot = "_Z12aot_snapshotPKj";

template <auto Dispose> struct Delete {
  void operator()(auto *value) const noexcept { Dispose(value); }
};
template <class Ref, auto Dispose>
using Owner = std::unique_ptr<std::remove_pointer_t<Ref>, Delete<Dispose>>;

void require(const bool passed, const char *message) {
  if (!passed) { throw std::runtime_error(message); }
}

std::vector<std::byte> copy(LLVMMemoryBufferRef buffer) {
  const auto *first = reinterpret_cast<const std::byte *>(LLVMGetBufferStart(buffer));
  return {first, first + LLVMGetBufferSize(buffer)};
}

std::vector<std::byte> read(const char *path) {
  LLVMMemoryBufferRef raw = nullptr;
  char *raw_error = nullptr;
  const auto failed = LLVMCreateMemoryBufferWithContentsOfFile(path, &raw, &raw_error);
  const Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer> input(raw);
  const Owner<char *, LLVMDisposeMessage> error(raw_error);
  require(failed == 0 && input != nullptr, "Read independently Clang-compiled component IR");
  return copy(input.get());
}

template <class Action>
std::vector<std::byte> change(const std::vector<std::byte> &bytes, Action action) {
  const Owner<LLVMContextRef, LLVMContextDispose> context(LLVMContextCreate());
  const Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer> input(
      LLVMCreateMemoryBufferWithMemoryRangeCopy(
          reinterpret_cast<const char *>(bytes.data()), bytes.size(), "contract"));
  LLVMModuleRef raw = nullptr;
  require(LLVMParseBitcodeInContext2(context.get(), input.get(), &raw) == 0,
          "Parse a valid independently compiled contract fixture");
  const Owner<LLVMModuleRef, LLVMDisposeModule> module(raw);
  action(module.get());
  const Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer> output(
      LLVMWriteBitcodeToMemoryBuffer(module.get()));
  return copy(output.get());
}

struct Inputs {
  std::vector<std::byte> first = read(BLOFELD_AOT_FIRST), second = read(BLOFELD_AOT_SECOND),
                         host = read(BLOFELD_AOT_HOST);
  std::array<std::string, 3> names{"aot_first", "aot_second", "aot_initializations"};
  std::array<Export, 3> exports{{{.name = names[0], .abi = function_abi},
                                 {.name = names[1], .abi = function_abi},
                                 {.name = names[2], .abi = getter_abi}}};

  [[nodiscard]] std::array<Module, 3> modules() const {
    return {{{.bitcode = first, .exports = std::span(exports).subspan(0, 1)},
             {.bitcode = second, .exports = std::span(exports).subspan(1, 1)},
             {.bitcode = host, .exports = std::span(exports).subspan(2, 1)}}};
  }
};

std::shared_ptr<const Code> prepare(const bool reverse) {
  const Inputs inputs;
  auto modules = inputs.modules();
  if (reverse) { std::ranges::reverse(modules); }
  return std::make_shared<const Code>(modules);
}

unsigned first_reference(const unsigned value, unsigned &state) {
  const auto mixed = (state * 3) ^ value;
  state = mixed + value;
  return (mixed + 7) ^ state;
}

unsigned second_reference(const unsigned value, unsigned &state) {
  const auto first = first_reference(value, state);
  state = (state * 3) ^ first;
  return first + state + 7;
}

using Function = unsigned (*)(unsigned, unsigned *) noexcept;

struct Binding {
  std::shared_ptr<const Code> owner;
  Function function = nullptr;
};

void verify(const Binding &binding, unsigned (*reference)(unsigned, unsigned &)) {
  constexpr unsigned byte_values = 256;
  constexpr std::array seeds{0U, 1U, 17U, 8192U};
  for (const auto seed : seeds) {
    for (unsigned value = 0; value < byte_values; ++value) {
      auto expected = seed;
      auto actual = seed;
      const auto result = reference(value, expected);
      require(binding.function(value, &actual) == result && actual == expected,
              "Joint native execution preserves independent arithmetic and ordered effects");
    }
  }
}

void lifetime(const bool reverse) {
  const auto before = calls();
  const auto initializations = started.load();
  const auto finalizations = finished.load();
  auto owner = prepare(reverse);
  const auto prepared = calls();
  require(prepared.machines == before.machines + 1 && prepared.engines == before.engines + 1 &&
              prepared.objects == before.objects + 1 && prepared.modules == before.modules + 1 &&
              prepared.helpers == before.helpers && owner->native_bytes() != 0,
          "All components receive one final native optimizer/object/owner with host calls fused");
  using Getter = unsigned (*)() noexcept;
  require(std::bit_cast<Getter>(owner->address("aot_initializations"))() == 1 &&
              started.load() == initializations + 1 && finished.load() == finalizations,
          "The actual native module initializer runs exactly once");
  const std::weak_ptr<const Code> weak = owner;
  Binding first{.owner = owner, .function = std::bit_cast<Function>(owner->address("aot_first"))};
  Binding second{.owner = owner, .function = std::bit_cast<Function>(owner->address("aot_second"))};
  bool rejected = false;
  try {
    static_cast<void>(owner->address("aot_host"));
  } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Only declared roots are public native exports");
  owner.reset();
  verify(first, first_reference);
  verify(second, second_reference);
  first = {};
  require(!weak.expired() && finished.load() == finalizations,
          "Either component binding retains the same code resource");
  verify(second, second_reference);
  require(calls() == prepared, "Native execution never reenters the compiler");
  second = {};
  require(weak.expired() && finished.load() == finalizations + 1,
          "The actual native module finalizer runs once after the last binding expires");
}

void return_contract() {
  Inputs inputs;
  inputs.first = change(inputs.first, [](LLVMModuleRef module) {
    auto *function = LLVMGetNamedFunction(module, "aot_first");
    for (auto *block = LLVMGetFirstBasicBlock(function); block != nullptr;
         block = LLVMGetNextBasicBlock(block)) {
      for (auto *instruction = LLVMGetFirstInstruction(block); instruction != nullptr;
           instruction = LLVMGetNextInstruction(instruction)) {
        if (LLVMIsACallInst(instruction) != nullptr &&
            LLVMGetCalledValue(instruction) == LLVMGetNamedFunction(module, "aot_host")) {
          constexpr std::string_view name = "noundef";
          LLVMAddCallSiteAttribute(
              instruction,
              LLVMAttributeReturnIndex,
              LLVMCreateEnumAttribute(LLVMGetModuleContext(module),
                                      LLVMGetEnumAttributeKindForName(name.data(), name.size()),
                                      0));
        }
      }
    }
  });
  const auto modules = inputs.modules();
  const auto owner = std::make_shared<const Code>(modules);
  verify({.owner = owner, .function = std::bit_cast<Function>(owner->address("aot_first"))},
         first_reference);
}

void rejected(const std::span<const Module> modules, const std::string_view reason) {
  const auto before = calls();
  const auto initializations = started.load();
  bool failed = false;
  try {
    const Code code(modules);
  } catch (const std::invalid_argument &error) {
    failed = std::string_view(error.what()).contains(reason);
  }
  require(failed && calls() == before && started.load() == initializations,
          "An invalid original contract fails before native machinery or initialization");
}

void attribute(LLVMValueRef function,
               const unsigned index,
               const std::string_view name,
               const std::uint64_t value = 0) {
  const auto kind = LLVMGetEnumAttributeKindForName(name.data(), name.size());
  require(kind != 0, "The contract attribute exists in packaged LLVM19");
  LLVMAddAttributeAtIndex(
      function,
      index,
      LLVMCreateEnumAttribute(LLVMGetModuleContext(LLVMGetGlobalParent(function)), kind, value));
}

void contracts() {
  Inputs inputs;
  rejected({}, "requires declared");
  auto modules = inputs.modules();
  rejected(std::span(modules).first(2), "aot_host");
  modules[2].bitcode = {};
  rejected(modules, "bitcode is empty");
  modules = inputs.modules();
  const std::array invalid{std::byte{0}, std::byte{1}};
  modules[2].bitcode = invalid;
  rejected(modules, "bitcode");
  modules = inputs.modules();
  inputs.exports[0].abi = getter_abi;
  rejected(modules, "incompatible ABI");
  inputs.exports[0].abi = function_abi;
  inputs.exports[0].name = "absent_export";
  rejected(modules, "missing");
  inputs.exports[0].name = inputs.names[0];
  modules[1].exports = modules[0].exports;
  rejected(modules, "duplicate export");
  const auto original = inputs.host;
  const auto malformed = [&](auto mutation, const std::string_view reason) {
    inputs.host = change(original, mutation);
    const auto changed = inputs.modules();
    rejected(changed, reason);
    auto reversed = changed;
    std::ranges::reverse(reversed);
    rejected(reversed, reason);
  };
  malformed([](LLVMModuleRef module) { LLVMSetTarget(module, "wasm32-unknown-unknown"); },
            "target");
  malformed([](LLVMModuleRef module) { LLVMSetDataLayout(module, "e-p:32:32"); }, "data layouts");
  malformed(
      [](LLVMModuleRef module) {
        LLVMSetFunctionCallConv(LLVMGetNamedFunction(module, "aot_host"), LLVMFastCallConv);
      },
      "ABI mismatch");
  malformed(
      [](LLVMModuleRef module) {
        attribute(LLVMGetNamedFunction(module, "aot_host"), 2, "noalias");
      },
      "parameter contract");
  malformed(
      [](LLVMModuleRef module) {
        attribute(LLVMGetNamedFunction(module, "aot_host"), 2, "align", 16);
      },
      "pointer guarantee");
  malformed(
      [](LLVMModuleRef module) {
        attribute(LLVMGetNamedFunction(module, "aot_host"), 2, "dereferenceable", 64);
      },
      "pointer guarantee");
  malformed(
      [](LLVMModuleRef module) {
        constexpr std::string_view name = "nounwind";
        LLVMRemoveEnumAttributeAtIndex(LLVMGetNamedFunction(module, "aot_host"),
                                       static_cast<LLVMAttributeIndex>(LLVMAttributeFunctionIndex),
                                       LLVMGetEnumAttributeKindForName(name.data(), name.size()));
      },
      "function guarantee");
  malformed(
      [](LLVMModuleRef module) {
        constexpr std::string_view key = "target-features";
        constexpr std::string_view value = "+blofeld_unknown_feature";
        LLVMAddAttributeAtIndex(
            LLVMGetNamedFunction(module, "aot_host"),
            static_cast<LLVMAttributeIndex>(LLVMAttributeFunctionIndex),
            LLVMCreateStringAttribute(
                LLVMGetModuleContext(module), key.data(), key.size(), value.data(), value.size()));
      },
      "unknown target feature");
  malformed(
      [](LLVMModuleRef module) {
        auto *context = LLVMGetModuleContext(module);
        auto *pointer = LLVMPointerTypeInContext(context, 0);
        auto *global = LLVMAddGlobal(module, pointer, "absolute_pointer");
        constexpr std::uint64_t address = 0x12345678;
        LLVMSetInitializer(
            global,
            LLVMConstIntToPtr(LLVMConstInt(LLVMInt64TypeInContext(context), address, 0), pointer));
      },
      "absolute host pointer");
}

void aggregate_contracts() {
  struct Shape {
    unsigned elements, bits;
    bool packed;
  };

  Inputs inputs;
  const auto original = inputs.second;
  constexpr std::array shapes{Shape{.elements = 7, .bits = 32, .packed = false},
                              Shape{.elements = 9, .bits = 32, .packed = false},
                              Shape{.elements = 4, .bits = 64, .packed = false},
                              Shape{.elements = 8, .bits = 32, .packed = true}};
  for (const auto shape : shapes) {
    inputs.second = change(original, [shape](LLVMModuleRef module) {
      auto *context = LLVMGetModuleContext(module);
      auto *function = LLVMGetNamedFunction(module, snapshot);
      require(function != nullptr, "The independent fixture defines its aggregate return");
      auto *array = LLVMArrayType(LLVMIntTypeInContext(context, shape.bits), shape.elements);
      auto *storage = LLVMStructTypeInContext(context, &array, 1, 0);
      auto *type = LLVMStructTypeInContext(context, &storage, 1, shape.packed ? 1 : 0);
      constexpr std::string_view name = "sret";
      LLVMAddAttributeAtIndex(
          function,
          1,
          LLVMCreateTypeAttribute(
              context, LLVMGetEnumAttributeKindForName(name.data(), name.size()), type));
    });
    auto modules = inputs.modules();
    rejected(modules, "ABI attribute mismatch");
    std::ranges::reverse(modules);
    rejected(modules, "ABI attribute mismatch");
  }
  inputs.second = original;
  const auto host = inputs.host;
  for (const bool escape : {false, true}) {
    inputs.host = change(host, [&](LLVMModuleRef module) {
      auto *function = LLVMGetNamedFunction(module, "aot_host");
      bool changed = false;
      for (auto *block = LLVMGetFirstBasicBlock(function); block != nullptr;
           block = LLVMGetNextBasicBlock(block)) {
        for (auto *call = LLVMGetFirstInstruction(block); call != nullptr;
             call = LLVMGetNextInstruction(call)) {
          if (LLVMIsACallInst(call) == nullptr ||
              LLVMGetCalledValue(call) != LLVMGetNamedFunction(module, snapshot)) {
            continue;
          }
          auto *output = LLVMGetOperand(call, 0);
          const Owner<LLVMBuilderRef, LLVMDisposeBuilder> builder(
              LLVMCreateBuilderInContext(LLVMGetModuleContext(module)));
          LLVMPositionBuilderBefore(builder.get(), call);
          if (escape) {
            auto *pointer = LLVMPointerTypeInContext(LLVMGetModuleContext(module), 0);
            auto *global = LLVMAddGlobal(module, pointer, "escaped_result");
            LLVMSetInitializer(global, LLVMConstNull(pointer));
            LLVMBuildStore(builder.get(), output, global);
          } else {
            LLVMBuildStore(builder.get(),
                           LLVMConstInt(LLVMInt32TypeInContext(LLVMGetModuleContext(module)), 0, 0),
                           output);
            LLVMSetOperand(call, 1, output);
          }
          changed = true;
        }
      }
      require(changed, "The independent caller has an actual aggregate-return invocation");
    });
    auto modules = inputs.modules();
    rejected(modules, "parameter contract");
    std::ranges::reverse(modules);
    rejected(modules, "parameter contract");
  }
}
}

extern "C" void aot_test_event(const unsigned event) noexcept {
  if (event == 1) {
    ++started;
  } else if (event == 2) {
    ++finished;
  } else {
    std::terminate();
  }
}

int main() try {
  lifetime(false);
  lifetime(true);
  return_contract();
  contracts();
  aggregate_contracts();
  std::cout << "Verified joint native fusion, prelink contracts, actual lifecycle and shared "
               "ownership.\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
