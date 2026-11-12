// SPDX-License-Identifier: GPL-3.0-only
#include "aot/code.hpp"
#include "aot/link_graph.hpp"
#include "aot/llvm.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <exception>
#include <iostream>
#include <llvm-c/BitReader.h>
#include <llvm-c/BitWriter.h>
#include <llvm-c/Core.h>
#include <llvm-c/Error.h>
#include <llvm-c/IRReader.h>
#include <llvm-c/LLJIT.h>
#include <llvm-c/Orc.h>
#include <llvm-c/Types.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld::host::aot;
constexpr unsigned nodes = 512, depth_limit = 4;
constexpr std::string_view abi = "llvm19;cc=0;type=i32 (i32, i32)";
constexpr std::array import_names{"_blofeld_private_graph_0", "unused_native_value"};
constexpr std::array export_names{"_blofeld_private_graph_0_0",
                                  "__blofeld_private_graph_0_0",
                                  "_blofeld_private_graph_1",
                                  "__blofeld_private_graph_1"};

unsigned native_unused(const unsigned x, const unsigned y) noexcept {
  return x ^ y;
}

std::vector<std::byte> source(const unsigned size) {
  std::string text = "@table = private constant [" + std::to_string(size) + " x ptr] [";
  for (unsigned i = 0; i < size; ++i) {
    text += (i == 0 ? "" : ",") + std::string("ptr @f") + std::to_string(i);
  }
  text += "]\n";
  const auto body = [&](const unsigned i) {
    const auto id = std::to_string(i);
    const auto mask = std::to_string(size - 1);
    return std::string("(i32 %x, i32 %depth) {\nentry:\n%done = icmp eq i32 %depth, 0\n") +
           "br i1 %done, label %stop, label %again\nstop:\nret i32 " + id +
           "\nagain:\n%sum = add i32 %x, " + id + "\n%idx = and i32 %sum, " + mask +
           "\n%ptr = getelementptr [" + std::to_string(size) +
           " x ptr], ptr @table, i32 0, i32 %idx\n%fn = load ptr, ptr %ptr\n" +
           "%next = sub i32 %depth, 1\n%value = call i32 %fn(i32 %x, i32 %next)\n" +
           "%result = add i32 %value, " + id + "\nret i32 %result\n}\n";
  };
  for (unsigned i = 0; i < size; ++i) {
    text += "define internal i32 @f" + std::to_string(i) + body(i);
  }
  text += "define i32 @graph_entry(i32 %x, i32 %depth) {\nbegin:\n"
          "br i1 false, label %unused, label %entry\nunused:\n";
  for (const auto *const name : import_names) {
    text += "call i32 @" + std::string(name) + "(i32 %x, i32 %depth)\n";
  }
  text += "br label %entry\nentry:\n%idx = and i32 %x, " + std::to_string(size - 1) +
          "\n%ptr = getelementptr [" + std::to_string(size) +
          " x ptr], ptr @table, i32 0, i32 %idx\n%fn = load ptr, ptr %ptr\n" +
          "%value = call i32 %fn(i32 %x, i32 %depth)\nret i32 %value\n}\n";
  for (const auto *const name : import_names) {
    text += "declare i32 @" + std::string(name) + "(i32, i32)\n";
  }
  for (unsigned index = 0; index < export_names.size(); ++index) {
    text += "define i32 @" + std::string(export_names[index]) + "(i32 %x, i32 %depth) { ret i32 " +
            std::to_string(index + 1) + " }\n";
  }
  auto *context = LLVMContextCreate();
  auto *input = LLVMCreateMemoryBufferWithMemoryRangeCopy(text.data(), text.size(), "graph");
  LLVMModuleRef module = nullptr;
  char *error = nullptr;
  if (LLVMParseIRInContext(context, input, &module, &error) != 0) {
    const std::string diagnostic(error);
    LLVMDisposeMessage(error);
    LLVMContextDispose(context);
    throw std::runtime_error(diagnostic);
  }
  auto *buffer = LLVMWriteBitcodeToMemoryBuffer(module);
  const auto *first = reinterpret_cast<const std::byte *>(LLVMGetBufferStart(buffer));
  std::vector<std::byte> bytes(first, first + LLVMGetBufferSize(buffer));
  LLVMDisposeMemoryBuffer(buffer);
  LLVMDisposeModule(module);
  LLVMContextDispose(context);
  return bytes;
}

void checked(LLVMErrorRef error) {
  if (error == nullptr) { return; }
  const detail::Owner<char *, LLVMDisposeErrorMessage> message(LLVMGetErrorMessage(error));
  throw std::runtime_error(message.get());
}

void dispose(LLVMOrcLLJITRef jit) noexcept {
  try {
    checked(LLVMOrcDisposeLLJIT(jit));
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    std::terminate();
  }
}

void weak_namespace() {
  LLVMOrcLLJITRef raw = nullptr;
  checked(LLVMOrcCreateLLJIT(&raw, nullptr));
  const detail::Owner<LLVMOrcLLJITRef, dispose> owner(raw);
  detail::named_dependencies(*reinterpret_cast<llvm::orc::LLJIT *>(raw));
  constexpr std::array names{"_blofeld_private_graph_0", "__blofeld_private_graph_0"};
  const auto address = std::bit_cast<LLVMOrcExecutorAddress>(&native_unused);
  std::array<LLVMOrcCSymbolMapPair, names.size()> symbols{};
  for (unsigned i = 0; i < names.size(); ++i) {
    symbols[i] = {.Name = LLVMOrcLLJITMangleAndIntern(raw, names[i]),
                  .Sym = {.Address = address,
                          .Flags = {.GenericFlags = LLVMJITSymbolGenericFlagsWeak |
                                                    LLVMJITSymbolGenericFlagsCallable |
                                                    LLVMJITSymbolGenericFlagsExported,
                                    .TargetFlags = 0}}};
  }
  auto *unit = LLVMOrcAbsoluteSymbols(symbols.data(), symbols.size());
  auto *error = LLVMOrcJITDylibDefine(LLVMOrcLLJITGetMainJITDylib(raw), unit);
  if (error != nullptr) { LLVMOrcDisposeMaterializationUnit(unit); }
  checked(error);
  const auto bytes = source(8);
  const detail::Owner<LLVMOrcThreadSafeContextRef, LLVMOrcDisposeThreadSafeContext> context(
      LLVMOrcCreateNewThreadSafeContext());
  const detail::Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer> input(
      LLVMCreateMemoryBufferWithMemoryRangeCopy(
          reinterpret_cast<const char *>(bytes.data()), bytes.size(), "weak-namespace"));
  LLVMModuleRef module = nullptr;
  if (LLVMParseBitcodeInContext2(
          LLVMOrcThreadSafeContextGetContext(context.get()), input.get(), &module) != 0) {
    throw std::runtime_error("Cannot parse native weak namespace fixture");
  }
  auto *prepared = LLVMOrcCreateNewThreadSafeModule(module, context.get());
  checked(LLVMOrcLLJITAddLLVMIRModule(raw, LLVMOrcLLJITGetMainJITDylib(raw), prepared));
  LLVMOrcExecutorAddress entry = 0;
  checked(LLVMOrcLLJITLookup(raw, &entry, "graph_entry"));
  using Function = unsigned (*)(unsigned, unsigned);
  if (std::bit_cast<Function>(entry)(3, 0) != 3) {
    throw std::runtime_error("Native graph changed while claiming local dependencies");
  }
  for (const auto *const name : names) {
    LLVMOrcExecutorAddress existing = 0;
    checked(LLVMOrcLLJITLookup(raw, &existing, name));
    if (existing != address) {
      throw std::runtime_error("Private native name replaced an existing weak definition");
    }
  }
}

struct Query {
  unsigned value, depth;
};

unsigned expected(const Query query) {
  unsigned result = 0;
  unsigned index = query.value;
  for (unsigned step = 0; step <= query.depth; ++step) {
    result += index;
    index = (query.value + index) & (nodes - 1);
  }
  return result;
}

void qualify() {
  const auto bytes = source(nodes);
  std::vector<Export> exports{{.name = "graph_entry", .abi = abi}};
  exports.reserve(export_names.size() + 1);
  for (const auto *const name : export_names) { exports.push_back({.name = name, .abi = abi}); }
  std::vector<Import> imports;
  imports.reserve(import_names.size());
  for (const auto *const name : import_names) {
    imports.push_back(
        {.name = name, .abi = abi, .address = std::bit_cast<const void *>(&native_unused)});
  }
  const std::array modules{Module{.bitcode = bytes, .exports = exports}};
  const Code code(modules, imports);
  using Function = unsigned (*)(unsigned, unsigned);
  const auto run = std::bit_cast<Function>(code.address("graph_entry"));
  for (unsigned x = 0; x < nodes; ++x) {
    for (unsigned depth = 0; depth <= depth_limit; ++depth) {
      if (run(x, depth) != expected({.value = x, .depth = depth})) {
        throw std::runtime_error(
            "Prepared function-table traversal differs from independent model");
      }
    }
  }
  for (unsigned index = 0; index < export_names.size(); ++index) {
    const auto value = std::bit_cast<Function>(code.address(export_names[index]));
    if (value(0, 0) != index + 1) {
      throw std::runtime_error("Private native names changed a valid declared export");
    }
  }
  if (code.native_bytes() == 0) { throw std::runtime_error("Expected actual native code"); }
  for (const auto *const name : import_names) {
    bool rejected = false;
    try {
      static_cast<void>(code.address(name));
    } catch (const std::invalid_argument &) { rejected = true; }
    if (!rejected) { throw std::runtime_error("An unexported native import became public"); }
  }
}
}

int main() try {
  qualify();
  qualify();
  weak_namespace();
  std::cout << "Verified native dependency cycles, exact exports, unused imports, weak definitions "
               "and owner reuse.\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
