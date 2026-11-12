// SPDX-License-Identifier: GPL-3.0-only
#include "aot/link_graph.hpp"

#if defined(BLOFELD_CENSUS)
#include "process/configuration.hpp"

#include "llvm/ADT/StringMap.h"
#include "llvm/Support/FormatVariadic.h"
#include "llvm/Support/JSON.h"

#include <chrono>
#include <fstream>
#include <ios>
#include <mutex>
#endif

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSet.h"
#include "llvm/ExecutionEngine/JITLink/JITLink.h"
#include "llvm/ExecutionEngine/JITSymbol.h"
#include "llvm/ExecutionEngine/Orc/Core.h"
#include "llvm/ExecutionEngine/Orc/LLJIT.h"
#include "llvm/ExecutionEngine/Orc/ObjectLinkingLayer.h"
#include "llvm/ExecutionEngine/Orc/Shared/OrcError.h"
#include "llvm/ExecutionEngine/Orc/SymbolStringPool.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/Error.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace blofeld::host::aot::detail {
namespace {
using Symbol = llvm::jitlink::Symbol;
using Responsibility = llvm::orc::MaterializationResponsibility;

struct Definition {
  Symbol *symbol;
  llvm::orc::SymbolStringPtr name;
};

llvm::StringSet<> graph_names(llvm::jitlink::LinkGraph &graph) {
  llvm::StringSet<> result;
  const auto remember = [&](const auto &symbols) {
    for (const auto *symbol : symbols) {
      if (symbol->hasName()) { result.insert(symbol->getName()); }
    }
  };
  remember(graph.defined_symbols());
  remember(graph.external_symbols());
  remember(graph.absolute_symbols());
  return result;
}

#if defined(BLOFELD_CENSUS)
using Labels = llvm::StringMap<std::string>;

llvm::Error
record_symbols(llvm::jitlink::LinkGraph &graph, const Labels &labels, const std::string &path) {
  static std::mutex mutex;
  const std::scoped_lock locked(mutex);
  std::ofstream out(path, std::ios::app);
  const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
  for (const auto *symbol : graph.defined_symbols()) {
    if (!symbol->hasName() || !symbol->isCallable() || symbol->getSize() == 0) { continue; }
    const auto found = labels.find(symbol->getName());
    const auto original = found == labels.end() ? symbol->getName().str() : found->second;
    llvm::json::Object record{{.K = "ns", .V = stamp},
                              {.K = "address", .V = symbol->getAddress().getValue()},
                              {.K = "size", .V = symbol->getSize()},
                              {.K = "name", .V = original},
                              {.K = "linked", .V = symbol->getName().str()}};
    out << llvm::formatv("{0}", llvm::json::Value(std::move(record))).str() << '\n';
  }
  out.flush();
  if (!out) {
    return llvm::make_error<llvm::StringError>("Cannot write native census symbols: " + path,
                                               llvm::inconvertibleErrorCode());
  }
  return llvm::Error::success();
}
#endif

class Dependencies final : public llvm::orc::ObjectLinkingLayer::Plugin {
public:
  void modifyPassConfig(Responsibility &owner,
                        llvm::jitlink::LinkGraph & /*graph*/,
                        llvm::jitlink::PassConfiguration &config) override {
#if defined(BLOFELD_CENSUS)
    auto path = std::string(host::inherited_setting("BLOFELD_CENSUS_SYMBOLS"));
    if (!path.empty()) {
      auto labels = std::make_shared<Labels>();
      config.PostPrunePasses.emplace_back(
          [this, &owner, labels](llvm::jitlink::LinkGraph &graph) -> llvm::Error {
            std::vector<std::pair<Symbol *, std::string>> original;
            for (auto *symbol : graph.defined_symbols()) {
              if (symbol->hasName()) { original.emplace_back(symbol, symbol->getName().str()); }
            }
            if (auto error = define(owner, graph)) { return error; }
            for (const auto &[symbol, name] : original) { (*labels)[symbol->getName()] = name; }
            return llvm::Error::success();
          });
      config.PostFixupPasses.emplace_back(
          [labels, path = std::move(path)](llvm::jitlink::LinkGraph &graph) {
            return record_symbols(graph, *labels, path);
          });
      return;
    }
#endif
    config.PostPrunePasses.emplace_back(
        [this, &owner](llvm::jitlink::LinkGraph &graph) { return define(owner, graph); });
  }

  llvm::Error notifyFailed(Responsibility & /*owner*/) override { return llvm::Error::success(); }

  llvm::Error notifyRemovingResources(llvm::orc::JITDylib & /*dylib*/,
                                      llvm::orc::ResourceKey /*key*/) override {
    return llvm::Error::success();
  }

  void notifyTransferringResources(llvm::orc::JITDylib & /*dylib*/,
                                   llvm::orc::ResourceKey /*destination*/,
                                   llvm::orc::ResourceKey /*source*/) override {}

private:
  llvm::Error name(llvm::StringSet<> &occupied,
                   llvm::orc::ExecutionSession &session,
                   llvm::orc::SymbolStringPtr &result) {
    std::string candidate;
    do {
      const auto index = next_.fetch_add(1, std::memory_order_relaxed);
      if (index == std::numeric_limits<std::uint64_t>::max()) {
        return llvm::make_error<llvm::StringError>("Native symbol namespace exhausted",
                                                   llvm::inconvertibleErrorCode());
      }
      candidate = "__blofeld_private_graph_" + std::to_string(index);
    } while (!occupied.insert(candidate).second);
    result = session.intern(candidate);
    return llvm::Error::success();
  }

  llvm::Error claim(Responsibility &owner,
                    llvm::jitlink::LinkGraph &graph,
                    std::vector<Definition> &definitions) {
    auto occupied = graph_names(graph);
    auto &session = owner.getTargetJITDylib().getExecutionSession();
    for (auto *symbol : graph.defined_symbols()) {
      if (symbol->getScope() != llvm::jitlink::Scope::Local ||
          symbol->getLinkage() != llvm::jitlink::Linkage::Strong) {
        continue;
      }
      llvm::orc::SymbolStringPtr interned;
      if (auto error = name(occupied, session, interned)) { return error; }
      definitions.push_back({.symbol = symbol, .name = std::move(interned)});
    }
    while (true) {
      llvm::orc::SymbolFlagsMap symbols;
      for (const auto &definition : definitions) {
        symbols[definition.name] = definition.symbol->isCallable() ? llvm::JITSymbolFlags::Callable
                                                                   : llvm::JITSymbolFlags::None;
      }
      auto error = owner.defineMaterializing(std::move(symbols));
      if (!error) { return llvm::Error::success(); }
      std::string duplicate;
      // LLVM19 rolls back the whole claim on any existing definition, including weak ones.
      error =
          llvm::handleErrors(std::move(error), [&](const llvm::orc::DuplicateDefinition &failure) {
            duplicate = failure.getSymbolName();
          });
      if (error) { return error; }
      const auto found = std::ranges::find_if(
          definitions, [&](const auto &definition) { return *definition.name == duplicate; });
      if (found == definitions.end()) {
        return llvm::make_error<llvm::orc::DuplicateDefinition>(std::move(duplicate));
      }
      if (auto failure = name(occupied, session, found->name)) { return failure; }
    }
  }

  llvm::Error define(Responsibility &owner, llvm::jitlink::LinkGraph &graph) {
    std::vector<Definition> definitions;
    if (auto error = claim(owner, graph, definitions)) { return error; }
    // Keep relocations intact; let ORC resolve cycles at symbol granularity.
    for (const auto &definition : definitions) {
      const auto name = *definition.name;
      const auto storage = graph.allocateContent(llvm::ArrayRef<char>(name.data(), name.size()));
      definition.symbol->setName(llvm::StringRef(storage.data(), storage.size()));
      definition.symbol->setScope(llvm::jitlink::Scope::Hidden);
    }
    return llvm::Error::success();
  }

  std::atomic<std::uint64_t> next_;
};
}

void named_dependencies(llvm::orc::LLJIT &jit) {
  if (auto *layer = llvm::dyn_cast<llvm::orc::ObjectLinkingLayer>(&jit.getObjLinkingLayer())) {
    layer->addPlugin(std::make_shared<Dependencies>());
  }
}
}
