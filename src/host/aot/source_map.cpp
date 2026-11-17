// SPDX-License-Identifier: GPL-3.0-only
#include "aot/source_map.hpp"

#include "aot/code.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <llvm/ADT/StringRef.h>
#include <llvm/DebugInfo/DIContext.h>
#include <llvm/DebugInfo/DWARF/DWARFContext.h>
#include <llvm/DebugInfo/DWARF/DWARFDebugLine.h>
#include <llvm/DebugInfo/DWARF/DWARFUnit.h>
#include <llvm/ExecutionEngine/JITLink/JITLink.h>
#include <llvm/ExecutionEngine/Orc/Core.h>
#include <llvm/ExecutionEngine/Orc/LLJIT.h>
#include <llvm/ExecutionEngine/Orc/ObjectLinkingLayer.h>
#include <llvm/ExecutionEngine/Orc/Shared/MemoryFlags.h>
#include <llvm/Object/ObjectFile.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/Error.h>
#include <llvm/Support/MemoryBufferRef.h>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace blofeld::host::aot::detail {
namespace {
using Graph = llvm::jitlink::LinkGraph;
using Responsibility = llvm::orc::MaterializationResponsibility;

struct Section {
  std::string name;
  std::uint64_t index, address, size;
};

struct Line {
  std::uint64_t section, address, size, variant;
  std::uint32_t pc;
};

struct Input {
  std::vector<Section> sections;
  std::vector<Line> lines;
  std::string failure;
};

struct Block {
  const llvm::jitlink::Block *block;
  std::uint64_t section, original;
};

bool executable(const llvm::jitlink::Section &section) {
  return (static_cast<unsigned>(section.getMemProt()) &
          static_cast<unsigned>(llvm::orc::MemProt::Exec)) != 0;
}

std::optional<std::uint64_t> variant(llvm::StringRef file) {
  if (!file.consume_front("core-variant-") || !file.consume_back(".S")) { return {}; }
  std::uint64_t result = 0;
  if (file.empty() || file.getAsInteger(10, result)) {
    throw std::runtime_error("Native source has an invalid variant identity");
  }
  return result;
}

void lines(Input &input, const llvm::DWARFDebugLine::LineTable &table) {
  for (std::size_t index = 1; index < table.Rows.size(); ++index) {
    const auto &row = table.Rows[index - 1];
    const auto &next = table.Rows[index];
    if (row.EndSequence || row.Line == 0 || row.Address.SectionIndex != next.Address.SectionIndex ||
        row.Address.Address >= next.Address.Address) {
      continue;
    }
    std::string file;
    if (!table.getFileNameByIndex(
            row.File, "", llvm::DILineInfoSpecifier::FileLineInfoKind::RawValue, file)) {
      throw std::runtime_error("Native source lacks its declared file");
    }
    const auto id = variant(file);
    if (!id) { continue; }
    input.lines.push_back({.section = row.Address.SectionIndex,
                           .address = row.Address.Address,
                           .size = next.Address.Address - row.Address.Address,
                           .variant = *id,
                           .pc = row.Line - 1});
  }
}

void resolve(Input &input) {
  for (auto &line : input.lines) {
    if (line.section != llvm::object::SectionedAddress::UndefSection) { continue; }
    const Section *match = nullptr;
    for (const auto &section : input.sections) {
      if (line.address < section.address || line.address - section.address >= section.size ||
          line.size > section.size - (line.address - section.address)) {
        continue;
      }
      if (match != nullptr) { throw std::runtime_error("Native source section is ambiguous"); }
      match = &section;
    }
    if (match == nullptr) { throw std::runtime_error("Native source has no executable section"); }
    line.section = match->index;
  }
  std::ranges::sort(input.lines, [](const Line &left, const Line &right) {
    return std::tie(left.section, left.address) < std::tie(right.section, right.address);
  });
}

Input parse(const llvm::MemoryBufferRef bytes) {
  auto object = llvm::object::ObjectFile::createObjectFile(bytes);
  if (!object) { throw std::runtime_error(llvm::toString(object.takeError())); }
  Input result;
  for (const auto section : (*object)->sections()) {
    if (!section.isText()) { continue; }
    auto name = section.getName();
    if (!name) { throw std::runtime_error(llvm::toString(name.takeError())); }
    result.sections.push_back({.name = name->str(),
                               .index = section.getIndex(),
                               .address = section.getAddress(),
                               .size = section.getSize()});
  }
  const auto failure = [&](llvm::Error error) {
    result.failure += llvm::toString(std::move(error));
  };
  const auto dwarf =
      llvm::DWARFContext::create(**object,
                                 llvm::DWARFContext::ProcessDebugRelocations::Process,
                                 nullptr,
                                 "",
                                 failure,
                                 failure);
  for (const auto &unit : dwarf->compile_units()) {
    if (const auto *table = dwarf->getLineTableForUnit(unit.get())) { lines(result, *table); }
  }
  resolve(result);
  return result;
}

std::vector<Block> blocks(const Input &input, Graph &graph) {
  std::vector<Block> result;
  for (const auto &section : graph.sections()) {
    if (!executable(section)) { continue; }
    // Mach-O graph names include the segment, object section names do not.
    const auto original = section.getName();
    const auto comma = original.rfind(',');
    const auto name = comma == llvm::StringRef::npos ? original : original.substr(comma + 1);
    const auto found = std::ranges::find(input.sections, name.str(), &Section::name);
    if (found == input.sections.end()) { continue; }
    for (const auto *block : section.blocks()) {
      result.push_back(
          {.block = block, .section = found->index, .original = block->getAddress().getValue()});
    }
  }
  return result;
}

void link(SourceMap &output, const Input &input, const std::vector<Block> &blocks, Graph &graph) {
  const std::lock_guard lock(output.mutation);
  for (const auto &section : graph.sections()) {
    if (!executable(section)) { continue; }
    for (const auto *block : section.blocks()) {
      if (block->getSize() != 0) {
        output.text.push_back(
            {.address = block->getAddress().getValue(), .size = block->getSize()});
      }
    }
  }
  for (const auto &block : blocks) {
    const auto address = block.block->getAddress().getValue();
    const auto size = block.block->getSize();
    if (size == 0) { continue; }
    auto line = std::ranges::partition_point(input.lines, [&](const Line &site) {
      return std::tie(site.section, site.address) < std::tie(block.section, block.original);
    });
    if (line != input.lines.begin() && (line - 1)->section == block.section) { --line; }
    for (; line != input.lines.end() && line->section == block.section &&
           line->address < block.original + size;
         ++line) {
      const auto first = std::max(line->address, block.original);
      const auto last = std::min(line->address + line->size, block.original + size);
      if (first < last) {
        output.sites.push_back({.address = address + first - block.original,
                                .size = last - first,
                                .variant = line->variant,
                                .guest_pc = line->pc});
      }
    }
  }
  std::ranges::sort(output.text, {}, &Code::TextRange::address);
  std::ranges::sort(output.sites, {}, &Code::Site::address);
}

class Sites final : public llvm::orc::ObjectLinkingLayer::Plugin {
public:
  explicit Sites(SourceMap &output) : output_(output) {}

  void notifyMaterializing(Responsibility & /*owner*/,
                           Graph &graph,
                           llvm::jitlink::JITLinkContext & /*context*/,
                           const llvm::MemoryBufferRef object) override {
    // LLVM19's input-object hook is needed before pruning and section relocation.
    auto input = std::make_shared<Input>();
    try {
      *input = parse(object);
    } catch (const std::exception &error) { input->failure = error.what(); }
    const std::lock_guard lock(mutation_);
    inputs_.emplace(&graph, std::move(input));
  }

  void modifyPassConfig(Responsibility & /*owner*/,
                        Graph &materializing,
                        llvm::jitlink::PassConfiguration &config) override {
    auto found = [&] {
      const std::lock_guard lock(mutation_);
      return inputs_.extract(&materializing);
    }();
    if (found.empty()) {
      config.PostPrunePasses.emplace_back([](Graph & /*graph*/) -> llvm::Error {
        return llvm::make_error<llvm::StringError>("JITLink lost its native source object",
                                                   llvm::inconvertibleErrorCode());
      });
      return;
    }
    auto input = std::move(found.mapped());
    auto retained = std::make_shared<std::vector<Block>>();
    config.PostPrunePasses.emplace_back([input, retained](Graph &graph) -> llvm::Error {
      if (!input->failure.empty()) {
        return llvm::make_error<llvm::StringError>(input->failure, llvm::inconvertibleErrorCode());
      }
      *retained = blocks(*input, graph);
      return llvm::Error::success();
    });
    config.PostFixupPasses.emplace_back([this, input, retained](Graph &graph) {
      link(output_, *input, *retained, graph);
      return llvm::Error::success();
    });
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
  SourceMap &output_;
  std::mutex mutation_;
  std::map<const Graph *, std::shared_ptr<Input>> inputs_;
};
}

void SourceMap::attach(llvm::orc::LLJIT &jit) {
  auto *layer = llvm::dyn_cast<llvm::orc::ObjectLinkingLayer>(&jit.getObjLinkingLayer());
  if (layer == nullptr) { throw std::runtime_error("Native source attribution requires JITLink"); }
  layer->addPlugin(std::make_shared<Sites>(*this));
}

Code::FaultMap SourceMap::map() const noexcept {
  return {.text_ranges = text, .sites = sites};
}
}
