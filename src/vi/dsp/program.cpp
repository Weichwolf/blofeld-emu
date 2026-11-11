// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/program.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <format>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace blofeld::vi::dsp {
namespace {
void validate_choice(const dsp56300::CodeVariant &variant, const ProgramChoice &choice) {
  for (const auto pc : choice.resumes) {
    if (std::ranges::find(variant.entries, pc) == variant.entries.end()) {
      throw std::invalid_argument("DSP program installation point is not an established entry");
    }
  }
  std::set<dsp56300::Word> addresses;
  for (const auto &word : choice.words) {
    if (word.address > dsp56300::word_mask || (word.value && *word.value > dsp56300::word_mask) ||
        !addresses.insert(word.address).second) {
      throw std::invalid_argument("Invalid DSP program word constraint");
    }
  }
}
}

Program::Program(dsp56300::NativeImage image,
                 const dsp56300::CodeImage source,
                 const std::span<const ProgramChoice> choices,
                 const std::span<const dsp56300::CodeTransition> transitions,
                 const std::span<const dsp56300::VariantLayout> memory)
    : image_(std::move(image)) {
  const auto ready = image_.report();
  if (ready.failure.kind != dsp56300::FaultKind::none || ready.prepared_entries == 0 ||
      ready.unresolved_entries != 0 || choices.empty() || source.variants.empty()) {
    throw std::invalid_argument(
        "DSP program requires complete prepared code and admission choices");
  }
  for (const auto &variant : source.variants) { source_variant(variant); }
  memory_variants(memory);
  std::set<std::uint64_t> covered;
  for (const auto &choice : choices) {
    const auto variant =
        std::ranges::find(source.variants, choice.variant, &dsp56300::CodeVariant::id);
    if (variant == source.variants.end() || choice.resumes.empty() ||
        choice.mode_mask > dsp56300::word_mask || (choice.mode & ~choice.mode_mask) != 0) {
      throw std::invalid_argument("Invalid DSP program admission choice");
    }
    validate_choice(*variant, choice);
    choices_.push_back({choice.variant,
                        {choice.resumes.begin(), choice.resumes.end()},
                        {choice.words.begin(), choice.words.end()},
                        choice.mode_mask,
                        choice.mode});
    covered.insert(choice.variant);
  }
  if (covered.size() != ranges_.size()) {
    throw std::invalid_argument("DSP program admission does not cover every prepared variant");
  }
  for (const auto &transfer : transitions) {
    const bool admitted = std::ranges::any_of(choices_, [&transfer](const auto &choice) {
      return choice.variant == transfer.to &&
             std::ranges::find(choice.resumes, transfer.entry) != choice.resumes.end();
    });
    if (!ranges_.contains(transfer.from) || transfer.from == transfer.to || !admitted ||
        !transitions_.emplace(std::pair{transfer.from, transfer.entry}, transfer.to).second) {
      throw std::invalid_argument("Invalid DSP program transition contract");
    }
  }
}

void Program::memory_variants(const std::span<const dsp56300::VariantLayout> memory) {
  for (const auto &item : memory) {
    if (!ranges_.contains(item.id) ||
        !layouts_
             .emplace(item.id,
                      std::vector<dsp56300::RegionLayout>(item.memory.begin(), item.memory.end()))
             .second) {
      throw std::invalid_argument("Invalid DSP program memory variant");
    }
    auto &regions = layouts_.at(item.id);
    if (std::ranges::any_of(
            regions, [](const auto &region) { return region.space == dsp56300::Space::program; })) {
      throw std::invalid_argument("Instrument DSP program requires external-memory layouts");
    }
    std::ranges::sort(regions, [](const auto &left, const auto &right) {
      return std::pair{left.space, left.first} < std::pair{right.space, right.first};
    });
  }
  if (!layouts_.empty() && layouts_.size() != ranges_.size()) {
    throw std::invalid_argument("DSP program memory does not cover every prepared variant");
  }
}

void Program::source_variant(const dsp56300::CodeVariant &variant) {
  const auto [found, added] = ranges_.try_emplace(variant.id);
  if (!added || variant.code.empty()) {
    throw std::invalid_argument("Invalid DSP program variant");
  }
  for (const auto &span : variant.code) {
    if (span.address > dsp56300::word_mask || span.words.empty() ||
        span.words.size() > dsp56300::address_words) {
      throw std::invalid_argument("Invalid DSP program source extent");
    }
    const auto head = std::min(span.words.size(), dsp56300::address_words - span.address);
    found->second.push_back({span.address, span.address + static_cast<dsp56300::Word>(head - 1)});
    if (head < span.words.size()) {
      found->second.push_back({0, static_cast<dsp56300::Word>(span.words.size() - head - 1)});
    }
  }

  auto &ranges = found->second;
  std::ranges::sort(ranges, {}, &Range::first);
  std::vector<Range> merged;
  for (const auto &range : ranges) {
    if (!merged.empty() && range.first <= merged.back().last + 1) {
      merged.back().last = std::max(merged.back().last, range.last);
    } else {
      merged.push_back(range);
    }
  }
  ranges = std::move(merged);
}

std::uint64_t Program::install(dsp56300::Core &core, dsp56300::Io &io) const {
  const auto registers = core.registers();
  std::optional<std::uint64_t> selected;
  for (const auto &choice : choices_) {
    if ((registers.omr & choice.mode_mask) != choice.mode ||
        std::ranges::find(choice.resumes, registers.pc) == choice.resumes.end() ||
        !layout(choice.variant, io) || !std::ranges::all_of(choice.words, [&io](const auto &word) {
          return io.inspect_program(word.address) == word.value;
        })) {
      continue;
    }
    if (selected && *selected != choice.variant) {
      throw std::runtime_error(std::format(
          "Ambiguous DSP image admission at P:${:06x}, OMR=${:06x}", registers.pc, registers.omr));
    }
    selected = choice.variant;
  }
  if (!selected) {
    throw std::runtime_error(std::format(
        "No prepared DSP image admits P:${:06x}, OMR=${:06x}", registers.pc, registers.omr));
  }
  core.install(image_, *selected);
  return *selected;
}

bool Program::layout(const std::uint64_t variant, dsp56300::Io &io) const {
  if (layouts_.empty()) { return true; }
  const auto &expected = layouts_.at(variant);
  for (const auto space : {dsp56300::Space::x, dsp56300::Space::y}) {
    const auto actual = io.memory_windows(space);
    if (std::ranges::count(expected, space, &dsp56300::RegionLayout::space) !=
        static_cast<std::ptrdiff_t>(actual.size())) {
      return false;
    }
    std::size_t index = 0;
    for (const auto &region : expected) {
      if (region.space != space) { continue; }
      const auto &view = actual[index++];
      if (view.address != region.first || view.words.size() != region.size ||
          view.writable != region.writable || view.aliases_program != region.aliases_program ||
          (!view.known.empty()) != region.tracked) {
        return false;
      }
    }
  }
  return true;
}

bool Program::contains(const ProgramPoint point) const noexcept {
  const auto found = ranges_.find(point.variant);
  if (found == ranges_.end()) { return false; }
  return std::ranges::any_of(found->second, [point](const auto &range) {
    return point.address >= range.first && point.address <= range.last;
  });
}

bool Program::transition(const ProgramPoint point) const {
  return transitions_.contains({point.variant, point.address});
}
}
