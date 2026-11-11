// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <map>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace blofeld::vi::dsp {
struct ProgramPoint {
  std::uint64_t variant;
  dsp56300::Word address;
};

struct ProgramWord {
  dsp56300::Word address;
  std::optional<dsp56300::Word> value;
};

struct ProgramChoice {
  std::uint64_t variant;
  std::span<const dsp56300::Word> resumes;
  std::span<const ProgramWord> words;
  dsp56300::Word mode_mask = 0, mode = 0;
};

// Immutable prepared code and explicit admission contracts, shared across resets.
// Mutable RAM and all loading effects remain with the executing instrument.
class Program {
public:
  Program(dsp56300::NativeImage image,
          dsp56300::CodeImage source,
          std::span<const ProgramChoice> choices,
          std::span<const dsp56300::CodeTransition> transitions,
          std::span<const dsp56300::VariantLayout> memory = {});
  Program(const Program &) = delete;
  Program &operator=(const Program &) = delete;
  Program(Program &&) noexcept = default;
  Program &operator=(Program &&) noexcept = default;

  [[nodiscard]] std::uint64_t install(dsp56300::Core &core, dsp56300::Io &io) const;
  [[nodiscard]] bool contains(ProgramPoint point) const noexcept;
  [[nodiscard]] bool transition(ProgramPoint point) const;

  [[nodiscard]] dsp56300::PreparationReport report() const noexcept { return image_.report(); }

private:
  struct Range {
    dsp56300::Word first, last;
  };

  struct Choice {
    std::uint64_t variant;
    std::vector<dsp56300::Word> resumes;
    std::vector<ProgramWord> words;
    dsp56300::Word mode_mask, mode;
  };

  void memory_variants(std::span<const dsp56300::VariantLayout> memory);
  void source_variant(const dsp56300::CodeVariant &variant);
  [[nodiscard]] bool layout(std::uint64_t variant, dsp56300::Io &io) const;
  dsp56300::NativeImage image_;
  std::map<std::uint64_t, std::vector<Range>> ranges_;
  std::vector<Choice> choices_;
  std::map<std::uint64_t, std::vector<dsp56300::RegionLayout>> layouts_;
  std::map<std::pair<std::uint64_t, dsp56300::Word>, std::uint64_t> transitions_;
};
}
