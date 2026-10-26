// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_pipeline_words.hpp"

#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <span>

namespace blofeld {
DspPipelineWords
select_pipeline_words(const std::span<const DspFetchedWord> fetched,
                      const std::array<std::optional<DspFetchedWord>, 3> &stages,
                      const std::array<std::optional<DspFetchedWord>, 2> &extensions) {
  constexpr unsigned word_mask = 0xffffff;
  DspPipelineWords result;
  std::size_t cursor = 0;
  for (std::size_t reverse = stages.size(); reverse != 0; --reverse) {
    const auto stage = stages.at(reverse - 1);
    if (!stage) {
      if (reverse > 1 && extensions.at(reverse - 2)) {
        throw Fault("Automatic DSP extension has no owning stage");
      }
      continue;
    }
    if (cursor >= fetched.size() || fetched[cursor] != *stage || stage->occurrence == 0) {
      throw Fault("Automatic DSP stage lost actual F2 occurrence at read P:" +
                  hex(stage->word.address, 6) + " pc=" + hex(stage->word.address, 6));
    }
    ++cursor;
    if (pipeline_word_count(stage->word) == 1) {
      if (reverse > 1 && extensions.at(reverse - 2)) {
        throw Fault("Unexpected automatic DSP extension receipt pc=" + hex(stage->word.address, 6));
      }
      continue;
    }
    const auto address = (stage->word.address + 1) & word_mask;
    if (cursor >= fetched.size() || fetched[cursor].word.address != address ||
        fetched[cursor].occurrence == 0 || fetched[cursor].occurrence <= stage->occurrence) {
      throw Fault("Automatic DSP stage awaits adjacent actual F2 extension read P:" +
                  hex(address, 6) + " pc=" + hex(stage->word.address, 6));
    }
    if (reverse == 1) { result.ag1_extension = fetched[cursor]; }
    if (reverse > 1 && extensions.at(reverse - 2) != fetched[cursor]) {
      throw Fault("Automatic DSP extension differs from retained F2 occurrence pc=" +
                  hex(stage->word.address, 6));
    }
    ++cursor;
  }
  if (cursor < fetched.size()) { result.next = fetched[cursor]; }
  return result;
}
}
