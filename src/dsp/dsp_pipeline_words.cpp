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
namespace {
std::size_t initial_cursor(const std::span<const DspFetchedWord> fetched,
                           const std::optional<DspFetchedWord> repeat) {
  if (!repeat) { return 0; }
  if (fetched.empty() || fetched.front() != repeat) {
    throw Fault("Repeated pipeline stage lost its actual retained F2 receipt read P:" +
                hex(repeat->word.address, 6));
  }
  return 1;
}

bool repeated_stage(const DspFetchedWord stage,
                    const std::optional<DspFetchedWord> repeat,
                    const std::optional<DspFetchedWord> extension) {
  if (!repeat || stage != *repeat) { return false; }
  if (pipeline_word_count(stage.word) != 1 || extension) {
    throw Fault("Repeated stage cannot invent an extension read P:" + hex(stage.word.address, 6));
  }
  return true;
}

DspFetchedWord adjacent(const std::span<const DspFetchedWord> fetched,
                        const std::size_t cursor,
                        const DspFetchedWord stage) {
  constexpr unsigned word_mask = 0xffffff;
  const auto address = (stage.word.address + 1) & word_mask;
  if (cursor >= fetched.size() || fetched[cursor].word.address != address ||
      fetched[cursor].occurrence == 0 || fetched[cursor].occurrence <= stage.occurrence) {
    throw Fault("Automatic DSP stage awaits adjacent actual F2 extension read P:" +
                hex(address, 6) + " pc=" + hex(stage.word.address, 6));
  }
  return fetched[cursor];
}
}

DspPipelineWords
select_pipeline_words(const std::span<const DspFetchedWord> fetched,
                      const std::array<std::optional<DspFetchedWord>, 3> &stages,
                      const std::array<std::optional<DspFetchedWord>, 2> &extensions,
                      const std::optional<DspFetchedWord> repeat) {
  DspPipelineWords result;
  auto cursor = initial_cursor(fetched, repeat);
  for (std::size_t reverse = stages.size(); reverse != 0; --reverse) {
    const auto stage = stages.at(reverse - 1);
    const auto extension = reverse > 1 ? extensions.at(reverse - 2) : std::nullopt;
    if (!stage) {
      if (extension) { throw Fault("Automatic DSP extension has no owning stage"); }
      continue;
    }
    if (repeated_stage(*stage, repeat, extension)) { continue; }
    if (cursor >= fetched.size() || fetched[cursor] != *stage || stage->occurrence == 0) {
      throw Fault("Automatic DSP stage lost actual F2 occurrence at read P:" +
                  hex(stage->word.address, 6) + " pc=" + hex(stage->word.address, 6));
    }
    ++cursor;
    if (pipeline_word_count(stage->word) == 1) {
      if (extension) {
        throw Fault("Unexpected DSP extension read P:" + hex(stage->word.address, 6));
      }
      continue;
    }
    const auto next = adjacent(fetched, cursor, *stage);
    if (reverse == 1) {
      result.ag1_extension = next;
    } else if (extension != next) {
      throw Fault("Automatic DSP extension differs from retained F2 occurrence pc=" +
                  hex(stage->word.address, 6));
    }
    ++cursor;
  }
  if (cursor < fetched.size()) { result.next = fetched[cursor]; }
  return result;
}
}
