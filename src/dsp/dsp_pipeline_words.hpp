// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/detail/dsp_program_fetch.hpp"

#include <array>
#include <optional>
#include <span>

namespace blofeld {
struct DspPipelineWords {
  std::optional<DspFetchedWord> next;
  std::optional<DspFetchedWord> ag1_extension;
};

// Validate the actual oldest-to-youngest queue against E1/AG2/AG1 receipts.
// Equal addresses/values do not identify the same F2 occurrence.
[[nodiscard]] DspPipelineWords
select_pipeline_words(std::span<const DspFetchedWord> fetched,
                      const std::array<std::optional<DspFetchedWord>, 3> &stages,
                      const std::array<std::optional<DspFetchedWord>, 2> &extensions);
}
