// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp56kEmu/dsp.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace blofeld {
struct DspAddressPosition {
  std::size_t slot = 0;
  std::uint64_t serial = 0;
};

struct DspAddressRecord {
  DspAddressStageView view;
  DspFetchedInstruction fetched{};
  dsp56k::DSP::ReferenceAttemptState source;
  std::uint64_t serial = 0;
};

// Host bound for AG1/AG2/E1 continuation records, not physical queue depth.
struct DspAddressStages {
  std::array<std::optional<DspAddressRecord>, 3> records;
  std::uint64_t serial = 0;
  [[nodiscard]] std::size_t capture(dsp56k::DSP &cpu, const DspFetchedInstruction &fetched);
  [[nodiscard]] const DspAddressRecord &get(DspAddressPosition position) const;
  [[nodiscard]] DspAddressRecord &get(DspAddressPosition position);
};
}
