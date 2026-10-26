// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_peripheral_read.hpp"
#include "dsp/dsp_peripheral_read_stage.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
struct DspPeripheralReadRecord {
  DspPeripheralRead sample;
  DspPeripheralReadPoint point;
  std::shared_ptr<const bool> origin;
  std::uint64_t serial = 0;
  bool automatic = false;
};

struct DspPeripheralReadPosition {
  std::shared_ptr<const bool> origin;
  std::size_t slot = 0;
  std::uint64_t serial = 0;
};

struct DspPeripheralReadStages {
  // A host bound, not a measured physical pipeline queue depth.
  std::array<std::optional<DspPeripheralReadRecord>, 3> records;
  std::uint64_t serial = 0;
  std::optional<std::uint64_t> last_store_cycle = std::nullopt;
};
}
