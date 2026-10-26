// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

namespace blofeld {
struct DspPeripheralReadPoint {
  std::uint32_t address = 0;
  std::uint64_t epoch = 0;
};

// A grant for one actual captured port response, never a supplied data word.
class DspPeripheralReadTicket {
public:
  DspPeripheralReadTicket() = default;
  bool operator==(const DspPeripheralReadTicket &) const = default;

private:
  friend class DspCore;
  std::shared_ptr<const bool> owner_;
  // Capture identity survives a trusted restore; a new post-restore capture uses
  // the new owner's identity, so a discarded future cannot alias a reused slot.
  std::shared_ptr<const bool> origin_;
  std::size_t slot_ = 0;
  std::uint64_t serial_ = 0;
};
}
