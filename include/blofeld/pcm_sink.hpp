// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/serial_pcm.hpp"

namespace blofeld {
// External output, outside modeled state. The sink outlives its board binding.
// Calls occur on the emulation owner thread; hardware faults may throw there.
class PcmFrameSink {
public:
  virtual ~PcmFrameSink() = default;
  virtual void accept(const SerialPcmFrame &frame) = 0;
  virtual void check() const = 0;
  // A restore/fault cannot replay or erase audio already sent to the host.
  // Repeated invalidation is permitted.
  virtual void invalidate() noexcept = 0;
};
}
