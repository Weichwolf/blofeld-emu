// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/serial_pcm.hpp"

#include <memory>

namespace blofeld {
// External output, outside modeled state. The sink outlives its board binding.
// Calls occur on the emulation owner thread; hardware faults may throw there.
class PcmFrameSink {
public:
  PcmFrameSink() = default;
  virtual ~PcmFrameSink() = default;
  PcmFrameSink(const PcmFrameSink &) = delete;
  PcmFrameSink &operator=(const PcmFrameSink &) = delete;
  PcmFrameSink(PcmFrameSink &&) = delete;
  PcmFrameSink &operator=(PcmFrameSink &&) = delete;
  // Ordinary owner-thread admission. Retain identity to prevent address reuse;
  // a live external stream cannot serve a second modeled source.
  void claim_source(const std::shared_ptr<const void> &identity);
  virtual void accept(const SerialPcmFrame &frame) = 0;
  virtual void check() const = 0;
  // A restore/fault cannot replay or erase audio already sent to the host.
  // Repeated invalidation is permitted.
  virtual void invalidate() noexcept = 0;

private:
  std::shared_ptr<const void> source_;
};
}
