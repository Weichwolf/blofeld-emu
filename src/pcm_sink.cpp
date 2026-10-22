// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/pcm_sink.hpp"

#include "blofeld/bus.hpp"

#include <memory>

namespace blofeld {
void PcmFrameSink::claim_source(const std::shared_ptr<const void> &identity) {
  if (!identity) { throw Fault("External PCM source requires a board identity"); }
  if (source_ && source_ != identity) {
    throw Fault("External PCM sink belongs to another board; create a new sink");
  }
  source_ = identity;
}
}
