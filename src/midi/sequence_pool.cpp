// SPDX-License-Identifier: GPL-3.0-only
#include "midi/sequence_pool.hpp"

#include "common/bus.hpp"

#include <algorithm>
#include <functional>

namespace blofeld::detail {
namespace {
int usable(const int granted) {
  if (granted < 2) { throw Fault("Host MIDI event pool cannot hold a SysEx record and payload"); }
  return granted;
}
}

int negotiate_sequence_pool(const int preferred,
                            const int current,
                            const std::function<int(int)> &request) {
  if (preferred < 2 || current < 1 || !request) {
    throw Fault("Invalid host MIDI pool negotiation");
  }
  auto granted = request(preferred);
  if (granted < 1) { throw Fault("Host MIDI backend returned an invalid pool grant"); }
  if (granted == preferred || granted != current) { return usable(granted); }
  auto low = std::min(current, preferred);
  auto high = preferred - 1;
  while (low < high) {
    const auto candidate = low + ((high - low + 1) / 2);
    const auto actual = request(candidate);
    if (actual < 1) { throw Fault("Host MIDI backend returned an invalid pool grant"); }
    if (actual == candidate) {
      low = candidate;
    } else if (actual != granted) {
      return usable(actual);
    } else {
      high = candidate - 1;
    }
    granted = actual;
  }
  return usable(granted);
}
}
