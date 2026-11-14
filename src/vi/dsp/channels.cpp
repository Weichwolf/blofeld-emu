// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/channels.hpp"

#include "io/execution_time.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"

#include <cstdint>

namespace blofeld::vi::dsp {
bool blofeld_dsp_shi_read(Shi *const channel,
                          const std::uint32_t address,
                          std::uint32_t &value) noexcept {
  return channel->read(address, value);
}

bool blofeld_dsp_shi_write(Shi *const channel,
                           const std::uint32_t address,
                           const std::uint32_t value) noexcept {
  return channel->write({.address = address, .value = value});
}

void blofeld_dsp_peer_sample(PeerPort *const channel,
                             const std::uint64_t ticks,
                             PeerPort::Sample &sample) noexcept {
  sample = channel->sample_mcu_data(ticks);
}

void blofeld_dsp_peer_data(PeerPort *const channel,
                           const std::uint8_t mask,
                           const std::uint8_t known,
                           const std::uint8_t value) noexcept {
  channel->dsp_data({.mask = mask, .known = known, .value = value});
}

bool blofeld_dsp_peer_write(PeerPort *const channel, const std::uint64_t ticks) noexcept {
  return channel->dsp_write(channel->dsp_data(), ticks);
}

bool blofeld_dsp_peer_ready(PeerPort *const channel, const bool active) noexcept {
  return channel->dsp_ready(active);
}

bool blofeld_dsp_time_wake(ExecutionTime *const channel, const std::uint64_t ticks) noexcept {
  return channel->wake_mcu(ticks);
}

bool blofeld_dsp_peer_interrupt(const PeerPort *const channel,
                                const bool edge,
                                std::uint64_t &generation) noexcept {
  const auto pending = channel->dsp_interrupt(edge);
  if (!pending) { return false; }
  generation = *pending;
  return true;
}
}
