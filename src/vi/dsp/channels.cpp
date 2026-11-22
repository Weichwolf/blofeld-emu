// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/channels.hpp"

#if defined(BLOFELD_CENSUS)
#include "census.hpp"
#endif

#include "dsp/coupling.hpp"
#include "dsp/io.hpp"
#include "io/coupling.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"

#include <cstdint>
#include <dsp56300/core.hpp>
#include <stdexcept>

namespace blofeld::vi::dsp {
namespace {
thread_local const Coupled *current_coupling = nullptr;
}

Coupled::Coupled(Coupling *coupling, Io &io, dsp56300::Core &core)
    : coupling_(coupling), io_(io), core_(core) {
  if (coupling == nullptr) { return; }
  if (current_coupling != nullptr) { throw std::logic_error("Nested DSP coupling owner"); }
  current_coupling = this;
}

Coupled::~Coupled() {
  if (coupling_ != nullptr) { current_coupling = nullptr; }
}

const Coupled *Coupled::current() noexcept {
  return current_coupling;
}

bool Coupled::service() const noexcept {
  const auto before = coupling_->revision();
  const auto stamp = io_.timestamp();
  const bool accepted = stamp && coupling_->service(*stamp) && !io_.coupling_interrupted();
  if (!accepted || coupling_->revision() != before) {
    io_.invalidate_peer_read();
#if defined(BLOFELD_CENSUS)
    io_.trace(Census::Event::coupling_service);
#endif
    core_.request_yield();
  }
  return accepted;
}

bool Coupled::changed(const bool peer_data_unchanged) const noexcept {
  const auto before = coupling_->revision();
  const auto stamp = io_.timestamp();
  // A GPIO data write can leave the published inputs unchanged. An
  // active reply still needs sampling because the MCU can finish it.
  const bool accepted = stamp && ((peer_data_unchanged && coupling_->reply_idle())
                                      ? !coupling_->stopped()
                                      : coupling_->outputs(stamp->ceil()));
  if (!accepted || coupling_->revision() != before) {
    io_.invalidate_peer_read();
#if defined(BLOFELD_CENSUS)
    io_.trace(Census::Event::coupling_output);
#endif
    core_.request_yield();
  }
  return accepted;
}

bool blofeld_dsp_shi_read(Shi *const channel,
                          const std::uint32_t address,
                          std::uint32_t &value) noexcept {
  if (const auto *coupled = Coupled::current(); coupled != nullptr && !coupled->service()) {
    return false;
  }
  return channel->read(address, value);
}

bool blofeld_dsp_shi_write(Shi *const channel,
                           const std::uint32_t address,
                           const std::uint32_t value) noexcept {
  if (const auto *coupled = Coupled::current(); coupled != nullptr && !coupled->service()) {
    return false;
  }
  const bool accepted = channel->write({.address = address, .value = value});
  if (accepted) {
    if (const auto *coupled = Coupled::current()) { return coupled->changed(); }
  }
  return accepted;
}

bool blofeld_dsp_peer_sample(PeerPort *const channel,
                             const std::uint64_t ticks,
                             PeerPort::Sample &sample) noexcept {
  if (const auto *coupled = Coupled::current(); coupled != nullptr && !coupled->service()) {
    return false;
  }
  sample = channel->sample_mcu_data(ticks);
  return true;
}

bool blofeld_dsp_peer_ports(PeerPort *const channel,
                            const std::uint8_t mask,
                            const std::uint8_t known,
                            const std::uint8_t value,
                            const bool ready_known,
                            const bool active) noexcept {
  const bool data_changed = channel->dsp_data({.mask = mask, .known = known, .value = value});
  const auto *coupled = Coupled::current();
  const bool observed = coupled != nullptr && coupled->changed(!data_changed);
  if (!ready_known || !channel->dsp_ready(active)) { return false; }
  return coupled == nullptr || (observed && coupled->ready_unchanged(active)) || coupled->changed();
}

bool blofeld_dsp_peer_write(PeerPort *const channel, const std::uint64_t ticks) noexcept {
  return channel->dsp_write(channel->dsp_data(), ticks);
}
}
