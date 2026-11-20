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
#include <optional>
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
#if defined(BLOFELD_CENSUS)
    io_.trace(Census::Event::coupling_service);
#endif
    core_.request_yield();
  }
  return accepted;
}

bool Coupled::changed() const noexcept {
  return changed(io_.timestamp());
}

bool Coupled::peer_outputs(PeerPort &peer,
                           const bool ready_known,
                           const bool ready_active) const noexcept {
  // Both observations occur within one DSP access at the same guest time.
  const auto stamp = io_.timestamp();
  static_cast<void>(changed(stamp));
  return ready_known && peer.dsp_ready(ready_active) && changed(stamp);
}

bool Coupled::changed(const std::optional<Coupling::Stamp> stamp) const noexcept {
  const auto before = coupling_->revision();
  const bool accepted = stamp && coupling_->outputs(stamp->ceil());
  if (!accepted || coupling_->revision() != before) {
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

bool blofeld_dsp_peer_publish(PeerPort *const channel,
                              const PeerPort::Sample sample,
                              const bool ready_known,
                              const bool ready_active) noexcept {
  channel->dsp_data(sample);
  if (const auto *coupled = Coupled::current()) {
    return coupled->peer_outputs(*channel, ready_known, ready_active);
  }
  return ready_known && channel->dsp_ready(ready_active);
}

bool blofeld_dsp_peer_write(PeerPort *const channel, const std::uint64_t ticks) noexcept {
  return channel->dsp_write(channel->dsp_data(), ticks);
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
