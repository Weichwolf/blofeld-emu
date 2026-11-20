// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "io/coupling.hpp"

#include <dsp56300/core.hpp>
#include <optional>

namespace blofeld::vi::dsp {
class Io;

class Coupled {
public:
  Coupled(Coupling *coupling, Io &io, dsp56300::Core &core);
  ~Coupled();
  Coupled(const Coupled &) = delete;
  Coupled &operator=(const Coupled &) = delete;
  [[nodiscard]] bool service() const noexcept;
  [[nodiscard]] bool changed() const noexcept;
  [[nodiscard]] bool
  peer_outputs(PeerPort &peer, bool ready_known, bool ready_active) const noexcept;
  [[nodiscard]] static const Coupled *current() noexcept;

private:
  [[nodiscard]] bool changed(std::optional<Coupling::Stamp> stamp) const noexcept;

  Coupling *coupling_;
  Io &io_;
  dsp56300::Core &core_;
};
}
