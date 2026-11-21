// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "io/coupling.hpp"

#include <dsp56300/core.hpp>

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

  [[nodiscard]] bool ready_unchanged(const bool active) const noexcept {
    return coupling_->ready_unchanged(active);
  }

  [[nodiscard]] static const Coupled *current() noexcept;

private:
  Coupling *coupling_;
  Io &io_;
  dsp56300::Core &core_;
};
}
