// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <dsp56300/core.hpp>

namespace blofeld::vi::dsp {
class Io;

struct Foreground {
  void *context = nullptr;
  const dsp56300::ForegroundCommit *(*offer)(void *,
                                             Io &,
                                             const dsp56300::Core &,
                                             const dsp56300::ForegroundGrant &) noexcept = nullptr;
};
}
