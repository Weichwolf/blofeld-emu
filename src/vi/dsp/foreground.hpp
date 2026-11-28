// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>

namespace blofeld::vi::dsp {
class Io;

struct Foreground {
  void *context = nullptr;
  const dsp56300::ForegroundCommit *(*offer)(void *,
                                             Io &,
                                             const dsp56300::Core &,
                                             const dsp56300::ForegroundGrant &) noexcept = nullptr;
};

enum class ForegroundCrossing : std::uint8_t { blocked, quiet, timer_effect };

struct ForegroundStores {
  const void *context;
  bool (*contains)(const void *, dsp56300::Word) noexcept;
};
}
