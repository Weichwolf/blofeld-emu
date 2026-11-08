// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "execution.hpp"

#include <dsp56300/memory.hpp>
#include <span>
#include <stop_token>

namespace blofeld::vi::dsp {
[[nodiscard]] ExecutionReport execute(std::span<const dsp56300::Word> boot,
                                      const std::stop_token &stop);
}
