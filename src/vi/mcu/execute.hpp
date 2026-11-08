// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/shi.hpp"
#include "storage/flash_image.hpp"

#include <stop_token>

namespace blofeld::vi::mcu {
[[nodiscard]] ExecutionReport
execute(FlashImage &image, Shi &shi, DspReset &reset, const std::stop_token &stop);
}
