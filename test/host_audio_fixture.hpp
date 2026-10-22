// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/serial_pcm.hpp"

namespace blofeld::test {
[[nodiscard]] SerialPcmFrame host_audio_native_frame(bool secondary);
}
