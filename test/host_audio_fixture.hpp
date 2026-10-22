// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/serial_pcm.hpp"

#include <cstdint>
#include <vector>

namespace blofeld::test {
[[nodiscard]] SerialPcmFrame host_audio_native_frame(bool secondary);
[[nodiscard]] std::vector<std::uint32_t> host_audio_native_program(bool secondary);
void reference_host_audio(const char *server);
}
