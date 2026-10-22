// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld::test {
struct VoicePcmCapture;
enum class VoiceInput : std::uint8_t { none, controllers, foreground, stream };
[[nodiscard]] int run_voice_test(int count,
                                 char **arguments,
                                 bool drive,
                                 bool master = false,
                                 bool pcm = false,
                                 VoicePcmCapture *capture = nullptr,
                                 VoiceInput input = VoiceInput::none);
}
