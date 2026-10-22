// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace blofeld::test {
struct VoicePcmCapture;
[[nodiscard]] int run_voice_test(int count,
                                 char **arguments,
                                 bool drive,
                                 bool master = false,
                                 bool pcm = false,
                                 VoicePcmCapture *capture = nullptr,
                                 bool midi = false);
}
