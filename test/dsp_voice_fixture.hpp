// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld {
class PcmFrameSink;
}

namespace blofeld::test {
struct VoicePcmCapture;
enum class VoiceInput : std::uint8_t { none, controllers, foreground, stream };

// External output remains outside peripheral snapshots. Select at most one.
// A direct sink selects one six-block stream; capture retains the selected matrix.
struct VoiceOutputOptions {
  PcmFrameSink *direct = nullptr;
  VoicePcmCapture *capture = nullptr;
};

[[nodiscard]] int run_voice_test(int count,
                                 char **arguments,
                                 bool drive,
                                 bool master = false,
                                 bool pcm = false,
                                 VoiceOutputOptions output = {},
                                 VoiceInput input = VoiceInput::none);
}
