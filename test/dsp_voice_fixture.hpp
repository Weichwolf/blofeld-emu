// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld {
class PcmFrameSink;
}

namespace blofeld::test {
struct VoicePcmCapture;
enum class VoiceInput : std::uint8_t { none, controllers, foreground, stream, stream_midi };

// Output objects stay outside peripheral snapshots and must outlive the call.
// A direct sink selects one default eight-block stream; capture retains the full
// matrix. Choose at most one; neither option changes the original PCM path.
// Stream modes select one six-block or ten-block idle-MIDI ring for direct delivery;
// its full matrix remains available without an external sink.
struct VoiceOutputOptions {
  PcmFrameSink *direct = nullptr;
  VoicePcmCapture *capture = nullptr;
  // Direct stream context: pitch bit2, rounding bit1, filter-routing bit0.
  unsigned stream_case_index = 0;
};

[[nodiscard]] int run_voice_test(int count,
                                 char **arguments,
                                 bool drive,
                                 bool master = false,
                                 bool pcm = false,
                                 VoiceOutputOptions output = {},
                                 VoiceInput input = VoiceInput::none);
}
