// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace blofeld {
class PcmFrameSink;
}

namespace blofeld::test {
struct VoicePcmCapture;

// Output objects stay outside peripheral snapshots and must outlive the call.
// A direct sink selects one default eight-block stream; capture retains the full
// matrix. Choose at most one; neither option changes the original PCM path.
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
                                 bool midi = false);
}
