// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace blofeld {
class PcmFrameSink;
}

namespace blofeld::test {
// An external sink selects one default eight-block PCM stream. It is outside
// peripheral replay state and must outlive the call; other modes retain matrices.
[[nodiscard]] int run_voice_test(int count,
                                 char **arguments,
                                 bool drive,
                                 bool master = false,
                                 bool pcm = false,
                                 PcmFrameSink *stream_output = nullptr);
}
