// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace blofeld {
// Worker-owned Linux event; lifetime exceeds the native audio session. The
// producer signals at most once per source wait, without allocation or blocking.
class AudioWake {
public:
  AudioWake();
  ~AudioWake();
  AudioWake(const AudioWake &) = delete;
  AudioWake &operator=(const AudioWake &) = delete;
  [[nodiscard]] int signal() const noexcept;
  void wait() const;

private:
  int descriptor_ = -1;
};
}
