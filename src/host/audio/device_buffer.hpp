// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <CoreAudio/AudioHardware.h>
#include <MacTypes.h>
#include <atomic>

namespace blofeld::host::audio {
class DeviceBuffer {
public:
  struct Request {
    AudioObjectID device;
    UInt32 frames;
  };

  DeviceBuffer() = default;
  ~DeviceBuffer();
  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer &operator=(const DeviceBuffer &) = delete;
  void open(Request request);
  void close();

private:
  static OSStatus changed(AudioObjectID device,
                          UInt32 count,
                          const AudioObjectPropertyAddress *addresses,
                          void *context);
  void apply(UInt32 frames);
  AudioObjectID device_ = kAudioObjectUnknown;
  UInt32 previous_ = 0, requested_ = 0;
  std::atomic<bool> notified_{false};
};
}
