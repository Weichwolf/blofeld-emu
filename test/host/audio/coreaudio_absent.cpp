// SPDX-License-Identifier: GPL-3.0-only
#include <CoreAudio/AudioHardware.h>
#include <CoreAudio/AudioHardwareBase.h>
#include <MacTypes.h>

namespace {
OSStatus unavailable(const AudioObjectID object,
                     const AudioObjectPropertyAddress *address,
                     const UInt32 qualifier_bytes,
                     const void *qualifier,
                     UInt32 *bytes,
                     void *output) {
  if (object != kAudioObjectSystemObject || qualifier_bytes != 0 || qualifier != nullptr ||
      address->mSelector != kAudioHardwarePropertyDefaultOutputDevice ||
      address->mScope != kAudioObjectPropertyScopeGlobal ||
      address->mElement != kAudioObjectPropertyElementMain) {
    return kAudioHardwareUnknownPropertyError;
  }
  if (*bytes < sizeof(AudioObjectID)) { return kAudioHardwareBadPropertySizeError; }
  *bytes = sizeof(AudioObjectID);
  *static_cast<AudioObjectID *>(output) = kAudioObjectUnknown;
  return noErr;
}

[[gnu::used, gnu::section("__DATA,__interpose")]] const struct {
  decltype(&AudioObjectGetPropertyData) replacement;
  decltype(&AudioObjectGetPropertyData) original;
} interpose{.replacement = unavailable, .original = AudioObjectGetPropertyData};
}
