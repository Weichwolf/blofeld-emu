// SPDX-License-Identifier: GPL-3.0-only
#include "audio/device_buffer.hpp"

#include <CoreAudio/AudioHardware.h>
#include <CoreAudio/AudioHardwareBase.h>
#include <CoreAudioTypes/CoreAudioBaseTypes.h>
#include <MacTypes.h>
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>

namespace blofeld::host::audio {
namespace {
constexpr AudioObjectPropertyAddress buffer_size{kAudioDevicePropertyBufferFrameSize,
                                                 kAudioObjectPropertyScopeGlobal,
                                                 kAudioObjectPropertyElementMain};

void require(const OSStatus status) {
  if (status != noErr) {
    throw std::runtime_error("Native device buffer: OSStatus=" + std::to_string(status));
  }
}

template <typename T>
T property(const AudioObjectID device, const AudioObjectPropertyAddress &key) {
  T result{};
  const auto payload = std::as_writable_bytes(std::span(&result, 1));
  auto bytes = static_cast<UInt32>(payload.size());
  require(AudioObjectGetPropertyData(device, &key, 0, nullptr, &bytes, payload.data()));
  if (bytes != payload.size()) { throw std::runtime_error("Invalid native device buffer grant"); }
  return result;
}
}

DeviceBuffer::~DeviceBuffer() {
  if (device_ != kAudioObjectUnknown) {
    AudioObjectRemovePropertyListener(device_, &buffer_size, changed, this);
  }
}

void DeviceBuffer::open(const Request request) {
  assert(device_ == kAudioObjectUnknown);
  const auto device = request.device;
  previous_ = property<UInt32>(device, buffer_size);
  Boolean settable = 0;
  require(AudioObjectIsPropertySettable(device, &buffer_size, &settable));
  if (settable == 0) { return; }
  const AudioObjectPropertyAddress range_key{kAudioDevicePropertyBufferFrameSizeRange,
                                             kAudioObjectPropertyScopeGlobal,
                                             kAudioObjectPropertyElementMain};
  const auto range = property<AudioValueRange>(device, range_key);
  if (!std::isfinite(range.mMinimum) || !std::isfinite(range.mMaximum) || range.mMinimum < 1 ||
      range.mMaximum > std::numeric_limits<UInt32>::max() ||
      std::ceil(range.mMinimum) > std::floor(range.mMaximum)) {
    throw std::runtime_error("Invalid native device buffer range");
  }
  requested_ = std::clamp(request.frames,
                          static_cast<UInt32>(std::ceil(range.mMinimum)),
                          static_cast<UInt32>(std::floor(range.mMaximum)));
  if (previous_ == requested_) { return; }
  require(AudioObjectAddPropertyListener(device, &buffer_size, changed, this));
  device_ = device;
  apply(requested_);
}

OSStatus DeviceBuffer::changed(AudioObjectID /*device*/,
                               UInt32 /*count*/,
                               const AudioObjectPropertyAddress * /*addresses*/,
                               void *const context) {
  static_cast<DeviceBuffer *>(context)->notified_.store(true, std::memory_order_release);
  return noErr;
}

void DeviceBuffer::apply(const UInt32 frames) {
  notified_.store(false, std::memory_order_relaxed);
  require(AudioObjectSetPropertyData(device_, &buffer_size, 0, nullptr, sizeof(frames), &frames));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!notified_.load(std::memory_order_acquire)) {
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("Native device buffer acknowledgement timed out");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (property<UInt32>(device_, buffer_size) != frames) {
    throw std::runtime_error("Native device did not grant the requested buffer size");
  }
}

void DeviceBuffer::close() {
  if (device_ == kAudioObjectUnknown) { return; }
  std::exception_ptr failure;
  try {
    if (property<UInt32>(device_, buffer_size) == requested_) { apply(previous_); }
  } catch (...) { failure = std::current_exception(); }
  const auto status = AudioObjectRemovePropertyListener(device_, &buffer_size, changed, this);
  if (status == noErr) { device_ = kAudioObjectUnknown; }
  if (failure) { std::rethrow_exception(failure); }
  require(status);
}
}
