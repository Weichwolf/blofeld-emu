// SPDX-License-Identifier: GPL-3.0-only
#include "audio/buffer.hpp"
#include "audio/playback.hpp"
#include "audio/render_buffer.hpp"

#include <AudioToolbox/AUComponent.h>
#include <AudioToolbox/AudioComponent.h>
#include <AudioToolbox/AudioOutputUnit.h>
#include <AudioToolbox/AudioUnitProperties.h>
#include <CoreAudio/AudioHardware.h>
#include <CoreAudio/AudioHardwareBase.h>
#include <CoreAudioTypes/CoreAudioBaseTypes.h>
#include <CoreFoundation/CFBase.h>
#include <CoreFoundation/CFString.h>
#include <MacTypes.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <semaphore>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>

namespace blofeld::host::audio {
namespace {
constexpr unsigned channels = 2;
constexpr std::int64_t us_per_second = 1000000;
constexpr std::chrono::milliseconds device_check_period{100};
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
static_assert(std::atomic<OSStatus>::is_always_lock_free);

void require(const OSStatus status, const std::string_view operation) {
  if (status != noErr) {
    throw std::runtime_error(std::string(operation) + ": OSStatus=" + std::to_string(status));
  }
}

struct Property {
  AudioObjectPropertySelector selector;
  AudioObjectPropertyScope scope = kAudioObjectPropertyScopeGlobal;
};

template <typename T> T property(const AudioObjectID object, const Property key) {
  const AudioObjectPropertyAddress address{
      key.selector, key.scope, kAudioObjectPropertyElementMain};
  T result{};
  const auto payload = std::as_writable_bytes(std::span(&result, 1));
  auto bytes = static_cast<UInt32>(payload.size());
  require(AudioObjectGetPropertyData(object, &address, 0, nullptr, &bytes, payload.data()),
          "Query audio device property");
  if (bytes != payload.size()) { throw std::runtime_error("Invalid audio device property size"); }
  return result;
}

AudioObjectID default_device() {
  const auto device = property<AudioObjectID>(
      kAudioObjectSystemObject, {.selector = kAudioHardwarePropertyDefaultOutputDevice});
  if (device == kAudioObjectUnknown) { throw std::runtime_error("No default audio output"); }
  return device;
}

std::string text_property(const AudioObjectID device, const AudioObjectPropertySelector selector) {
  const auto *const text = property<CFStringRef>(device, {.selector = selector});
  if (text == nullptr) { throw std::runtime_error("Missing audio device identifier"); }
  constexpr std::size_t text_bytes = 4096;
  std::array<char, text_bytes> bytes{};
  const auto converted = CFStringGetCString(
      text, bytes.data(), static_cast<CFIndex>(bytes.size()), kCFStringEncodingUTF8);
  CFRelease(text);
  if (converted == 0) { throw std::runtime_error("Audio device identifier exceeds UTF-8 bounds"); }
  return bytes.data();
}

template <typename T>
T unit_property(const AudioUnit unit,
                const AudioUnitPropertyID property,
                const AudioUnitScope scope = kAudioUnitScope_Global) {
  T result{};
  const auto payload = std::as_writable_bytes(std::span(&result, 1));
  auto bytes = static_cast<UInt32>(payload.size());
  require(AudioUnitGetProperty(unit, property, scope, 0, payload.data(), &bytes),
          "Query audio unit grant");
  if (bytes != payload.size()) { throw std::runtime_error("Invalid audio unit property size"); }
  return result;
}
}

struct Playback::Impl {
  enum class Phase : std::uint8_t { opening, priming, playing, stopped, failed };
  static_assert(std::atomic<Phase>::is_always_lock_free);
  Grants granted;
  AudioUnit unit = nullptr;
  AudioObjectID device = kAudioObjectUnknown;
  UInt32 device_frames = 0, maximum_slice = 0;
  std::size_t physical_buffer = 0;
  std::int64_t device_latency_us = 0;
  std::unique_ptr<RenderBuffer> buffer;
  std::exception_ptr error;
  std::atomic<Phase> phase{Phase::opening};
  std::atomic<OSStatus> render_error{noErr};
  std::atomic<bool> reset_pending{false};
  std::atomic<std::uint64_t> submitted{0}, underruns{0}, renders{0};
  std::binary_semaphore reset_done{0};
  std::mutex wake_mutex;
  std::condition_variable wake;
  std::jthread owner;

  Impl(const unsigned rate, const std::string_view name) {
    constexpr unsigned maximum_rate = 384000;
    if (rate == 0 || rate > maximum_rate) {
      throw std::invalid_argument(std::string(name) + ": invalid audio sample rate");
    }
    owner = std::jthread([this, rate](const std::stop_token &stop) { run(rate, stop); });
    phase.wait(Phase::opening, std::memory_order_acquire);
    if (phase.load(std::memory_order_acquire) == Phase::failed) { std::rethrow_exception(error); }
  }

  ~Impl() { stop(); }

  [[nodiscard]] bool open() const noexcept {
    const auto current = phase.load(std::memory_order_acquire);
    return current == Phase::priming || current == Phase::playing;
  }

  void stop() noexcept {
    owner.request_stop();
    wake.notify_one();
    if (owner.joinable()) { owner.join(); }
  }

  static OSStatus render(void *context,
                         AudioUnitRenderActionFlags *flags,
                         const AudioTimeStamp *time,
                         const UInt32 bus,
                         const UInt32 frames,
                         AudioBufferList *data) noexcept [[clang::nonblocking]] {
    auto &self = *static_cast<Impl *>(context);
    const auto current = self.phase.load(std::memory_order_acquire);
    if (frames == 0) { return noErr; }
    if (bus != 0 || time == nullptr || data == nullptr || data->mNumberBuffers != 1 ||
        data->mBuffers[0].mNumberChannels != channels || data->mBuffers[0].mData == nullptr ||
        frames > self.maximum_slice ||
        data->mBuffers[0].mDataByteSize < std::uint64_t{frames} * sizeof(FloatFrame)) {
      self.render_error.store(kAudio_ParamError, std::memory_order_release);
      return kAudio_ParamError;
    }
    const std::span output(static_cast<FloatFrame *>(data->mBuffers[0].mData), frames);
    if (current != Phase::playing) {
      std::ranges::fill(output, FloatFrame{});
      if (flags != nullptr) { *flags |= kAudioUnitRenderAction_OutputIsSilence; }
      return noErr;
    }
    const auto supplied = self.buffer->render(output);
    self.submitted.fetch_add(supplied, std::memory_order_relaxed);
    self.renders.fetch_add(1, std::memory_order_release);
    if (supplied != frames) { self.underruns.fetch_add(1, std::memory_order_release); }
    if (flags != nullptr) {
      if (supplied == 0) {
        *flags |= kAudioUnitRenderAction_OutputIsSilence;
      } else {
        *flags &= ~kAudioUnitRenderAction_OutputIsSilence;
      }
    }
    return noErr;
  }

  void create_unit(const unsigned rate) {
    const AudioComponentDescription description{.componentType = kAudioUnitType_Output,
                                                .componentSubType = kAudioUnitSubType_DefaultOutput,
                                                .componentManufacturer =
                                                    kAudioUnitManufacturer_Apple,
                                                .componentFlags = 0,
                                                .componentFlagsMask = 0};
    auto *const component = AudioComponentFindNext(nullptr, &description);
    if (component == nullptr) {
      throw std::runtime_error("Native output audio unit is unavailable");
    }
    require(AudioComponentInstanceNew(component, &unit), "Create native output audio unit");
    const AudioStreamBasicDescription format{.mSampleRate = static_cast<Float64>(rate),
                                             .mFormatID = kAudioFormatLinearPCM,
                                             .mFormatFlags = kAudioFormatFlagIsFloat |
                                                             kAudioFormatFlagIsPacked |
                                                             kAudioFormatFlagsNativeEndian,
                                             .mBytesPerPacket = sizeof(FloatFrame),
                                             .mFramesPerPacket = 1,
                                             .mBytesPerFrame = sizeof(FloatFrame),
                                             .mChannelsPerFrame = channels,
                                             .mBitsPerChannel = 32,
                                             .mReserved = 0};
    require(AudioUnitSetProperty(unit,
                                 kAudioUnitProperty_StreamFormat,
                                 kAudioUnitScope_Input,
                                 0,
                                 &format,
                                 sizeof(format)),
            "Bind original stereo sample rate");
    const AURenderCallbackStruct callback{.inputProc = render, .inputProcRefCon = this};
    require(AudioUnitSetProperty(unit,
                                 kAudioUnitProperty_SetRenderCallback,
                                 kAudioUnitScope_Input,
                                 0,
                                 &callback,
                                 sizeof(callback)),
            "Bind native render callback");
    require(AudioUnitInitialize(unit), "Initialize native output audio unit");
  }

  void initialize(const unsigned rate) {
    device = default_device();
    create_unit(rate);
    const auto client = unit_property<AudioStreamBasicDescription>(
        unit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input);
    if (client.mFormatID != kAudioFormatLinearPCM || client.mSampleRate != rate ||
        client.mChannelsPerFrame != channels || client.mBytesPerFrame != sizeof(FloatFrame) ||
        client.mBitsPerChannel != 32 || (client.mFormatFlags & kAudioFormatFlagIsFloat) == 0 ||
        (client.mFormatFlags & kAudioFormatFlagIsNonInterleaved) != 0) {
      throw std::runtime_error("Native output did not grant the requested stereo PCM format");
    }
    if (unit_property<AudioObjectID>(unit, kAudioOutputUnitProperty_CurrentDevice) != device) {
      throw std::runtime_error("Default audio output changed while opening");
    }
    const auto sink_rate =
        property<Float64>(device, {.selector = kAudioDevicePropertyNominalSampleRate});
    if (!std::isfinite(sink_rate) || sink_rate < 1 ||
        sink_rate > std::numeric_limits<unsigned>::max() || std::trunc(sink_rate) != sink_rate) {
      throw std::runtime_error("Unsupported device sample rate");
    }
    const auto sink_hz = static_cast<unsigned>(sink_rate);
    device_frames = property<UInt32>(device, {.selector = kAudioDevicePropertyBufferFrameSize});
    maximum_slice = unit_property<UInt32>(unit, kAudioUnitProperty_MaximumFramesPerSlice);
    physical_buffer =
        static_cast<std::size_t>((std::uint64_t{device_frames} * rate + sink_hz - 1) / sink_hz);
    constexpr unsigned prime_quanta = 3;
    const auto target = std::max(physical_buffer * prime_quanta, std::size_t{maximum_slice});
    const auto capacity = target + maximum_slice;
    if (physical_buffer == 0 || maximum_slice == 0 || capacity > rate) {
      throw std::runtime_error("Native audio grant exceeds the one-second queue bound");
    }
    buffer = std::make_unique<RenderBuffer>(capacity);
    const auto latency = property<UInt32>(
        device,
        {.selector = kAudioDevicePropertyLatency, .scope = kAudioObjectPropertyScopeOutput});
    const auto safety = property<UInt32>(
        device,
        {.selector = kAudioDevicePropertySafetyOffset, .scope = kAudioObjectPropertyScopeOutput});
    const auto unit_latency = unit_property<Float64>(unit, kAudioUnitProperty_Latency);
    if (!std::isfinite(unit_latency) || unit_latency < 0 || unit_latency > 1) {
      throw std::runtime_error("Unsupported native audio unit latency");
    }
    device_latency_us =
        static_cast<std::int64_t>(std::ceil(unit_latency * us_per_second)) +
        static_cast<std::int64_t>((std::uint64_t{latency} + safety + device_frames) *
                                  us_per_second / sink_hz);
    const auto hardware = unit_property<AudioStreamBasicDescription>(
        unit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output);
    granted = {.rate = rate,
               .sink_rate = sink_hz,
               .target_frames = target,
               .maximum_frames = capacity,
               .minimum_request_frames = {},
               .sink = text_property(device, kAudioObjectPropertyName),
               .sink_format = "CoreAudio " + std::to_string(hardware.mBitsPerChannel) + "bit/" +
                              std::to_string(hardware.mChannelsPerFrame) + "ch PCM"};
  }

  void reset() {
    phase.store(Phase::priming, std::memory_order_release);
    require(AudioOutputUnitStop(unit), "Stop old audio epoch");
    require(AudioUnitReset(unit, kAudioUnitScope_Global, 0), "Flush old audio epoch");
    buffer->discard();
    reset_done.release();
  }

  void check_device() const {
    if (default_device() != device ||
        property<Float64>(device, {.selector = kAudioDevicePropertyNominalSampleRate}) !=
            granted.sink_rate ||
        property<UInt32>(device, {.selector = kAudioDevicePropertyBufferFrameSize}) !=
            device_frames ||
        property<UInt32>(device, {.selector = kAudioDevicePropertyDeviceIsAlive}) == 0) {
      throw std::runtime_error("Audio device disconnected or changed");
    }
    if (phase.load(std::memory_order_acquire) == Phase::playing &&
        unit_property<UInt32>(unit, kAudioOutputUnitProperty_IsRunning) == 0) {
      throw std::runtime_error("Native audio output stopped unexpectedly");
    }
  }

  void drive(const std::stop_token &stop) {
    auto next_check = std::chrono::steady_clock::now();
    auto last_render = next_check;
    std::uint64_t observed = 0;
    while (!stop.stop_requested()) {
      if (reset_pending.exchange(false, std::memory_order_acq_rel)) { reset(); }
      require(render_error.load(std::memory_order_acquire), "Native render format changed");
      if (underruns.load(std::memory_order_acquire) != 0) {
        throw std::runtime_error("Native audio output underrun");
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_check) {
        check_device();
        next_check = now + device_check_period;
      }
      if (phase.load(std::memory_order_acquire) == Phase::priming &&
          buffer->queued() >= granted.target_frames) {
        phase.store(Phase::playing, std::memory_order_release);
        require(AudioOutputUnitStart(unit), "Start primed native audio output");
        last_render = now;
      }
      const auto current = renders.load(std::memory_order_acquire);
      if (current != observed) {
        observed = current;
        last_render = now;
      }
      if (phase.load(std::memory_order_acquire) == Phase::playing &&
          now - last_render > std::chrono::seconds(2)) {
        throw std::runtime_error("Native audio callback timed out");
      }
      std::unique_lock lock(wake_mutex);
      wake.wait_for(lock, std::chrono::milliseconds(1));
    }
  }

  void run(const unsigned rate, const std::stop_token &stop) noexcept {
    try {
      initialize(rate);
      phase.store(Phase::priming, std::memory_order_release);
      phase.notify_all();
      drive(stop);
    } catch (...) {
      if (!stop.stop_requested()) {
        error = std::current_exception();
        phase.store(Phase::failed, std::memory_order_release);
      }
    }
    if (phase.load(std::memory_order_acquire) != Phase::failed) {
      phase.store(Phase::stopped, std::memory_order_release);
    }
    if (unit != nullptr) {
      AudioOutputUnitStop(unit);
      AudioUnitUninitialize(unit);
      AudioComponentInstanceDispose(unit);
    }
    phase.notify_all();
  }
};

Playback::Playback(const unsigned rate, const std::string_view name)
    : impl_(std::make_unique<Impl>(rate, name)) {}

Playback::~Playback() = default;

void Playback::stop() noexcept {
  impl_->stop();
}

const Grants &Playback::grants() const noexcept {
  return impl_->granted;
}

void Playback::check() const {
  if (impl_->phase.load(std::memory_order_acquire) == Impl::Phase::failed) {
    std::rethrow_exception(impl_->error);
  }
  require(impl_->render_error.load(std::memory_order_acquire), "Native render format changed");
  if (impl_->underruns.load(std::memory_order_acquire) != 0) {
    throw std::runtime_error("Native audio output underrun");
  }
}

bool Playback::write(std::span<const Frame> frames, const std::stop_token &stop) {
  while (!frames.empty() && !stop.stop_requested() && impl_->open()) {
    const auto count = impl_->buffer->produce(frames);
    if (count != 0) {
      frames = frames.subspan(count);
      impl_->wake.notify_one();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return frames.empty() && !stop.stop_requested() && impl_->open();
}

bool Playback::reset(const std::stop_token &stop) {
  if (stop.stop_requested() || !impl_->open()) { return false; }
  impl_->reset_pending.store(true, std::memory_order_release);
  impl_->wake.notify_one();
  while (!stop.stop_requested() && impl_->open()) {
    if (impl_->reset_done.try_acquire_for(std::chrono::milliseconds(1))) { return true; }
  }
  impl_->owner.request_stop();
  impl_->wake.notify_one();
  return false;
}

PlaybackStatus Playback::status() const noexcept {
  const auto queued = impl_->buffer->queued();
  const auto phase = impl_->phase.load(std::memory_order_acquire);
  return {.submitted_frames = impl_->submitted.load(std::memory_order_relaxed),
          .underruns = impl_->underruns.load(std::memory_order_acquire),
          .queued_frames = queued,
          .server_buffer_frames = impl_->physical_buffer,
          .latency_us = impl_->device_latency_us +
                        static_cast<std::int64_t>(queued * us_per_second / impl_->granted.rate),
          .started = phase == Impl::Phase::playing,
          .failed = phase == Impl::Phase::failed};
}
}
