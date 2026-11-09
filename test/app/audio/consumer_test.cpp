// SPDX-License-Identifier: GPL-3.0-only
#include "audio/buffer.hpp"
#include "audio/consumer.hpp"
#include "audio/pcm_clock.hpp"
#include "audio/playback.hpp"
#include "audio/render_position.hpp"
#include "io/pcm_output.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <semaphore>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <string_view>
#include <thread>
#include <vector>

namespace {
using blofeld::app::AudioConsumer;
using blofeld::app::PcmClock;
using blofeld::host::audio::Playback;
using blofeld::vi::PcmOutput;
constexpr unsigned rate = 44100, frame_ticks = 256;
constexpr std::size_t history = 64;
constexpr std::uint64_t guest_epoch = 9, host_ns = 10000;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void rejects(auto &&action) {
  bool rejected = false;
  try {
    action();
  } catch (const std::exception &) { rejected = true; }
  require(rejected, "Invalid source PCM or an unavailable origin must fail");
}

struct Device {
  std::vector<PcmOutput::Frame> written;
  blofeld::host::audio::PlaybackStatus status{};
  unsigned producers = 0, resets = 0, begins = 0, ends = 0;
  bool accept_write = true, accept_reset = true;
};

Device *&active_device() {
  static Device *device = nullptr;
  return device;
}
}

namespace blofeld::host::audio {
struct Playback::Impl {
  Device &device;
  Grants grants{.rate = rate,
                .sink_rate = rate,
                .target_frames = 8,
                .maximum_frames = 16,
                .minimum_request_frames = 4,
                .position_history_frames = history,
                .sink = "test",
                .sink_format = "s24"};
};

struct Playback::Producer::Impl {
  Device &device;
};

Playback::Playback(const unsigned requested_rate, const std::string_view name)
    : impl_(std::make_unique<Impl>(Impl{.device = *active_device()})) {
  require(requested_rate == rate && !name.empty(), "Actual requested host format");
}

Playback::~Playback() = default;

bool Playback::write(const std::span<const Frame> frames, const std::stop_token &stop) {
  if (stop.stop_requested()) { return false; }
  auto &device = impl_->device;
  if (!device.accept_write) { return false; }
  device.written.insert(device.written.end(), frames.begin(), frames.end());
  return true;
}

bool Playback::reset(const std::stop_token &stop) {
  if (stop.stop_requested()) { return false; }
  auto &device = impl_->device;
  ++device.resets;
  device.status.position.reset();
  return device.accept_reset;
}

void Playback::stop() noexcept {}

void Playback::check() const {}

const Grants &Playback::grants() const noexcept {
  return impl_->grants;
}

PlaybackStatus Playback::status() const noexcept {
  return impl_->device.status;
}

Playback::Producer Playback::producer() const {
  return Producer(std::chrono::nanoseconds(impl_->grants.minimum_request_frames.value_or(0)));
}

Playback::Producer::Producer(const std::chrono::nanoseconds period)
    : impl_(std::make_unique<Impl>(Impl{.device = *active_device()})) {
  require(period.count() > 0, "A bounded native producer period");
  ++impl_->device.producers;
}

Playback::Producer::Producer(Producer &&) noexcept = default;
Playback::Producer::~Producer() = default;

void Playback::Producer::begin_work() {
  ++impl_->device.begins;
}

void Playback::Producer::end_work() {
  ++impl_->device.ends;
}
}

namespace {
struct Fixture {
  Device device;
  std::unique_ptr<Playback> playback;
  std::unique_ptr<AudioConsumer> consumer;

  explicit Fixture(const bool deferred = true) {
    require(active_device() == nullptr, "One deterministic host device per fixture");
    active_device() = &device;
    playback = std::make_unique<Playback>(rate);
    consumer = std::make_unique<AudioConsumer>(*playback, deferred);
  }

  ~Fixture() {
    consumer.reset();
    playback.reset();
    active_device() = nullptr;
  }

  Fixture(const Fixture &) = delete;
  Fixture(Fixture &&) = delete;
  Fixture &operator=(const Fixture &) = delete;
  Fixture &operator=(Fixture &&) = delete;

  [[nodiscard]] bool write(const std::uint64_t first,
                           const std::span<const std::uint64_t> ticks,
                           const std::uint64_t epoch = guest_epoch,
                           const std::uint16_t underruns = 0,
                           const std::stop_token &stop = {}) const {
    std::array<PcmOutput::Frame, 16> frames{};
    for (std::size_t index = 0; index < ticks.size(); ++index) {
      const auto value = static_cast<std::int32_t>(first + index);
      frames[index] = {value, -value};
    }
    const PcmOutput::Block block{.frames = std::span(frames).first(ticks.size()),
                                 .ticks = ticks,
                                 .epoch = epoch,
                                 .first_frame = first,
                                 .reference_hz = rate * frame_ticks,
                                 .frame_ticks = frame_ticks,
                                 .underruns = underruns};
    const auto output = consumer->output();
    return output.write(output.owner, block, stop);
  }

  void position(const std::uint64_t first, const std::uint64_t epoch = 0) {
    device.status.position = blofeld::host::audio::RenderPosition{
        .epoch = epoch, .first_frame = first, .host_ns = host_ns, .frames = 1};
  }
};

void selected_original_frames() {
  Fixture fixture;
  auto &consumer = *fixture.consumer;
  constexpr std::array<std::uint64_t, 2> initial{10, 20};
  constexpr std::array<std::uint64_t, 4> selected{25, 30, 31, 50};
  require(!consumer.has_pcm(), "No readiness before original PCM");
  rejects([&] { consumer.begin_at(1); });
  require(fixture.write(0, initial, guest_epoch, 3) && consumer.has_pcm(),
          "Boot PCM is validated without submitting audio");
  require(fixture.device.written.empty() && fixture.device.producers == 0 &&
              fixture.device.resets == 0 && !consumer.clock(),
          "Boot neither primes native playback nor changes producer scheduling");
  rejects([&] { consumer.begin_at(initial.back()); });
  constexpr std::uint64_t begin = 30;
  consumer.begin_at(begin);
  rejects([&] { consumer.begin_at(begin + 1); });
  require(fixture.write(2, selected, guest_epoch, 11), "Cross the selected origin inside a block");
  require(fixture.device.written == std::vector<PcmOutput::Frame>{{3, -3}, {4, -4}, {5, -5}},
          "Retain exact original channels and order without creating a boundary frame");
  require(fixture.device.producers == 1 && fixture.device.resets == 0 &&
              consumer.statistics().accepted_frames == 3 &&
              consumer.statistics().device_underruns == 2,
          "Only selected frames and their remapped hardware underrun bits are counted");
  fixture.position(0);
  require(consumer.clock() ==
              PcmClock::Point{
                  .epoch = guest_epoch, .audio_epoch = 0, .ticks = begin, .host_ns = host_ns},
          "Native frame zero resolves original frame three before another DSP write");
  fixture.position(2);
  require(consumer.clock() == PcmClock::Point{.epoch = guest_epoch,
                                              .audio_epoch = 0,
                                              .ticks = selected.back(),
                                              .host_ns = host_ns},
          "Native relative indices preserve source ticks");
  constexpr std::array<std::uint64_t, 2> later{60, 70};
  require(fixture.write(6, later) && consumer.statistics().accepted_frames == 5,
          "Selection does not rewrite the expected next original frame number");
  fixture.position(3);
  require(consumer.clock() == PcmClock::Point{.epoch = guest_epoch,
                                              .audio_epoch = 0,
                                              .ticks = later.front(),
                                              .host_ns = host_ns},
          "Later blocks retain the selected source origin");
  const auto output = consumer.output();
  output.close(output.owner);
  require(!consumer.clock() && !consumer.has_pcm(), "Shutdown revokes readiness and clock");
  rejects([&] { return fixture.write(8, later); });
  rejects([&] { consumer.begin_at(later.back() + 1); });
}

void reset_epochs() {
  Fixture fixture;
  constexpr std::array<std::uint64_t, 2> ticks{10, 20};
  constexpr std::array<std::uint64_t, 2> after{30, 40};
  require(fixture.write(0, ticks), "Valid boot PCM");
  require(fixture.write(0, ticks, guest_epoch + 1) && fixture.device.resets == 0,
          "Boot resets validate a new source sequence without resetting an unopened native stream");
  fixture.consumer->begin_at(after.front());
  require(fixture.write(2, after, guest_epoch + 1), "Start from a nonzero source frame");
  fixture.position(0);
  require(fixture.consumer->clock() == PcmClock::Point{.epoch = guest_epoch + 1,
                                                       .audio_epoch = 0,
                                                       .ticks = after.front(),
                                                       .host_ns = host_ns},
          "Selected initial audio epoch");
  require(fixture.write(0, ticks, guest_epoch + 2),
          "A running reset accepts new original frame zero");
  require(fixture.device.resets == 1 && !fixture.consumer->clock(),
          "Reset flushes the real stream and revokes its previous native interval");
  fixture.position(0, 1);
  require(fixture.consumer->clock() == PcmClock::Point{.epoch = guest_epoch + 2,
                                                       .audio_epoch = 1,
                                                       .ticks = ticks.front(),
                                                       .host_ns = host_ns},
          "A hardware reset does not reapply the obsolete one-time startup boundary");
  rejects([&] { return fixture.write(2, after, guest_epoch + 1); });
  require(!fixture.consumer->clock(), "A backwards epoch revokes the clock");
}

void invalid_and_cancelled() {
  constexpr std::array<std::uint64_t, 2> ticks{10, 20};
  {
    Fixture fixture;
    require(fixture.write(0, ticks), "Initial source");
    rejects([&] { return fixture.write(3, ticks); });
    require(fixture.device.written.empty(), "Even discarded boot PCM must have exact sequencing");
    rejects([&] { fixture.consumer->begin_at(ticks.back() + 1); });
  }
  for (const bool reset : {false, true}) {
    Fixture fixture(false);
    require(fixture.write(0, ticks), "Immediate mode preserves its original behavior");
    fixture.position(0);
    require(fixture.consumer->clock().has_value(), "A real accepted interval maps successfully");
    fixture.device.accept_write = false;
    fixture.device.accept_reset = false;
    constexpr std::array<std::uint64_t, 2> later{30, 40};
    require(!fixture.write(reset ? 0 : 2, later, guest_epoch + (reset ? 1U : 0U)) &&
                !fixture.consumer->clock(),
            "A cancelled native write or reset revokes the old clock");
    rejects([&] { return fixture.write(0, ticks); });
  }
  {
    Fixture fixture;
    std::stop_source stop;
    stop.request_stop();
    require(!fixture.write(0, ticks, guest_epoch, 0, stop.get_token()) &&
                !fixture.consumer->has_pcm() && fixture.device.producers == 0,
            "Stop applies before boot validation or host producer creation");
  }
  {
    Fixture fixture;
    rejects([&] { return fixture.write(0, ticks, guest_epoch, 4); });
    require(!fixture.consumer->has_pcm(), "Reject hardware underrun bits outside the source block");
  }
  {
    Fixture fixture;
    constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
    constexpr std::array<std::uint64_t, 2> end{maximum - 1, maximum};
    require(fixture.write(0, end) && fixture.device.written.empty() &&
                fixture.device.producers == 0,
            "Exhausting guest time cannot select an unpublished startup boundary");
    rejects([&] { fixture.consumer->begin_at(maximum); });
  }
}

void concurrent_begin() {
  Fixture fixture;
  std::binary_semaphore ready(0);
  std::binary_semaphore admitted(0);
  std::exception_ptr error;
  constexpr std::chrono::seconds guard{5};
  std::jthread producer([&] {
    bool announced = false;
    try {
      constexpr std::array<std::uint64_t, 2> initial{10, 20};
      require(fixture.write(0, initial), "Producer validates original startup PCM");
      announced = true;
      ready.release();
      require(admitted.try_acquire_for(guard), "Bounded admission wait");
      constexpr std::array<std::uint64_t, 3> next{25, 30, 40};
      require(fixture.write(2, next), "Resume only after the control owner publishes admission");
    } catch (...) {
      error = std::current_exception();
      if (!announced) { ready.release(); }
    }
  });
  require(ready.try_acquire_for(guard), "Bounded startup wait");
  constexpr std::uint64_t origin = 30;
  fixture.consumer->begin_at(origin);
  admitted.release();
  producer.join();
  if (error) { std::rethrow_exception(error); }
  require(
      fixture.device.written == std::vector<PcmOutput::Frame>{{3, -3}, {4, -4}} &&
          fixture.device.producers == 1,
      "Cross-owner admission retains exact original samples without a producer on the host thread");
}
}

int main() try {
  selected_original_frames();
  reset_epochs();
  invalid_and_cancelled();
  concurrent_begin();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
