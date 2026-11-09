// SPDX-License-Identifier: GPL-3.0-only
#include "audio/wav_consumer.hpp"
#include "files/read.hpp"
#include "io/pcm_output.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <semaphore>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <thread>
#include <unistd.h>

namespace {
using blofeld::app::WavConsumer;
using blofeld::vi::PcmOutput;
constexpr unsigned rate = 44100, reference_hz = 11289600, frame_ticks = 256;
constexpr std::size_t header_bytes = 44;
constexpr std::array<PcmOutput::Frame, 2> frames{PcmOutput::Frame{-1, 1}, PcmOutput::Frame{0, 0}};
constexpr std::array<std::uint64_t, 2> completion_ticks{1360, 2721};
constexpr PcmOutput::Block initial{.frames = frames,
                                   .ticks = completion_ticks,
                                   .epoch = 9,
                                   .reference_hz = reference_hz,
                                   .frame_ticks = frame_ticks};

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

template <typename Error> void rejects(auto &&action, const char *message) {
  bool rejected = false;
  try {
    action();
  } catch (const Error &) { rejected = true; }
  require(rejected, message);
}

void successful(const std::filesystem::path &root) {
  const auto path = root / "original.wav";
  WavConsumer consumer(path, rate);
  const auto output = consumer.output();
  require(output.write(output.owner, initial, {}), "Accept an original stereo PCM block");
  auto next = initial;
  next.first_frame = frames.size();
  constexpr std::array<std::uint64_t, 2> next_ticks{4081, 5442};
  next.ticks = next_ticks;
  require(output.write(output.owner, next, {}) && consumer.frames() == frames.size() * 2,
          "Accept consecutive frames exactly once");
  rejects<std::logic_error>([&] { consumer.finish(); },
                            "Do not publish while a DSP owner is active");
  output.close(output.owner);
  consumer.finish();
  constexpr std::size_t bytes = header_bytes + (std::size_t{4} * 6);
  const auto wav = blofeld::host::read_file(path, bytes);
  constexpr std::array<std::uint8_t, 24> payload{0xff, 0xff, 0xff, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                                                 0xff, 0xff, 0xff, 1, 0, 0, 0, 0, 0, 0, 0, 0};
  require(wav.size() == bytes && std::ranges::equal(std::span(wav).subspan(header_bytes), payload),
          "Original PCM reaches the WAV without gain, host synthesis or float conversion");
  rejects<std::logic_error>([&] { consumer.finish(); }, "Do not finalize the same render twice");
}

void selected_window(const std::filesystem::path &root) {
  const auto path = root / "window.wav";
  WavConsumer consumer(path, rate, true);
  const auto output = consumer.output();
  require(!consumer.has_pcm() && !consumer.complete(), "No readiness before actual original PCM");
  require(output.write(output.owner, initial, {}) && consumer.frames() == 0 && consumer.has_pcm(),
          "Validate startup PCM without writing it into the musical file");
  rejects<std::logic_error>([&] { consumer.begin_at(1); }, "Reject a retroactive musical origin");
  constexpr std::uint64_t begin = 3000;
  constexpr std::uint64_t end = 5000;
  consumer.begin_at(begin);
  rejects<std::logic_error>([&] { consumer.begin_at(begin); }, "Select musical origin only once");
  rejects<std::logic_error>([&] { consumer.end_at(begin); }, "Completion must follow the origin");
  consumer.end_at(end);
  auto block = initial;
  block.first_frame = 2;
  constexpr std::array<std::uint64_t, 2> next{4081, 5442};
  block.ticks = next;
  require(output.write(output.owner, block, {}) && consumer.frames() == 1 && consumer.complete(),
          "Select only original frames inside the half-open musical interval");
  block.first_frame = 4;
  constexpr std::array<std::uint64_t, 2> after{6802, 8163};
  block.ticks = after;
  require(output.write(output.owner, block, {}) && consumer.frames() == 1,
          "Shutdown work never duplicates or appends post-boundary frames");
  output.close(output.owner);
  consumer.finish();
  const auto data = blofeld::host::read_file(path, header_bytes + 6);
  constexpr std::array<std::uint8_t, 6> expected{0xff, 0xff, 0xff, 1, 0, 0};
  require(data.size() == header_bytes + 6 &&
              std::ranges::equal(std::span(data).subspan(header_bytes), expected),
          "Boundary clipping keeps the exact selected stereo samples");
}

void incomplete_window(const std::filesystem::path &root) {
  const auto path = root / "incomplete-window.wav";
  {
    WavConsumer consumer(path, rate, true);
    const auto output = consumer.output();
    consumer.begin_at(1);
    constexpr std::uint64_t end = 5000;
    consumer.end_at(end);
    require(output.write(output.owner, initial, {}), "Accept valid unfinished musical frames");
    output.close(output.owner);
    rejects<std::logic_error>([&] { consumer.finish(); },
                              "Never publish a truncated render window");
  }
  require(!std::filesystem::exists(path), "Incomplete musical output remains unpublished");
}

void quantized_timing(const std::filesystem::path &root) {
  WavConsumer consumer(root / "quantized.wav", rate);
  const auto output = consumer.output();
  constexpr std::array<std::uint64_t, 6> ticks{1632, 2857, 4081, 5714, 6938, 8163};
  auto block = initial;
  for (std::size_t first = 0; first < ticks.size(); first += frames.size()) {
    block.first_frame = first;
    block.ticks = std::span(ticks).subspan(first, frames.size());
    require(output.write(output.owner, block, {}),
            "Legal native completion rounding preserves the nominal musical timeline");
  }
  output.close(output.owner);
  consumer.finish();
  require(consumer.frames() == ticks.size(), "Quantization never adds or loses a frame");
}

void invalid_blocks(const std::filesystem::path &root) {
  auto wrong_rate = initial;
  wrong_rate.frame_ticks = frame_ticks / 2;
  auto underrun = initial;
  underrun.underruns = 1;
  auto discontinuity = initial;
  discontinuity.first_frame = frames.size() + 1;
  auto reset = initial;
  reset.epoch = initial.epoch + 1;
  reset.first_frame = frames.size();
  auto missing_ticks = initial;
  missing_ticks.first_frame = frames.size();
  missing_ticks.ticks = {};
  auto repeated_tick = missing_ticks;
  constexpr std::array<std::uint64_t, 2> repeated{4000, 4000};
  repeated_tick.ticks = repeated;
  auto reversed_tick = repeated_tick;
  constexpr std::array<std::uint64_t, 2> reversed{5000, 4000};
  reversed_tick.ticks = reversed;
  auto old_tick = repeated_tick;
  old_tick.ticks = completion_ticks;
  auto gap = repeated_tick;
  constexpr std::array<std::uint64_t, 2> skipped{5442, 6802};
  gap.ticks = skipped;
  auto stretched = repeated_tick;
  constexpr std::array<std::uint64_t, 2> drift{4770, 6130};
  stretched.ticks = drift;
  const std::array invalid{wrong_rate,
                           underrun,
                           discontinuity,
                           reset,
                           missing_ticks,
                           repeated_tick,
                           reversed_tick,
                           old_tick,
                           gap,
                           stretched,
                           PcmOutput::Block{}};
  for (std::size_t index = 0; index < invalid.size(); ++index) {
    const auto path = root / (std::to_string(index) + ".wav");
    {
      WavConsumer consumer(path, rate);
      const auto output = consumer.output();
      require(output.write(output.owner, initial, {}), "Start with known valid original frames");
      rejects<std::runtime_error>(
          [&] { static_cast<void>(output.write(output.owner, invalid[index], {})); },
          "Reject invalid format, underrun, reset or discontinuity");
      output.close(output.owner);
      rejects<std::logic_error>([&] { consumer.finish(); }, "A failed render cannot publish a WAV");
    }
    require(!std::filesystem::exists(path), "Do not leave partial output at its selected path");
  }
}

void lifecycle(const std::filesystem::path &root) {
  {
    WavConsumer consumer(root / "empty.wav", rate);
    const auto output = consumer.output();
    output.close(output.owner);
    rejects<std::logic_error>([&] { consumer.finish(); },
                              "Zero original frames is not a valid render");
    rejects<std::logic_error>([&] { static_cast<void>(output.write(output.owner, initial, {})); },
                              "A closed DSP owner cannot resume writing");
  }
  {
    WavConsumer consumer(root / "cancelled.wav", rate);
    const auto output = consumer.output();
    // libc++ mutates the source on request_stop(); libstdc++ accepts a const source.
#if defined(_LIBCPP_VERSION)
    std::stop_source stop;
#else
    const std::stop_source stop;
#endif
    stop.request_stop();
    require(!output.write(output.owner, initial, stop.get_token()) && consumer.frames() == 0,
            "Cancellation consumes and counts no frames");
    output.close(output.owner);
  }
  require(!std::filesystem::exists(root / "empty.wav") &&
              !std::filesystem::exists(root / "cancelled.wav"),
          "Empty and cancelled renders do not publish output");
}

void window_handoff(const std::filesystem::path &root) {
  WavConsumer consumer(root / "window-handoff.wav", rate, true);
  const auto output = consumer.output();
  std::binary_semaphore ready{0};
  std::binary_semaphore admitted{0};
  std::exception_ptr failure;
  std::jthread producer([&](const std::stop_token &stop) {
    const std::stop_callback cancelled(stop, [&] { admitted.release(); });
    try {
      require(output.write(output.owner, initial, {}), "Original owner submits startup PCM");
      ready.release();
      admitted.acquire();
      if (stop.stop_requested()) { return; }
      auto next = initial;
      next.first_frame = frames.size();
      constexpr std::array<std::uint64_t, 2> ticks{4081, 5442};
      next.ticks = ticks;
      require(output.write(output.owner, next, {}),
              "Original owner observes admitted render window");
      output.close(output.owner);
    } catch (...) {
      failure = std::current_exception();
      ready.release();
    }
  });
  ready.acquire();
  require(consumer.has_pcm(), "Control observes actual PCM without inspecting owner state");
  constexpr std::uint64_t begin = 3000;
  constexpr std::uint64_t end = 5000;
  consumer.begin_at(begin);
  consumer.end_at(end);
  admitted.release();
  producer.join();
  if (failure) { std::rethrow_exception(failure); }
  require(consumer.complete() && consumer.frames() == 1,
          "One original frame crosses the owner handoff");
  consumer.finish();
}

void owner_handoff(const std::filesystem::path &root) {
  WavConsumer consumer(root / "handoff.wav", rate);
  const auto output = consumer.output();
  std::exception_ptr failure;
  constexpr std::size_t blocks = 1025;
  std::jthread producer([&] {
    try {
      auto block = initial;
      std::array<std::uint64_t, 2> ticks{};
      block.ticks = ticks;
      for (std::size_t index = 0; index < blocks; ++index) {
        ticks[0] = (block.first_frame + 1) * PcmOutput::time_hz / rate;
        ticks[1] = (block.first_frame + 2) * PcmOutput::time_hz / rate;
        require(output.write(output.owner, block, {}), "Owner submits consecutive original frames");
        block.first_frame += frames.size();
      }
      output.close(output.owner);
    } catch (...) { failure = std::current_exception(); }
  });
  producer.join();
  if (failure) { std::rethrow_exception(failure); }
  require(consumer.frames() == blocks * frames.size(), "Read offline totals only after owner join");
  consumer.finish();
}
}

int main(const int argc, char **argv) try {
  require(argc == 2, "Supply a build artifact directory");
  const auto root = std::filesystem::path(argv[1]) / std::to_string(::getpid());
  require(std::filesystem::create_directories(root), "Create owned unit directory");
  successful(root);
  selected_window(root);
  incomplete_window(root);
  quantized_timing(root);
  invalid_blocks(root);
  lifecycle(root);
  window_handoff(root);
  owner_handoff(root);
  std::filesystem::remove_all(root);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
