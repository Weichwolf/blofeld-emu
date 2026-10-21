// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/pcm_recording.hpp"
#include "blofeld/serial_pcm.hpp"
#include "pcm_transport_fixture.hpp"
#include "serial_pcm_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <type_traits>

namespace blofeld::test::pcm {
namespace {
void copy_on_write_histories() {
  static_assert(std::is_nothrow_copy_constructible_v<PcmRecording>);
  static_assert(std::is_nothrow_copy_assignable_v<PcmRecording>);
  PcmRecording recording({}, 3);
  recording.append(frame());
  const auto snapshot = recording;
  require(recording.frames().data() == snapshot.frames().data(),
          "A checkpoint shares the completed prefix without copying or allocating samples");
  recording.append(frame(second_tick));
  const auto completed = recording;
  require(recording.frames().size() == 2 && snapshot.frames().size() == 1 &&
              snapshot.frames()[0].samples[0] == first_word &&
              recording.frames().data() != snapshot.frames().data(),
          "Appending after capture preserves an immutable one-frame snapshot");
  recording = snapshot;
  recording.append(frame(second_tick));
  require(recording == completed, "Restoring and replaying reconstructs the entire sample history");
  recording = snapshot;
  auto changed = frame(second_tick);
  changed.samples[0] = 0;
  recording.append(changed);
  require(recording != completed && completed.frames()[1].samples[0] == first_word,
          "An alternative branch cannot overwrite a previously completed branch");
  PcmRecording rebuilt({}, 3);
  auto changed_prefix = frame();
  changed_prefix.samples[0] = 0;
  rebuilt.append(changed_prefix);
  rebuilt.append(frame(second_tick));
  require(rebuilt != completed,
          "History comparison checks earlier frames even when last frame and count match");
}

void bounded_rejections_preserve_prefix() {
  PcmRecording disabled;
  require(!disabled && disabled.frames().empty(), "A disabled recording has no allocated history");
  (void)rejects([&] { disabled.append(frame()); });
  PcmRecording recording({}, 1);
  recording.append(frame());
  const auto saved = recording;
  require(rejects([&] { recording.append(frame(second_tick)); }).find("budget exhausted") !=
                  std::string::npos &&
              recording == saved,
          "A full recording rejects without dropping or replacing its known prefix");
  for (unsigned variant = 0; variant < 4; ++variant) {
    PcmRecording target({}, 3);
    target.append(frame());
    const auto before = target;
    auto invalid = frame(second_tick);
    if (variant == 0) { invalid.channels = 1; }
    if (variant == 1) { invalid.last_tick = 0; }
    if (variant == 2) { invalid.first_tick = first_tick; }
    if (variant == 3) { invalid.samples[0] = std::numeric_limits<std::int32_t>::max(); }
    (void)rejects([&] { target.append(invalid); });
    require(target == before, "Shape, chronology and precision errors preserve every saved frame");
  }
  for (const auto capacity : {std::size_t{0}, PcmRecording::maximum_capacity + 1}) {
    (void)rejects([&] { const PcmRecording invalid({}, capacity); });
  }
  (void)rejects([] { const PcmRecording invalid({.word_bits = 0}, 1); });
}
}
}

int main(const int count, char **values) {
  try {
    using namespace blofeld::test::pcm;
    require(count == 2, "PCM transport test needs a private output directory");
    const std::filesystem::path directory(values[1]);
    std::filesystem::create_directories(directory);
    copy_on_write_histories();
    bounded_rejections_preserve_prefix();
    wave_checks(directory);
    std::cout
        << "PCM complete histories, bounds, timing, literal WAV and atomic publication passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
