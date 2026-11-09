// SPDX-License-Identifier: GPL-3.0-only
#include "audio/playback.hpp"

#include <chrono>

namespace blofeld::host::audio {
struct Playback::Producer::Impl {};

Playback::Producer::Producer(std::chrono::nanoseconds /*period*/) {}

Playback::Producer::Producer(Producer &&) noexcept = default;
Playback::Producer::~Producer() = default;

void Playback::Producer::begin_work() {}

void Playback::Producer::end_work() {}
}
