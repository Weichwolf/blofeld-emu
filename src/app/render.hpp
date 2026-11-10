// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/wav_consumer.hpp"
#include "execution.hpp"
#include "io/pcm_output.hpp"
#include "midi/file.hpp"
#include "options.hpp"
#include "process/signals.hpp"
#include "vm_context.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stop_token>

namespace blofeld::app {
class OfflineRender {
public:
  explicit OfflineRender(const Options &options);
  [[nodiscard]] vi::PcmOutput output() noexcept;
  int run(vi::VmContext &vm,
          const host::Signals &signals,
          std::chrono::steady_clock::time_point launched);

private:
  [[nodiscard]] std::optional<std::uint64_t> startup(vi::VmContext &vm,
                                                     const host::Signals &signals);
  void feed(vi::MidiPort &port, std::uint64_t origin, const std::stop_token &stop);
  void startup_statistics() const;
  void statistics() const;
  WavConsumer wav_;
  MidiFile midi_;
  std::uint64_t tail_;
  vi::VmContext *vm_ = nullptr;
  std::chrono::steady_clock::time_point launched_, started_, ready_, finished_;
  std::chrono::steady_clock::duration startup_execution_{};
};
}
