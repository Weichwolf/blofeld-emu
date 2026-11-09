// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/st7565.hpp"
#include "display/state.hpp"
#include "execution.hpp"
#include "io/controls.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/pcm_output.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "storage/flash_image.hpp"
#include "storage/media_image.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <memory>
#include <optional>
#include <stop_token>
#include <thread>
#include <vector>

namespace blofeld::vi {
// Owns stable input memory and independent execution owners. Device channels and
// audio consumers extend this context; no host API handles or global stepper.
class VmContext {
public:
  VmContext(std::unique_ptr<FlashImage> image,
            std::vector<dsp56300::Word> boot,
            std::unique_ptr<MediaImage> media = {},
            PcmOutput output = {},
            MidiPort::InputMode input = MidiPort::InputMode::live);
  ~VmContext();
  VmContext(const VmContext &) = delete;
  VmContext &operator=(const VmContext &) = delete;
  void request_stop() noexcept;

  void pedal(const bool level) noexcept { controls_.pedal(level); }

  [[nodiscard]] MidiPort &midi() noexcept { return midi_; }

  [[nodiscard]] std::optional<std::uint64_t> mcu_wait_time() const noexcept {
    return time_.mcu_wait_time();
  }

  void request_display() noexcept { display_.request(); }

  [[nodiscard]] std::optional<St7565> take_display() { return display_.take(); }

  void start();

  [[nodiscard]] bool finished() const noexcept {
    return completed_.load(std::memory_order_acquire) != 0;
  }

  [[nodiscard]] std::array<ExecutionReport, 2> wait();
  [[nodiscard]] std::array<ExecutionReport, 2> run();

private:
  std::unique_ptr<FlashImage> image_;
  std::vector<dsp56300::Word> boot_;
  std::unique_ptr<MediaImage> media_;
  Controls controls_;
  MidiPort midi_;
  DisplayState display_;
  PcmOutput output_;
  Shi shi_;
  PeerPort peer_;
  ExecutionTime time_;
  DspReset reset_;
  std::array<ExecutionReport, 2> reports_;
  std::atomic<unsigned> completed_{0};
  std::stop_source stop_;
  bool started_ = false;
  std::jthread mcu_;
  std::jthread dsp_;
};
}
