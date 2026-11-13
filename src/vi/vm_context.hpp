// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#if defined(BLOFELD_CENSUS)
#include "census.hpp"
#endif
#include "display/st7565.hpp"
#include "display/state.hpp"
#include "dsp/program.hpp"
#include "execution.hpp"
#include "io/controls.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/pcm_output.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "mcu/program.hpp"
#include "preparation_clock.hpp"
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
            std::shared_ptr<const mcu::Program> mcu_program,
            std::unique_ptr<MediaImage> media = {},
            PcmOutput output = {},
            MidiPort::InputMode input = MidiPort::InputMode::live,
            std::shared_ptr<const dsp::Program> program = {});
  ~VmContext();
  VmContext(const VmContext &) = delete;
  VmContext &operator=(const VmContext &) = delete;
  void request_stop() noexcept;

  void pedal(const bool level) noexcept { controls_.pedal(level); }

  [[nodiscard]] MidiPort &midi() noexcept { return midi_; }

  [[nodiscard]] std::optional<std::uint64_t> mcu_wait_time() const noexcept {
    return time_.mcu_wait_time();
  }

  [[nodiscard]] PreparationClock::Clock::duration execution_elapsed() const {
    return preparation_.elapsed();
  }

  void request_display() noexcept { display_.request(); }

  [[nodiscard]] std::optional<St7565> take_display() { return display_.take(); }

#if defined(BLOFELD_CENSUS)
  [[nodiscard]] const Census &census() const noexcept { return census_; }
#endif
  void start();

  [[nodiscard]] bool finished() const noexcept {
    return completed_.load(std::memory_order_acquire) != 0;
  }

  [[nodiscard]] std::array<ExecutionReport, 2> wait();
  [[nodiscard]] std::array<ExecutionReport, 2> run();

private:
  void join_owners();
  ExecutionTime time_;
  PeerPort peer_;
  std::unique_ptr<FlashImage> image_;
  std::unique_ptr<MediaImage> media_;
  std::stop_source stop_;
  std::shared_ptr<const mcu::Program> mcu_program_;
  std::shared_ptr<const dsp::Program> program_;
  std::jthread mcu_;
  std::jthread dsp_;
  std::vector<dsp56300::Word> boot_;
  PcmOutput output_;
  PreparationClock preparation_;
  DspReset reset_;
  Shi shi_;
  std::array<ExecutionReport, 2> reports_;
  DisplayState display_;
  MidiPort midi_;
  std::atomic<unsigned> completed_{0};
  Controls controls_;
#if defined(BLOFELD_CENSUS)
  Census census_;
#endif
  bool started_ = false;
};
}
