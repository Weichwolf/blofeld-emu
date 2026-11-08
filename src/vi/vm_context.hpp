// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "execution.hpp"
#include "io/controls.hpp"
#include "io/dsp_reset.hpp"
#include "io/pcm_output.hpp"
#include "io/shi.hpp"
#include "storage/flash_image.hpp"
#include "storage/media_image.hpp"

#include <array>
#include <atomic>
#include <dsp56300/memory.hpp>
#include <memory>
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
            PcmOutput output = {});
  ~VmContext();
  VmContext(const VmContext &) = delete;
  VmContext &operator=(const VmContext &) = delete;
  void request_stop() noexcept;

  void pedal(const bool level) noexcept { controls_.pedal(level); }

  [[nodiscard]] std::array<ExecutionReport, 2> run();

private:
  std::unique_ptr<FlashImage> image_;
  std::vector<dsp56300::Word> boot_;
  std::unique_ptr<MediaImage> media_;
  Controls controls_;
  PcmOutput output_;
  Shi shi_;
  DspReset reset_;
  std::array<ExecutionReport, 2> reports_;
  std::atomic<unsigned> completed_{0};
  std::jthread mcu_;
  std::jthread dsp_;
};
}
