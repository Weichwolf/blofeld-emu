// SPDX-License-Identifier: GPL-3.0-only
#include "vm_context.hpp"

#include "dsp/execute.hpp"
#include "execution.hpp"
#include "io/pcm_output.hpp"
#include "mcu/execute.hpp"
#include "storage/flash_image.hpp"
#include "storage/media_image.hpp"

#include <array>
#include <atomic>
#include <dsp56300/memory.hpp>
#include <exception>
#include <memory>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>

namespace blofeld::vi {
VmContext::VmContext(std::unique_ptr<FlashImage> image,
                     std::vector<dsp56300::Word> boot,
                     std::unique_ptr<MediaImage> media,
                     const PcmOutput output)
    : image_(std::move(image)), boot_(std::move(boot)), media_(std::move(media)), output_(output) {
  if (!image_) { throw std::invalid_argument("VM requires a persistent MCU image"); }
}

VmContext::~VmContext() {
  request_stop();
}

void VmContext::request_stop() noexcept {
  stop_.request_stop();
}

void VmContext::start() {
  if (started_) { throw std::logic_error("VM execution can only be started once"); }
  started_ = true;
  mcu_ = std::jthread([this] {
    const auto stop = stop_.get_token();
    try {
      reports_[0] =
          mcu::execute(*image_, shi_, reset_, stop, &controls_, &display_, &peer_, &time_, &midi_);
    } catch (...) { reports_[0].exception = std::current_exception(); }
    completed_.fetch_add(1, std::memory_order_release);
    completed_.notify_one();
  });
  dsp_ = std::jthread([this] {
    const auto stop = stop_.get_token();
    try {
      reports_[1] =
          dsp::execute(boot_, shi_, reset_, stop, media_.get(), output_, &peer_, &time_, &midi_);
    } catch (...) { reports_[1].exception = std::current_exception(); }
    completed_.fetch_add(1, std::memory_order_release);
    completed_.notify_one();
  });
}

std::array<ExecutionReport, 2> VmContext::wait() {
  if (!started_) { throw std::logic_error("VM execution has not started"); }
  completed_.wait(0, std::memory_order_acquire);
  request_stop();
  if (mcu_.joinable()) { mcu_.join(); }
  if (dsp_.joinable()) { dsp_.join(); }
  return reports_;
}

std::array<ExecutionReport, 2> VmContext::run() {
  start();
  return wait();
}
}
