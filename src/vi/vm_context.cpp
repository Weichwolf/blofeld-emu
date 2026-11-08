// SPDX-License-Identifier: GPL-3.0-only
#include "vm_context.hpp"

#include "dsp/execute.hpp"
#include "execution.hpp"
#include "mcu/execute.hpp"
#include "storage/flash_image.hpp"

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
VmContext::VmContext(std::unique_ptr<FlashImage> image, std::vector<dsp56300::Word> boot)
    : image_(std::move(image)), boot_(std::move(boot)) {
  if (!image_) { throw std::invalid_argument("VM requires a persistent MCU image"); }
}

VmContext::~VmContext() {
  request_stop();
}

void VmContext::request_stop() noexcept {
  mcu_.request_stop();
  dsp_.request_stop();
}

std::array<ExecutionReport, 2> VmContext::run() {
  if (mcu_.joinable() || dsp_.joinable() || completed_.load(std::memory_order_relaxed) != 0) {
    throw std::logic_error("VM execution can only be started once");
  }
  mcu_ = std::jthread([this](const std::stop_token &stop) {
    try {
      reports_[0] = mcu::execute(*image_, shi_, stop);
    } catch (...) { reports_[0].exception = std::current_exception(); }
    completed_.fetch_add(1, std::memory_order_release);
    completed_.notify_one();
  });
  dsp_ = std::jthread([this](const std::stop_token &stop) {
    try {
      reports_[1] = dsp::execute(boot_, shi_, stop);
    } catch (...) { reports_[1].exception = std::current_exception(); }
    completed_.fetch_add(1, std::memory_order_release);
    completed_.notify_one();
  });
  completed_.wait(0, std::memory_order_acquire);
  request_stop();
  mcu_.join();
  dsp_.join();
  return reports_;
}
}
