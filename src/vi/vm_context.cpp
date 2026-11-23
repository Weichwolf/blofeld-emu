// SPDX-License-Identifier: GPL-3.0-only
#include "vm_context.hpp"

#include "mcu/program.hpp"

#if defined(BLOFELD_CENSUS)
#include "census.hpp"
#include "mcu/state_probe.hpp"
#include "process/configuration.hpp"
#endif

#include "dsp/boot_rom.hpp"
#include "dsp/execute.hpp"
#include "dsp/memory.hpp"
#include "dsp/program.hpp"
#include "execution.hpp"
#include "io/coupling.hpp"
#include "io/midi_port.hpp"
#include "io/pcm_output.hpp"
#include "mcu/execute.hpp"
#include "storage/flash_image.hpp"
#include "storage/media_image.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <exception>
#include <memory>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>

namespace blofeld::vi {
VmContext::VmContext(std::unique_ptr<FlashImage> image,
                     std::vector<dsp56300::Word> boot,
                     std::shared_ptr<const mcu::Program> mcu_program,
                     std::unique_ptr<MediaImage> media,
                     const PcmOutput output,
                     const MidiPort::InputMode input,
                     std::shared_ptr<const dsp::Program> program,
                     const std::uint64_t speculation_window,
                     const dsp::Memory::Storage dsp_memory)
    : image_(std::move(image)),
      media_(std::move(media)),
      mcu_program_(std::move(mcu_program)),
      program_(std::move(program)),
      dsp_memory_(dsp_memory),
      boot_(std::move(boot)),
      output_(output),
      midi_(input) {
#if defined(BLOFELD_CENSUS)
  if (host::inherited_setting("BLOFELD_MCU_STATE_PROBE") == "1") {
    state_probe_ = std::make_unique<mcu::StateProbe>(&stop_);
  }
#endif
  if (speculation_window == 0) {
    throw std::invalid_argument("A speculative window must be positive");
  }
  coupling_ = std::make_unique<Coupling>(
      peer_, shi_, reset_, time_, midi_, stop_.get_token(), speculation_window);
  if (!image_) { throw std::invalid_argument("VM requires a persistent MCU image"); }
  if (!mcu_program_) { throw std::invalid_argument("VM requires a complete prepared MCU program"); }
  if (!program_) { throw std::invalid_argument("VM requires a complete prepared DSP program"); }
}

VmContext::~VmContext() {
  request_stop();
  join_owners();
}

void VmContext::join_owners() {
  if (mcu_.joinable()) { mcu_.join(); }
  if (dsp_.joinable()) { dsp_.join(); }
}

void VmContext::request_stop() noexcept {
  stop_.request_stop();
}

void VmContext::start(const OwnerSetup &setup) {
  if (started_) { throw std::logic_error("VM execution can only be started once"); }
  started_ = true;
  mcu_ = std::jthread([this, setup] {
    const auto stop = stop_.get_token();
    try {
      if (setup) { setup(Owner::mcu); }
#if defined(BLOFELD_CENSUS)
      const Census::Scope observed(census_, Census::Actor::mcu);
      const mcu::StateProbe::Scope probed(state_probe_.get());
#endif
      reports_[0] = mcu::execute(*image_,
                                 *mcu_program_,
                                 shi_,
                                 reset_,
                                 stop,
                                 &controls_,
                                 &display_,
                                 &peer_,
                                 &time_,
                                 &midi_,
                                 coupling_.get());
    } catch (...) { reports_[0].exception = std::current_exception(); }
    completed_.fetch_add(1, std::memory_order_release);
    completed_.notify_one();
  });
  dsp_ = std::jthread([this, setup] {
    const auto stop = stop_.get_token();
    try {
      if (setup) { setup(Owner::dsp); }
#if defined(BLOFELD_CENSUS)
      const Census::Scope observed(census_, Census::Actor::dsp);
#endif
      reports_[1] = dsp::execute(dsp::BootRom::document(boot_),
                                 shi_,
                                 reset_,
                                 stop,
                                 media_.get(),
                                 output_,
                                 &peer_,
                                 &time_,
                                 &midi_,
                                 &preparation_,
                                 program_.get(),
                                 coupling_.get(),
                                 dsp_memory_);
    } catch (...) { reports_[1].exception = std::current_exception(); }
    completed_.fetch_add(1, std::memory_order_release);
    completed_.notify_one();
  });
}

std::array<ExecutionReport, 2> VmContext::wait() {
  if (!started_) { throw std::logic_error("VM execution has not started"); }
  completed_.wait(0, std::memory_order_acquire);
  request_stop();
  join_owners();
  return reports_;
}

std::array<ExecutionReport, 2> VmContext::run() {
  start();
  return wait();
}

VmContext::Storage VmContext::stopped_storage() {
  if (!started_ || mcu_.joinable() || dsp_.joinable()) {
    throw std::logic_error("Storage serialization requires joined execution owners");
  }
  auto device = image_->encode();
  const auto changed_media =
      media_ && media_->needs_encoding() ? media_->encoded() : std::span<const std::uint8_t>{};
  return {.device = std::move(device), .changed_media = changed_media};
}
}
