// SPDX-License-Identifier: GPL-3.0-only
#include "audio/pcm_sink.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "mcu/hc12.hpp"

#include <string>

namespace blofeld::reference {
void Board::attach_pcm_output(PcmFrameSink &sink) {
  require_healthy();
  if (!config_.pcm) { throw Fault("External audio requires a decoded PCM route"); }
  if (config_.checkpoint) { throw Fault("External audio rejects automatic checkpoint replay"); }
  if (pcm_output_ == &sink) { throw Fault("External audio sink is already bound"); }
  sink.check();
  sink.claim_source(identity_);
  retire_pcm_output();
  pcm_output_ = &sink;
}

void Board::retire_pcm_output() noexcept {
  if (pcm_output_ != nullptr) { pcm_output_->invalidate(); }
}

void Board::mark_fault() noexcept {
  control_.faulted = true;
  retire_pcm_output();
}

void Board::check_pcm_output() const {
  if (pcm_output_ == nullptr) { return; }
  try {
    pcm_output_->check();
  } catch (const Fault &error) {
    throw Fault("Reference PCM output check at=" + std::to_string(ticks()) +
                " mcu-pc=" + hex(native_->mcu.cpu().state().pc) +
                " dsp-pc=" + hex(native_->dsp.cpu().state().pc, 6) + ": " + error.what());
  }
}
}
