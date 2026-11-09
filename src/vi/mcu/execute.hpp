// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/state.hpp"
#include "execution.hpp"
#include "io/controls.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "storage/flash_image.hpp"

#include <stop_token>

namespace blofeld::vi::mcu {
[[nodiscard]] ExecutionReport execute(FlashImage &image,
                                      Shi &shi,
                                      DspReset &reset,
                                      const std::stop_token &stop,
                                      const Controls *controls = nullptr,
                                      DisplayState *display = nullptr,
                                      PeerPort *peer = nullptr,
                                      ExecutionTime *time = nullptr,
                                      MidiPort *midi = nullptr);
}
