// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/boot_rom.hpp"
#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/pcm_output.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "preparation_clock.hpp"
#include "storage/media_image.hpp"

#include <stop_token>

namespace blofeld::vi::dsp {
[[nodiscard]] ExecutionReport execute(BootRom boot,
                                      Shi &shi,
                                      DspReset &reset,
                                      const std::stop_token &stop,
                                      MediaImage *image = nullptr,
                                      PcmOutput output = {},
                                      PeerPort *peer = nullptr,
                                      ExecutionTime *time = nullptr,
                                      MidiPort *midi = nullptr,
                                      PreparationClock *preparation = nullptr);
}
