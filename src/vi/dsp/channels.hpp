// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "io/peer_port.hpp"
#include "io/shi.hpp"

#include <cstdint>

namespace blofeld::vi::dsp {
// Native channel boundaries shared by the concrete adapter and its compiled IR.
// These functions retain the actual channel owners, synchronization and wakeups.
extern "C" {
[[nodiscard]] bool
blofeld_dsp_shi_read(Shi *channel, std::uint32_t address, std::uint32_t &value) noexcept;
[[nodiscard]] bool
blofeld_dsp_shi_write(Shi *channel, std::uint32_t address, std::uint32_t value) noexcept;
void blofeld_dsp_peer_sample(PeerPort *channel,
                             std::uint64_t ticks,
                             PeerPort::Sample &sample) noexcept;
void blofeld_dsp_peer_data(PeerPort *channel,
                           std::uint8_t mask,
                           std::uint8_t known,
                           std::uint8_t value) noexcept;
// Record the currently published output only on an actual PDRC data write.
[[nodiscard]] bool blofeld_dsp_peer_write(PeerPort *channel, std::uint64_t ticks) noexcept;
[[nodiscard]] bool blofeld_dsp_peer_ready(PeerPort *channel, bool active) noexcept;
// A missing request leaves generation untouched; generation zero is not a sentinel.
[[nodiscard]] bool
blofeld_dsp_peer_interrupt(const PeerPort *channel, bool edge, std::uint64_t &generation) noexcept;
}
}
