// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "io/peer_port.hpp"
#include "io/shi.hpp"

#include <cstdint>

namespace dsp56300 {
class Io;
struct PairedPortTransfer;
}

namespace blofeld::vi::dsp {
// Native channel boundaries shared by the concrete adapter and its compiled IR.
// These functions retain the actual channel owners, synchronization and wakeups.
extern "C" {
[[nodiscard]] bool
blofeld_dsp_shi_read(Shi *channel, std::uint32_t address, std::uint32_t &value) noexcept;
[[nodiscard]] bool
blofeld_dsp_shi_write(Shi *channel, std::uint32_t address, std::uint32_t value) noexcept;
[[nodiscard]] bool
blofeld_dsp_peer_sample(PeerPort *channel, std::uint64_t ticks, PeerPort::Sample &sample) noexcept;
[[nodiscard]] bool blofeld_dsp_peer_ports(PeerPort *channel,
                                          std::uint8_t mask,
                                          std::uint8_t known,
                                          std::uint8_t value,
                                          bool ready_known,
                                          bool active) noexcept;
// Record the currently published output only on an actual PDRC data write.
[[nodiscard]] bool blofeld_dsp_peer_write(PeerPort *channel, std::uint64_t ticks) noexcept;
[[nodiscard]] std::uint64_t
blofeld_native_media_transfer(dsp56300::Io *owner,
                              const dsp56300::PairedPortTransfer *transfer,
                              std::uint64_t request) noexcept;
}
}
