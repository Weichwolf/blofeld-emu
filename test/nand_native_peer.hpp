// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "board/nand_peer.hpp"

#include <cstdint>

namespace blofeld::test {
inline constexpr std::uint32_t nand_timebase_hz = 1080000000;
using NandPeer = blofeld::NandPeer;
using NandPeerSnapshot = blofeld::NandPeerSnapshot;
using NandPeerEvents = blofeld::NandPeerEvents;
using PeerStatus = blofeld::NandPeerStatus;
using PeerDetailed = blofeld::NandPeerDetailed;
}
