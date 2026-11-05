// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "nand_native_peer.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld::test {
struct NandNativeCpu {
  dsp56300::Registers cpu;
  dsp56300::Progress progress;
  bool operator==(const NandNativeCpu &) const = default;
};

[[nodiscard]] NandNativeCpu native_cpu(const OwnDspMachine &machine);

struct NandNativeState {
  std::uint64_t horizon = 0;
  std::shared_ptr<const PeripheralEventIdentity> external_identity;
  std::optional<std::uint64_t> next_external = std::nullopt;
  std::uint64_t external_events = 0;
  bool operator==(const NandNativeState &) const = default;
};

// Test-only reference order: inherited input, NAND deadline, actual own-core
// effect/F2 cycle, then resulting GPIO observations. No silicon phase claim.
class NandNativeRun {
public:
  NandNativeRun(OwnDspMachine &machine, NandPeer &peer, NandPeerEvents inherited);
  [[nodiscard]] OwnDspFetchCycleResult tick();
  [[nodiscard]] PeripheralCalendarState peripheral_state() const;
  [[nodiscard]] NandNativeState state() const;

private:
  void verify() const;
  void reset_to(std::uint64_t horizon);
  void drain(std::uint64_t horizon, std::optional<std::uint64_t> native);
  OwnDspMachine &machine_;
  NandPeer &peer_;
  NandPeerEvents inherited_;
  NandNativeState state_;
  PeripheralCalendar calendar_;
  std::unique_ptr<OwnDspTimeline> timeline_;
  bool faulted_ = false;
};
}
