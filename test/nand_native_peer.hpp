// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_read_timing.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
class DspMachine;
}

namespace blofeld::test {
inline constexpr std::uint32_t nand_timebase_hz = 1080000000;

struct PeerStatus {
  std::uint64_t ticks;
  std::optional<bool> ready;
  blofeld::NandPending pending;
  std::optional<std::uint64_t> next;
  blofeld::NandByte cell;
  bool operator==(const PeerStatus &) const = default;
};

struct PeerDetailed {
  PeerStatus base;
  unsigned page;
  unsigned column;
  blofeld::NandReadOutput output;
  bool power_ready;
  std::optional<unsigned> known_controls;
  std::optional<bool> previous_re;
  bool connected;
  std::uint32_t last_pc;
  std::uint32_t last_omr;
  bool trace_access;
  bool operator==(const PeerDetailed &) const = default;
};

class NandPeer;

class NandPeerSnapshot {
public:
  NandPeerSnapshot(const NandPeerSnapshot &) = default;
  NandPeerSnapshot &operator=(const NandPeerSnapshot &) = default;
  ~NandPeerSnapshot();

private:
  friend class NandPeer;
  struct Storage;
  explicit NandPeerSnapshot(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

// Explicit candidate board wiring. No global device/selector and no retained raw
// bus reference. The native owner replaces its internal bus during restoration.
class NandPeer {
public:
  NandPeer(blofeld::DspMachine &machine, blofeld::Nand device);
  NandPeer(blofeld::DspMachine &machine, blofeld::Nand device, NandCellProfile profile);
  NandPeer(const NandPeerSnapshot &snapshot, blofeld::DspMachine &target);
  ~NandPeer();
  NandPeer(const NandPeer &) = delete;
  NandPeer &operator=(const NandPeer &) = delete;
  [[nodiscard]] blofeld::DspFetchPeripheralEvents events();
  void before_native(std::uint64_t common_ticks);
  void after_native(std::uint64_t common_ticks);
  [[nodiscard]] PeerDetailed detailed() const;
  // One-way complete protocol observation; cannot replace live state or publish IO.
  [[nodiscard]] blofeld::Nand protocol_copy() const;
  [[nodiscard]] NandPeerSnapshot checkpoint() const;
  // Stage payload only, before native/calendar restore. No bus write or callback.
  void restore(const NandPeerSnapshot &snapshot);
  [[nodiscard]] bool faulted() const;

private:
  struct Impl;
  const std::shared_ptr<const bool> lifetime_ = std::make_shared<const bool>(true);
  std::unique_ptr<Impl> impl_;
};
}
