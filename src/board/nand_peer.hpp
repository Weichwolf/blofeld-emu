// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "board/nand_bus_decode.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_read_timing.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

namespace blofeld {
class OwnDspMachine;
}

namespace blofeld {

struct NandPeerStatus {
  std::uint64_t ticks;
  std::optional<bool> ready;
  blofeld::NandPending pending;
  std::optional<std::uint64_t> next;
  blofeld::NandByte cell;
  bool operator==(const NandPeerStatus &) const = default;
};

struct NandPeerDetailed {
  NandPeerStatus base;
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
  NandBusContract bus_contract;
  bool neutral_bus;
  bool operator==(const NandPeerDetailed &) const = default;
};

struct NandPeerEvents {
  std::shared_ptr<const PeripheralEventIdentity> identity;
  std::function<std::optional<std::uint64_t>()> next_ticks;
  std::function<void(std::uint64_t)> observe;
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
  NandPeer(blofeld::OwnDspMachine &machine, blofeld::Nand device);
  NandPeer(blofeld::OwnDspMachine &machine, blofeld::Nand device, NandCellProfile profile);
  NandPeer(blofeld::OwnDspMachine &machine,
           blofeld::Nand device,
           NandCellProfile profile,
           NandBusContract contract);
  NandPeer(const NandPeerSnapshot &snapshot, blofeld::OwnDspMachine &target);
  ~NandPeer();
  NandPeer(const NandPeer &) = delete;
  NandPeer &operator=(const NandPeer &) = delete;
  [[nodiscard]] NandPeerEvents events();
  void before_native(std::uint64_t common_ticks);
  void after_native(std::uint64_t common_ticks, std::optional<std::uint32_t> native_pc);
  [[nodiscard]] NandPeerDetailed detailed() const;
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
