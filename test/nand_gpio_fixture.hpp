// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/nand.hpp"
#include "blofeld/nand_array.hpp"
#include "blofeld/nand_store.hpp"

#include <cstdint>
#include <iosfwd>
#include <optional>
#include <variant>
#include <vector>

namespace blofeld::test {
inline constexpr unsigned nand_controls = 0xf00;
inline constexpr unsigned nand_re = 0x100;
inline constexpr unsigned nand_we = 0x200;
inline constexpr unsigned nand_cle = 0x400;
inline constexpr unsigned nand_ale = 0x800;
inline constexpr unsigned nand_data = 0xff;
inline constexpr unsigned nand_other_f = 0x300;
inline constexpr unsigned nand_timer_one = 1;

struct NandLinkConditions {
  std::vector<unsigned> poll_pcs;
  std::optional<bool> selected = std::nullopt;
  std::vector<NandOutcome> outcomes;
};

// Candidate route only. Each native instruction is followed by a pin observation.
// Completion after three actual ready polls is an explicit untimed lab event.
class NandLink {
public:
  NandLink(DspBus &bus, Nand &nand, std::ostream &trace, NandLinkConditions conditions);
  NandLink(DspBus &bus, NandStore &nand, std::ostream &trace, NandLinkConditions conditions);
  void after_step(const DspState &before);

  [[nodiscard]] unsigned commands() const { return commands_; }

  [[nodiscard]] unsigned addresses() const { return addresses_; }

  [[nodiscard]] unsigned reads() const { return reads_; }

  [[nodiscard]] unsigned polls() const { return polls_; }

  [[nodiscard]] unsigned writes() const { return writes_; }

  [[nodiscard]] unsigned completed_writes() const { return completed_writes_; }

private:
  using Device = std::variant<Nand *, NandStore *>;
  NandLink(DspBus &bus, Device device, std::ostream &trace, NandLinkConditions conditions);

  template <typename Action> decltype(auto) with(Action action) {
    return std::visit([&action](auto *device) -> decltype(auto) { return action(*device); },
                      device_);
  }

  void observe_edges();
  void latch_write(unsigned controls);
  void drive_status();
  void complete_operation();
  DspBus &bus_;
  Device device_;
  std::ostream &trace_;
  NandLinkConditions conditions_;
  std::optional<unsigned> previous_;
  unsigned commands_ = 0;
  unsigned addresses_ = 0;
  unsigned reads_ = 0;
  unsigned polls_ = 0;
  unsigned operation_polls_ = 0;
  unsigned writes_ = 0;
  unsigned completed_writes_ = 0;
  bool reading_ = false;
  std::optional<NandByte> previous_status_;
};
}
