// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/nand_read.hpp"

#include <cstdint>
#include <iosfwd>
#include <optional>

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
  unsigned poll_pc = 0;
  std::optional<bool> selected;
};

// Candidate route only. Each native instruction is followed by a pin observation.
// Completion after three actual ready polls is an explicit untimed lab event.
class NandLink {
public:
  NandLink(DspBus &bus, NandRead &nand, std::ostream &trace, NandLinkConditions conditions);
  void after_step(const DspState &before);

  [[nodiscard]] unsigned commands() const { return commands_; }

  [[nodiscard]] unsigned addresses() const { return addresses_; }

  [[nodiscard]] unsigned reads() const { return reads_; }

  [[nodiscard]] unsigned polls() const { return polls_; }

private:
  void observe_edges();
  void latch_write(unsigned controls);
  DspBus &bus_;
  NandRead &nand_;
  std::ostream &trace_;
  NandLinkConditions conditions_;
  std::optional<unsigned> previous_;
  unsigned commands_ = 0;
  unsigned addresses_ = 0;
  unsigned reads_ = 0;
  unsigned polls_ = 0;
};
}
