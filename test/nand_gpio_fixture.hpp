// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_run_clock.hpp"
// Retain the existing public pin-name interface for fixture consumers.
#include "nand_gpio_pins.hpp" // IWYU pragma: export
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_store.hpp"

#include <cstdint>
#include <iosfwd>
#include <optional>
#include <variant>
#include <vector>

namespace blofeld::test {
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

  void after_step(DspRunContext before);

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
