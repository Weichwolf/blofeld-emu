// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_peripheral_calendar.hpp"
#include "storage/nand.hpp"
#include "storage/nand_cell_model.hpp"

#include <cstdint>
#include <functional>
#include <iosfwd>
#include <memory>
#include <optional>

namespace blofeld {
class DspMachine;
class NandWriteStore;

struct MemoryControllerEvents {
  std::shared_ptr<const PeripheralEventIdentity> identity;
  std::function<std::optional<std::uint64_t>()> next_ticks;
  std::function<void(std::uint64_t)> observe;
};

// Firmware GPIO transactions over persistent NAND storage. Only actual register
// accesses and operation completions change state; no setup/hold or pad replay.
class MemoryController {
public:
  MemoryController(DspMachine &machine, Nand device, NandCellProfile profile = {});
  MemoryController(DspMachine &machine, std::unique_ptr<NandWriteStore> store);
  ~MemoryController();
  MemoryController(const MemoryController &) = delete;
  MemoryController &operator=(const MemoryController &) = delete;
  void set_telemetry(std::ostream *output);
  [[nodiscard]] MemoryControllerEvents events();
  void before_native(std::uint64_t ticks);
  void after_native(std::uint64_t ticks, std::optional<std::uint32_t> pc);
  [[nodiscard]] Nand protocol_copy() const;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] bool durable() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
