// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "storage/nand.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_file.hpp"
#include "storage/nand_image.hpp"
#include "storage/nand_operation_clock.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>

namespace blofeld {
// Durable owner of the timed cell model. Host publication precedes visible
// persistent transitions; ordinary reads never write the backing file.
class NandWriteStore {
public:
  NandWriteStore(const std::filesystem::path &path,
                 NandReadContinuation continuation,
                 std::uint64_t timebase,
                 NandCellProfile profile);
  // A seed is used only for a missing file. Existing/corrupt state is never repaired.
  NandWriteStore(const std::filesystem::path &path,
                 const NandImage &seed,
                 NandReadContinuation continuation,
                 std::uint64_t timebase,
                 NandCellProfile profile);
  ~NandWriteStore();
  NandWriteStore(const NandWriteStore &) = delete;
  NandWriteStore &operator=(const NandWriteStore &) = delete;
  void set_observer(NandFileObserver observer);
  [[nodiscard]] const Nand &device() const;
  [[nodiscard]] std::uint64_t ticks() const;
  [[nodiscard]] std::uint64_t timebase_hz() const;
  [[nodiscard]] NandReadContinuation continuation() const;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] std::optional<NandCompletionEvent> next_event() const;
  [[nodiscard]] std::optional<NandCompletionEvent> advance(std::uint64_t at);
  void observe_power_ready();
  void observe_identification(NandIdentification identification);
  void observe_wp(std::optional<bool> high);
  void command(std::uint8_t value);
  void address(std::uint8_t value);
  void write(std::uint8_t value);
  [[nodiscard]] std::uint8_t read(std::uint8_t mask = Nand::byte_mask);
  void observe_read_end(NandReadEnd edge);
  void power_loss();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
