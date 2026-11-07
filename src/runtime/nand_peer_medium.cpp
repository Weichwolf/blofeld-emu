// SPDX-License-Identifier: GPL-3.0-only
#include "runtime/nand_peer_medium.hpp"

#include "common/bus.hpp"
#include "storage/nand.hpp"
#include "storage/nand_operation_clock.hpp"
#include "storage/nand_write_medium.hpp"
#include "storage/nand_write_store.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <variant>

namespace blofeld {
NandPeerMedium::NandPeerMedium(NandWriteMedium medium) : backend_(std::move(medium)) {}

NandPeerMedium::NandPeerMedium(std::unique_ptr<NandWriteStore> store, const std::uint64_t timebase)
    : backend_(std::move(store)) {
  const auto &owner = std::get<std::unique_ptr<NandWriteStore>>(backend_);
  if (!owner || owner->faulted() || owner->timebase_hz() != timebase || owner->ticks() != 0 ||
      owner->next_event() || owner->device().ready().has_value()) {
    throw Fault("Durable NAND peer requires a fresh healthy owner with matching timebase");
  }
}

bool NandPeerMedium::durable() const {
  return std::holds_alternative<std::unique_ptr<NandWriteStore>>(backend_);
}

const Nand &NandPeerMedium::device() const {
  return inspect([](const auto &medium) -> const Nand & { return medium.device(); });
}

std::uint64_t NandPeerMedium::ticks() const {
  return inspect([](const auto &medium) { return medium.ticks(); });
}

bool NandPeerMedium::faulted() const {
  return inspect([](const auto &medium) { return medium.faulted(); });
}

std::optional<NandCompletionEvent> NandPeerMedium::next_event() const {
  return inspect([](const auto &medium) { return medium.next_event(); });
}

std::optional<NandCompletionEvent> NandPeerMedium::advance(const std::uint64_t at) {
  return with([at](auto &medium) { return medium.advance(at); });
}

void NandPeerMedium::observe_power_ready() {
  with([](auto &medium) { medium.observe_power_ready(); });
}

void NandPeerMedium::command(const std::uint8_t value) {
  with([value](auto &medium) { medium.command(value); });
}

void NandPeerMedium::address(const std::uint8_t value) {
  with([value](auto &medium) { medium.address(value); });
}

void NandPeerMedium::write(const std::uint8_t value) {
  with([value](auto &medium) { medium.write(value); });
}

std::uint8_t NandPeerMedium::read(const std::uint8_t mask) {
  return with([mask](auto &medium) { return medium.read(mask); });
}

void NandPeerMedium::observe_read_end(const NandReadEnd edge) {
  with([edge](auto &medium) { medium.observe_read_end(edge); });
}
}
