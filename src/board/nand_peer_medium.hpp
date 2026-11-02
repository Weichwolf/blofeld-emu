// SPDX-License-Identifier: GPL-3.0-only
#pragma once
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
// One timed device, either checkpointable value storage or an exclusive file owner.
// Live storage never acquires a second protocol or a RAM-restore capability.
class NandPeerMedium {
public:
  explicit NandPeerMedium(NandWriteMedium medium);
  NandPeerMedium(std::unique_ptr<NandWriteStore> store, std::uint64_t timebase);
  NandPeerMedium(const NandPeerMedium &other);
  NandPeerMedium(NandPeerMedium &&other) noexcept = default;
  NandPeerMedium &operator=(const NandPeerMedium &) = delete;
  NandPeerMedium &operator=(NandPeerMedium &&) noexcept = default;
  ~NandPeerMedium() = default;
  [[nodiscard]] bool durable() const;
  [[nodiscard]] const Nand &device() const;
  [[nodiscard]] std::uint64_t ticks() const;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] std::optional<NandCompletionEvent> next_event() const;
  [[nodiscard]] std::optional<NandCompletionEvent> advance(std::uint64_t at);
  void observe_power_ready();
  void command(std::uint8_t value);
  void address(std::uint8_t value);
  void write(std::uint8_t value);
  [[nodiscard]] std::uint8_t read(std::uint8_t mask = Nand::byte_mask);
  void observe_read_end(NandReadEnd edge);

private:
  using Backend = std::variant<NandWriteMedium, std::unique_ptr<NandWriteStore>>;

  template <typename Action> decltype(auto) with(Action action) {
    return std::visit(
        [&action](auto &medium) -> decltype(auto) {
          if constexpr (requires { *medium; }) {
            return action(*medium);
          } else {
            return action(medium);
          }
        },
        backend_);
  }

  template <typename Action> [[nodiscard]] decltype(auto) inspect(Action action) const {
    return std::visit(
        [&action](const auto &medium) -> decltype(auto) {
          if constexpr (requires { *medium; }) {
            return action(std::as_const(*medium));
          } else {
            return action(medium);
          }
        },
        backend_);
  }

  [[nodiscard]] NandWriteMedium copy_medium() const;
  Backend backend_;
};
}
