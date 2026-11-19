// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "mcu/state_probe_fwd.hpp"

#include <array>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::vi {
// UF32 USB20D6E2F register/configuration bank and local endpoint RAM.
// One MCU owner; UMCR clock/control storage remains with its clock owner.
// Host observations enter on the MCU owner after packet decoding.
class Usb {
  friend class ::blofeld::vi::mcu::StateProbe;

public:
  using Setup = std::array<std::uint8_t, 8>;

  enum class Link : std::uint8_t { disconnected, reset, full_speed, suspended };
  enum class Transfer : std::uint8_t { unavailable, nak, stall, ready };

  struct InputPacket {
    Transfer transfer = Transfer::unavailable;
    std::span<const std::uint8_t> bytes;
  };

  struct Read {
    std::uint16_t offset = 0;
    std::uint8_t bytes = 1;
  };

  struct Write {
    std::uint16_t offset = 0, value = 0;
    std::uint8_t bytes = 1;
  };

  [[nodiscard]] static bool contains(std::uint16_t offset) noexcept;
  [[nodiscard]] bool read(Read access, std::uint16_t &value) const noexcept;
  // control is the actual UMCR value, not a second register mirror.
  [[nodiscard]] bool write(Write access, std::uint16_t control) noexcept;
  [[nodiscard]] bool observe_link(Link link, std::uint16_t control) noexcept;
  [[nodiscard]] bool receive_frame(std::uint16_t frame, std::uint16_t control) noexcept;
  // Dispatch a decoded control request to the controller or the firmware SETUP buffer.
  [[nodiscard]] bool receive_setup(const Setup &packet, std::uint16_t control) noexcept;
  // Addresses include the USB direction bit. Read and acknowledge at the same
  // retired MCU boundary. The view borrows endpoint RAM.
  [[nodiscard]] InputPacket input(std::uint8_t address, std::uint16_t control) noexcept;
  [[nodiscard]] bool acknowledge_input(std::uint8_t address, std::uint16_t control) noexcept;
  [[nodiscard]] Transfer output(std::uint8_t address,
                                std::span<const std::uint8_t> packet,
                                std::uint16_t control) noexcept;
  [[nodiscard]] std::optional<std::uint16_t> pending() const noexcept;
  [[nodiscard]] bool interrupts_enabled() const noexcept;
  [[nodiscard]] bool mapping_pending() const noexcept;
  bool commit_mapping() noexcept;
  // Ordinary RAM/knowledge grants; rebind after a retired bank-select write.
  // Both descriptors and this object's backing storage must outlive Core use.
  [[nodiscard]] std::array<cpu12::Ram, 2> regions(std::uint16_t register_base) noexcept;

private:
  static constexpr std::size_t buffer_bytes = 64;

  struct Buffer {
    std::array<std::uint8_t, buffer_bytes> bytes{}, known{};
  };

  struct LocalEndpoint {
    unsigned slot;
    unsigned maximum;
  };

  [[nodiscard]] std::optional<unsigned> configuration_index() const noexcept;
  [[nodiscard]] unsigned window(std::uint16_t offset) const noexcept;
  [[nodiscard]] bool accepts_packets(std::uint16_t control) const noexcept;
  [[nodiscard]] std::optional<LocalEndpoint>
  local_endpoint(std::uint8_t address, bool in, std::uint16_t control) const noexcept;
  [[nodiscard]] InputPacket input_packet(LocalEndpoint endpoint) const noexcept;
  [[nodiscard]] bool halted(unsigned slot) const noexcept;
  void record_halt(unsigned slot) noexcept;
  void restart_control() noexcept;
  bool write_configuration(Write access, std::uint16_t control) noexcept;
  bool write_byte(Write access) noexcept;
  bool read_byte(std::uint16_t offset, std::uint8_t &value) const noexcept;
  std::array<Buffer, 4> buffers_{};
  std::array<std::uint32_t, 7> configurations_{};
  std::array<std::uint16_t, 8> endpoints_{};
  static constexpr std::uint16_t reset_status = 0xe040;
  std::uint16_t status_ = reset_status, interrupts_ = 0, configuration_status_ = 0;
  std::uint16_t selector_ = 0;
  std::uint16_t frame_ = 0;
  std::uint8_t bound_buffers_ = 0, halted_ = 0;
  bool configuration_status_pending_ = false;
  std::optional<std::uint16_t> staged_high_;
  std::optional<Setup> setup_;
};
}
