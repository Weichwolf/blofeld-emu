// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <array>
#include <bitset>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace blofeld {

enum class UsbBuffer : std::uint8_t { control_in, control_out, endpoint2, endpoint3 };
enum class UsbResponse : std::uint8_t { none, ack, nak, stall, data };

// Explicit post-PHY/protocol-decoder signals, not raw D+/D- observations.
struct UsbDecoderState {
  bool reset = false;
  bool suspended = false;
  bool enumerated = false;
  bool full_speed = true;
  bool operator==(const UsbDecoderState &) const = default;
};

struct UsbOutPacket {
  UsbBuffer buffer = UsbBuffer::control_out;
  std::span<const std::uint8_t> data;
  bool data1 = false;
  bool crc_valid = true;
};

struct UsbConfigurationRequest {
  std::uint8_t configuration = 0;
  std::optional<std::uint8_t> interface = std::nullopt;
  std::uint8_t alternate = 0;
};

struct UsbInPacket {
  UsbResponse response = UsbResponse::none;
  std::array<std::uint8_t, 64> data{};
  unsigned size = 0;
  bool data1 = false;
};

struct UsbConfigurationInfo {
  std::uint8_t configuration = 0;
  unsigned interfaces = 0;
  std::array<std::uint8_t, 4> alternatives{};
  bool operator==(const UsbConfigurationInfo &) const = default;
};

// Host capture lifetime metadata, not guest-visible hardware registers.
struct UsbHostEvidence {
  std::uint64_t link = 0;
  std::uint64_t descriptions = 0;
  bool operator==(const UsbHostEvidence &) const = default;
};

struct UsbEndpointInfo {
  UsbBuffer buffer = UsbBuffer::control_in;
  std::uint8_t address = 0;
  unsigned maximum_packet = 0;
  unsigned type = 0;         // USB transfer type: 0 control, 2 bulk, 3 interrupt.
  std::optional<bool> data1; // Unknown after an unqualified endpoint reset.
};

// USB20D6E2F register/buffer side, including decoded address/configuration/interface control.
// Wire decoding, other automatic standard requests,
// high-speed packets and IQUE data remain unsupported.
class McuUsb {
public:
  void reset();
  // EPRST resets only its documented register domain; DATA PID state is unqualified.
  void reset_endpoints();
  [[nodiscard]] static std::string_view register_name(std::uint16_t offset);
  [[nodiscard]] static bool configuration_data(std::uint16_t offset);
  [[nodiscard]] std::uint8_t read(std::uint16_t offset) const;
  void write(ByteWrite access);
  void write_memory(MemoryWrite access);
  void set_control(std::uint16_t value);
  void observe_decoder(UsbDecoderState state);
  void observe_sof(std::uint16_t frame, bool crc_valid = true);
  void observe_configuration(UsbConfigurationRequest request);
  UsbResponse receive_setup(const std::array<std::uint8_t, 8> &data, bool crc_valid = true);
  UsbResponse receive_out(UsbOutPacket packet);
  [[nodiscard]] UsbInPacket request_in(UsbBuffer endpoint);
  void finish_in(bool acknowledged);
  [[nodiscard]] UsbEndpointInfo host_endpoint(std::uint8_t address) const;
  [[nodiscard]] UsbConfigurationInfo host_configuration() const;
  [[nodiscard]] bool host_suspended() const;
  [[nodiscard]] std::uint8_t host_address() const;

  [[nodiscard]] UsbHostEvidence host_evidence() const { return host_evidence_; }

  [[nodiscard]] std::uint8_t interrupt_sources() const;

private:
  [[nodiscard]] static std::uint64_t next_host_revision(std::uint64_t revision);

  struct Buffer {
    std::array<std::uint8_t, 64> bytes{};
    std::bitset<64> known;
    std::uint16_t control = 0;
    bool data1 = false;
    bool pid_known = true;
    std::optional<UsbInPacket> transmitted = std::nullopt;
  };

  struct PendingIn {
    unsigned buffer = 0;
    UsbInPacket packet;
  };

  struct AutomaticRead {
    std::uint8_t value = 0;
    bool status = false;
    bool error = false;
  };

  [[nodiscard]] unsigned selected_register() const;
  [[nodiscard]] static unsigned buffer_index(UsbBuffer buffer);
  [[nodiscard]] unsigned window_buffer(std::uint16_t offset) const;
  [[nodiscard]] std::optional<std::uint8_t> read_live(std::uint16_t offset) const;
  bool write_live(ByteWrite access);
  void write_endpoint(ByteWrite access);
  [[nodiscard]] std::uint8_t read_ique_control(std::uint16_t offset) const;
  void write_ique_control(ByteWrite access);
  void require_unconfigured_ique(unsigned index) const;
  void acknowledge_configuration(ByteWrite access);
  [[nodiscard]] bool enabled() const;
  void require_observation() const;
  void require_link() const;
  void require_packet() const;
  void require_control_phase() const;
  [[nodiscard]] bool input_endpoint(unsigned index) const;
  [[nodiscard]] unsigned maximum_packet(unsigned index) const;
  [[nodiscard]] std::optional<UsbConfigurationRequest>
  decode_configuration_setup(const std::array<std::uint8_t, 8> &data) const;
  [[nodiscard]] std::optional<AutomaticRead>
  decode_read_setup(const std::array<std::uint8_t, 8> &data) const;
  [[nodiscard]] std::optional<std::uint8_t>
  decode_address_setup(const std::array<std::uint8_t, 8> &data) const;
  [[nodiscard]] UsbInPacket request_automatic_in();
  [[nodiscard]] UsbInPacket request_local_in(UsbBuffer endpoint, unsigned maximum);
  UsbResponse stall_response(unsigned index);
  std::array<std::uint32_t, 7> configuration_{};
  UsbHostEvidence host_evidence_;
  std::uint8_t selector_ = 0;
  std::uint8_t buffers_ = 0;
  std::optional<std::uint16_t> staged_high_;
  static constexpr std::uint16_t initial_status = 0xe040;
  static constexpr std::uint16_t initial_control = 0x0820;
  std::uint16_t module_control_ = initial_control;
  std::uint16_t status_ = initial_status;
  std::uint16_t interrupt_mask_ = 0;
  std::uint16_t frame_ = 0;
  std::uint16_t configuration_status_ = 0;
  std::array<std::uint8_t, 8> setup_{};
  std::array<Buffer, 4> buffers_data_{};
  std::array<std::uint16_t, 2> ique_controls_{};
  std::optional<PendingIn> pending_in_;
  UsbDecoderState decoder_{};
  bool decoder_known_ = false;
  bool setup_stall_ = false;
  bool control_phase_known_ = true;
  bool configuration_control_ = false;
  std::optional<AutomaticRead> automatic_read_;
  std::optional<std::uint8_t> pending_address_;
  std::uint8_t device_address_ = 0;
  std::array<std::uint8_t, 4> alternate_{};
};

}
