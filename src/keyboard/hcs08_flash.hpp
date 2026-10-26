// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"

#include <array>
#include <bitset>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
namespace hcs08_flash {
constexpr unsigned first = 0xe000, bytes = 8192, page_bytes = 512;
constexpr unsigned divider = 0x1820, options = 0x1821, configuration = 0x1823;
constexpr unsigned protection = 0x1824, status = 0x1825, command = 0x1826;
constexpr unsigned nv_protection = 0xffbd, nv_options = 0xffbf;
constexpr unsigned empty = 0x80, complete = 0x40, violation = 0x20, access_error = 0x10;
constexpr unsigned blank = 4, erased = 255;
constexpr unsigned verify = 5, program = 0x20, burst = 0x25, erase_page = 0x40, erase_all = 0x41;
}

struct Hcs08FlashImage {
  std::array<std::uint8_t, hcs08_flash::bytes> data{};
  std::bitset<hcs08_flash::bytes> known;
  std::bitset<hcs08_flash::bytes> fresh;
  bool operator==(const Hcs08FlashImage &) const = default;
};

enum class Hcs08FlashAgent : std::uint8_t { cpu, background };
enum class Hcs08FlashContract : std::uint8_t { unqualified, candidate_gt8a };
enum class Hcs08IdleLaunch : std::uint8_t { unqualified, reference_ignore_empty };
// External observations, not poll-driven progress or qualified chip latencies.
enum class Hcs08FlashMilestone : std::uint8_t { transfer_buffer, complete_command };
enum class Hcs08FlashAbort : std::uint8_t { unqualified_retention, reference_command_range };
enum class Hcs08FlashBuffer : std::uint8_t { empty, data, command, launched };

struct Hcs08FlashJob {
  std::uint16_t address = 0;
  std::uint8_t value = 0;
  std::uint8_t command = 0;
  bool operator==(const Hcs08FlashJob &) const = default;
};

struct Hcs08FlashState {
  Hcs08FlashBuffer buffer = Hcs08FlashBuffer::empty;
  Hcs08FlashJob staged;
  std::optional<Hcs08FlashJob> active;
  std::uint8_t errors = 0;
  std::uint8_t divider = 0;
  bool divider_loaded = false;
  bool blank = false;
  bool operator==(const Hcs08FlashState &) const = default;
};

// Explicit candidate peripheral, with no default board or clock connection.
// Copies are independent reference checkpoints; no callbacks/peers are retained.
class Hcs08Flash {
public:
  explicit Hcs08Flash(const Hcs08FlashImage &image,
                      Hcs08FlashContract contract = Hcs08FlashContract::unqualified,
                      Hcs08IdleLaunch idle = Hcs08IdleLaunch::unqualified);
  void context(ExecutionContext value, Hcs08FlashAgent agent = Hcs08FlashAgent::cpu);
  [[nodiscard]] std::uint8_t read(std::uint16_t address);
  void write(ByteWrite access);
  [[nodiscard]] std::uint8_t read_array(std::uint16_t address) const;
  // CPU reset fetch only: FFFE/FFFF are independent of pre-reset instruction SEC
  // privilege. This does not grant BDC access, supply unknown data or complete NVM.
  [[nodiscard]] std::uint8_t read_reset_vector_byte(std::uint16_t address) const;
  // Hardware vector fetch only; ordinary CPU/BDC array reads remain direct.
  [[nodiscard]] std::uint8_t read_interrupt_vector_byte(std::uint16_t address) const;
  void stage(ByteWrite access);
  void observe_bus_clock(std::uint32_t hz);
  void advance(Hcs08FlashMilestone milestone);
  [[nodiscard]] Hcs08FlashState state() const;
  [[nodiscard]] bool security_engaged() const;
  [[nodiscard]] const Hcs08FlashImage &image() const;
  [[nodiscard]] Hcs08FlashImage
  power_off_image(Hcs08FlashAbort contract = Hcs08FlashAbort::unqualified_retention) const;
  void reset(Hcs08FlashAbort contract = Hcs08FlashAbort::unqualified_retention);

private:
  struct KnownByte {
    std::uint8_t value = 0;
    std::uint8_t known = 0;
  };

  struct RegisterRead {
    unsigned address;
    unsigned mask;
  };

  [[nodiscard]] bool secure() const;
  [[nodiscard]] bool blocked() const;
  [[nodiscard]] unsigned index(std::uint16_t address) const;
  [[nodiscard]] bool protected_address(unsigned address) const;
  [[nodiscard]] bool command_protected(Hcs08FlashJob job) const;
  [[nodiscard]] std::uint8_t status() const;
  [[nodiscard]] std::uint8_t known_register(KnownByte value, RegisterRead access) const;
  [[noreturn]] void
  fail(const std::string &operation, unsigned address, const std::string &reason) const;
  void reject(std::uint8_t errors);
  void write_status(std::uint8_t value);
  void write_command(std::uint8_t value);
  void transfer();
  void finish();
  void load_configuration();
  Hcs08FlashImage image_;
  Hcs08IdleLaunch idle_;
  Hcs08FlashState state_;
  KnownByte options_;
  KnownByte protection_;
  std::optional<std::uint32_t> bus_hz_;
  ExecutionContext context_;
  Hcs08FlashAgent agent_ = Hcs08FlashAgent::cpu;
};
} // namespace blofeld
