// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/hcs08_bdc.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
enum class Hcs08BdcDrive : std::uint8_t { released, low, high, unknown };
enum class Hcs08BdcWireContract : std::uint8_t { unqualified, reference_command_delay };
enum class Hcs08BdcSync : std::uint8_t { idle, wait_release, wait_response, response };

struct Hcs08BdcWireClock {
  std::uint64_t period_ticks = 0;
  std::uint64_t first_edge_ticks = 0;
};

struct Hcs08BdcWireState {
  std::uint64_t cycles = 0, received_bytes = 0, transmitted_bytes = 0, syncs = 0, timeouts = 0;
  std::uint64_t time_ticks = 0, period_ticks = 0, next_clock_ticks = 0;
  // Falling edges and service deadlines are reference ticks; perceived and
  // SYNC-release instants are BDC clock ordinals.
  std::optional<std::uint64_t> last_fall, perceived, service_due, sync_release;
  Hcs08BdcDrive host = Hcs08BdcDrive::unknown, target = Hcs08BdcDrive::released;
  Hcs08BdcSync sync = Hcs08BdcSync::idle;
  std::uint8_t incoming = 0, outgoing = 0;
  unsigned receive_bits = 0, transmit_bits = 0;
  bool bit_pending = false, reply = false;
  bool operator==(const Hcs08BdcWireState &) const = default;
};

class Hcs08BdcWireCheckpoint {
public:
  Hcs08BdcWireCheckpoint(const Hcs08BdcWireCheckpoint &) = default;
  Hcs08BdcWireCheckpoint &operator=(const Hcs08BdcWireCheckpoint &) = default;

private:
  friend class Hcs08BdcWire;
  Hcs08BdcWireCheckpoint() = default;
  Hcs08BdcWireState state_;
  ExecutionContext context_;
};

// External owner supplies every actual BDC clock edge and all host drive changes.
// The command-service event follows the named reference delay hypothesis; this
// does not establish silicon bus arbitration or absolute clock frequencies.
class Hcs08BdcWire {
public:
  Hcs08BdcWire(Hcs08Bdc &bdc,
               Hcs08BdcWireContract contract,
               Hcs08BdcDrive initial_host,
               Hcs08BdcWireClock clock);
  Hcs08BdcWire(const Hcs08BdcWire &) = delete;
  Hcs08BdcWire &operator=(const Hcs08BdcWire &) = delete;
  void host_drive(Hcs08BdcDrive drive,
                  ExecutionContext context,
                  std::optional<std::uint64_t> time_ticks = std::nullopt);
  void clock();
  [[nodiscard]] bool level() const;
  [[nodiscard]] Hcs08BdcDrive target_drive() const;
  [[nodiscard]] Hcs08BdcWireState state() const;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] Hcs08BdcWireCheckpoint checkpoint() const;
  void restore_checkpoint(const Hcs08BdcWireCheckpoint &checkpoint);

private:
  void healthy() const;
  [[noreturn]] void fail(const std::string &reason) const;
  [[nodiscard]] bool resolved() const;
  void begin_bit();
  void receive_bit();
  void bit_clock();
  void finish_bit(std::uint64_t time_ticks);
  void sync_clock();
  void abort_communication();
  Hcs08Bdc &bdc_;
  Hcs08BdcWireState state_;
  ExecutionContext context_;
  bool healthy_ = true;
};
} // namespace blofeld
