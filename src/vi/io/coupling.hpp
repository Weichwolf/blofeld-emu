// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "mcu/effects.hpp"
#include "mcu/epoch.hpp"
#include "mcu/input_history.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>

namespace blofeld::vi {
class Coupling {
public:
  static constexpr std::uint64_t default_window = 19200;

  struct Grant {
    enum class Kind : std::uint8_t { frontier, current };
    std::uint64_t until;
    Kind kind = Kind::frontier;
    bool operator==(const Grant &) const = default;
  };

  struct Stamp {
    std::uint64_t ticks = 0;
    bool fractional = false;

    [[nodiscard]] std::uint64_t ceil() const noexcept { return ticks + std::uint64_t{fractional}; }
  };

  struct Input {
    std::uint64_t ticks = 0, serial = 0;
    mcu::InputHistory::Values values{};
  };

  Coupling(PeerPort &peer,
           Shi &shi,
           DspReset &reset,
           ExecutionTime &time,
           MidiPort &midi,
           std::stop_token stop,
           std::uint64_t window);
  ~Coupling();
  Coupling(const Coupling &) = delete;
  Coupling &operator=(const Coupling &) = delete;

  [[nodiscard]] std::span<mcu::Effects::Entry> source_storage() noexcept { return storage_; }

  void bind() noexcept;
  void checkpoint(std::uint64_t ticks) noexcept;
  void cancel_checkpoint() noexcept;
  void publish(std::span<const mcu::Effects::Entry> entries, std::uint64_t frontier) noexcept;
  void request(std::uint64_t ticks) noexcept;
  [[nodiscard]] std::optional<Input> input(std::uint64_t seen) const noexcept;
  void acknowledge(std::uint64_t serial) noexcept;
  [[nodiscard]] bool pause_after(std::uint64_t ticks) const noexcept;
  [[nodiscard]] std::size_t consumed() const noexcept;
  [[nodiscard]] std::uint64_t cut() const noexcept;
  [[nodiscard]] std::uint64_t known() const noexcept;
  void replace(std::uint64_t origin) noexcept;

  [[nodiscard]] std::uint64_t window() const noexcept { return window_; }

  [[nodiscard]] const char *error() const noexcept {
    return error_.load(std::memory_order_acquire);
  }

  [[nodiscard]] bool stopped() const noexcept {
    return stop_.stop_requested() || error() != nullptr;
  }

  [[nodiscard]] std::uint64_t observation() const noexcept {
    return changed_.load(std::memory_order_seq_cst);
  }

  void wait(std::uint64_t observed) noexcept;
  void notify() noexcept;

  [[nodiscard]] std::optional<Grant> receiver(Stamp stamp);
  [[nodiscard]] bool service(Stamp stamp);
  [[nodiscard]] bool outputs(std::uint64_t ticks);

  [[nodiscard]] std::uint64_t revision() const noexcept { return serial_; }

  [[nodiscard]] bool held();

private:
  struct Cancel {
    Coupling *owner;
    void operator()() const noexcept;
  };

  [[nodiscard]] bool fail(const char *message) noexcept;
  [[nodiscard]] bool apply(const mcu::Effects::Entry &entry);
  [[nodiscard]] mcu::InputHistory::Values sample();
  [[nodiscard]] bool service_held();
  void refresh() noexcept;
  void attained(std::uint64_t exclusive) noexcept;
  [[nodiscard]] bool checkpoint_barrier();
  [[nodiscard]] Grant grant(Stamp stamp) const noexcept;

  PeerPort &peer_;
  Shi &shi_;
  DspReset &reset_;
  ExecutionTime &time_;
  MidiPort &midi_;
  const std::stop_token stop_;
  std::uint64_t window_;
  std::array<mcu::Effects::Entry, 4096> storage_{};
  Input input_;
  mcu::InputHistory::Values previous_{};
  std::size_t cursor_ = 0;
  std::uint64_t receiver_generation_ = 0, serial_ = 0;
  alignas(128) std::atomic<std::uint64_t> frontier_{0};
  // Preserve the existing cache-line separation explicitly.
  [[maybe_unused]] std::array<std::byte, 128 - sizeof(frontier_)> frontier_padding_;
  alignas(128) std::atomic<std::uint64_t> generation_{0};
  std::atomic<std::size_t> published_{0};
  std::atomic<bool> bound_{false};
  std::atomic<std::uint64_t> checkpoint_ticks_{0}, checkpoint_request_{0}, checkpoint_ready_{0};
  std::uint64_t checkpoint_sequence_ = 0;
  alignas(128) std::atomic<std::uint64_t> known_{0};
  [[maybe_unused]] std::array<std::byte, 128 - sizeof(known_)> known_padding_;
  alignas(128) std::atomic<std::uint64_t> cut_{0}, message_{0};
  std::atomic<std::size_t> consumed_{0};
  alignas(128) std::atomic<std::uint64_t> changed_{0};
  std::atomic<unsigned> waiting_{0};
  std::atomic<const char *> error_{nullptr};
  std::stop_callback<Cancel> cancellation_;
};
}
