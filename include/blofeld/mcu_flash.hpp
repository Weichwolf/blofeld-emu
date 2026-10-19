// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/mcu_flash_config.hpp"
#include "blofeld/mcu_memory.hpp"

#include <cstdint>
#include <optional>
#include <string_view>

namespace blofeld {

// Explicit reference-state transitions, not timed clocks or observed silicon
// latencies. A future qualified scheduler must determine when these occur.
enum class FlashMilestone : std::uint8_t { transfer_buffer, complete_command };

struct FlashClockRates {
  std::uint32_t oscillator_hz = 0;
  std::uint32_t bus_hz = 0;
};

class McuFlash {
public:
  explicit McuFlash(McuMemory &memory);
  McuFlash(const McuFlash &) = delete;
  McuFlash &operator=(const McuFlash &) = delete;
  void reset();
  [[nodiscard]] static std::string_view register_name(std::uint16_t offset);
  [[nodiscard]] std::uint8_t read(std::uint16_t offset) const;
  void write(ByteWrite access);
  void stage(MemoryWrite physical_word);
  void advance(FlashMilestone milestone);
  void observe_clock(FlashClockRates rates);
  void check_array_read() const;
  [[nodiscard]] bool interrupt_pending() const;
  [[nodiscard]] McuFlashImage power_off_image() const;

private:
  friend class McuBus;
  McuFlash(McuMemory &memory, const McuFlash &source);
  enum class BufferState : std::uint8_t { empty, word, command, launched };

  struct Job {
    std::uint16_t offset = 0;
    std::uint16_t data = 0;
    std::uint8_t command = 0;
  };

  [[nodiscard]] static FlashOptions options(const McuMemory &memory);
  [[nodiscard]] static FlashRange affected(Job job);
  [[nodiscard]] std::uint8_t status() const;
  void reject_sequence(std::uint8_t flag);
  void write_command(std::uint8_t value);
  void write_status(std::uint8_t value);
  void transfer();
  void complete();
  McuMemory &memory_;
  McuFlashConfig config_;
  Job buffered_;
  std::optional<Job> active_;
  BufferState buffer_state_ = BufferState::empty;
  std::uint8_t control_ = 0;
  std::uint8_t command_ = 0;
  std::uint8_t errors_ = 0;
  bool blank_ = false;
  std::optional<FlashClockRates> clock_;
};

}
