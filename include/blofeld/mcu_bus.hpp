// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/mcu_clock.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_interrupts.hpp"
#include "blofeld/mcu_memory.hpp"
#include "blofeld/mcu_sci.hpp"
#include "blofeld/mcu_timer.hpp"
#include "blofeld/mcu_usb.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string_view>

namespace blofeld {

struct BusEvent {
  bool write = false;
  std::uint16_t address = 0;
  std::uint8_t value = 0;
  std::string_view name;
};

// Initial strict bring-up model. Unimplemented reads fail rather than returning
// fabricated ready flags. Register and pin coverage is expanded from traces.
class McuBus final : public Bus {
public:
  explicit McuBus(std::span<const std::uint8_t> flash);
  std::uint8_t read(std::uint16_t address) override;
  void write(std::uint16_t address, std::uint8_t value) override;
  void write_memory(MemoryWrite access) override;
  [[nodiscard]] std::uint8_t peek(std::uint16_t address) const override;

  [[nodiscard]] std::uint64_t revision() const override { return revision_; }

  void check_execute(InstructionFetch fetch) const override;
  void retire_instruction() override;
  void begin_instruction(ExecutionContext context) override;
  [[nodiscard]] std::optional<InterruptRequest> interrupt_request(std::uint8_t ccr) const override;
  void acknowledge_interrupt(std::uint16_t vector) override;
  void set_observer(std::function<void(const BusEvent &)> observer);
  void set_pin_drive(McuPort port, PinDrive drive);
  void observe_phy(PhyClockWindow window);
  void advance_oscillator(unsigned cycles);
  [[nodiscard]] unsigned oscillator_cycles_until_rti() const;
  void advance_sci(unsigned bus_cycles);
  void sample_timer_inputs(std::uint8_t mask);
  void advance_timer(unsigned bus_cycles);
  void reset();

private:
  [[nodiscard]] std::uint8_t read_register(std::uint16_t offset);
  [[nodiscard]] std::string_view write_register(ByteWrite write);
  [[nodiscard]] static bool reserved_register(std::uint16_t offset);
  void sync_peripheral_pins();
  void check_timer_outputs() const;
  std::uint64_t revision_ = 0;
  McuMemory memory_;
  McuGpio gpio_;
  McuClock clock_;
  McuSci sci_;
  McuTimer timer_;
  McuUsb usb_;
  McuInterrupts interrupts_;
  ExecutionContext context_{};
  std::function<void(const BusEvent &)> observer_;
};

} // namespace blofeld
