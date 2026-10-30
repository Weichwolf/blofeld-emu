// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "common/masked_bus.hpp"
#include "mcu/mcu_clock.hpp"
#include "mcu/mcu_cop.hpp"
#include "mcu/mcu_flash.hpp"
#include "mcu/mcu_flash_timing_state.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_interrupts.hpp"
#include "mcu/mcu_memory.hpp"
#include "mcu/mcu_sci.hpp"
#include "mcu/mcu_timer.hpp"
#include "mcu/mcu_usb.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <bitset>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string_view>

namespace blofeld {
class VirtualDeviceState;

struct BusEvent {
  bool write = false;
  std::uint16_t address = 0;
  std::uint8_t value = 0;
  std::string_view name;
};

struct McuWaitClockGates {
  bool sci = true;
  bool timer = true;
  bool cop = true;
};

// Initial strict bring-up model. Unimplemented reads fail rather than returning
// fabricated ready flags. Register and pin coverage is expanded from traces.
class McuBus final : public MaskedByteBus {
public:
  explicit McuBus(std::span<const std::uint8_t> flash);
  explicit McuBus(const McuFlashImage &image);
  // Deep modeled-state copy. External observer callbacks are not copied.
  [[nodiscard]] std::unique_ptr<McuBus> clone() const;
  [[nodiscard]] McuFlashImage power_off_flash_image() const;
  std::uint8_t read(std::uint16_t address) override;
  [[nodiscard]] std::uint8_t read_masked_byte(MaskedByteRead access) override;
  void write(std::uint16_t address, std::uint8_t value) override;
  void write_memory(MemoryWrite access) override;
  [[nodiscard]] std::uint8_t peek(std::uint16_t address) const override;

  [[nodiscard]] std::uint64_t revision() const override { return revision_; }

  [[nodiscard]] std::uint64_t code_revision() const override { return code_revision_; }

  void note_code_byte(std::uint16_t address) const override;

  void check_execute(InstructionFetch fetch) const override;
  void retire_instruction() override;
  void begin_instruction(ExecutionContext context) override;
  void begin_reset(ExecutionContext context) override;
  [[nodiscard]] std::optional<InterruptRequest> interrupt_request(std::uint8_t ccr) const override;
  void acknowledge_interrupt(std::uint16_t vector) override;
  void set_observer(std::function<void(const BusEvent &)> observer);
  void set_pin_drive(McuPort port, PinDrive drive);
  void update_pin_drive(McuPort port, std::uint8_t affected, PinDrive drive);
  [[nodiscard]] PinSample observe_pads(McuPort port, std::uint8_t mask) const;
  [[nodiscard]] PinOutputs gpio_outputs(McuPort port) const;
  [[nodiscard]] PinBiases gpio_biases(McuPort port) const;
  void attach_gpio_pad_source(McuPort port, GpioPadReader reader);
  // Observe resolved PE0/PE1 after board pad changes, including while masked.
  // Selection remains pure; the board must deliver every edge it claims to model.
  void observe_interrupt_pads();
  void observe_phy(PhyClockWindow window);
  [[nodiscard]] McuRunClockRoute run_clock_route() const;
  // Qualified system clocks continue through WAI; unsupported CRG gates fail.
  [[nodiscard]] McuWaitClockGates wait_clock_gates() const;
  void observe_usb_decoder(UsbDecoderState state);
  void observe_usb_sof(std::uint16_t frame, bool crc_valid = true);
  void observe_usb_configuration(UsbConfigurationRequest request);
  UsbResponse receive_usb_setup(const std::array<std::uint8_t, 8> &data, bool crc_valid = true);
  UsbResponse receive_usb_out(UsbOutPacket packet);
  [[nodiscard]] UsbInPacket request_usb_in(UsbBuffer buffer);
  void finish_usb_in(bool acknowledged);
  [[nodiscard]] UsbEndpointInfo usb_host_endpoint(std::uint8_t address) const;
  [[nodiscard]] UsbConfigurationInfo usb_host_configuration() const;

  [[nodiscard]] bool usb_host_suspended() const { return usb_.host_suspended(); }

  [[nodiscard]] UsbHostEvidence usb_host_evidence() const { return usb_.host_evidence(); }

  void advance_oscillator(unsigned cycles, bool cop_running = true);

  [[nodiscard]] McuCopState cop_state() const { return clock_.cop_state(); }

  [[nodiscard]] unsigned oscillator_cycles_until_cop() const {
    return clock_.oscillator_cycles_until_cop();
  }

  [[nodiscard]] unsigned oscillator_cycles_until_rti() const;
  void advance_sci(unsigned bus_cycles);
  void sample_timer_inputs(std::uint8_t mask);
  void advance_timer(unsigned bus_cycles);
  void reset();
  void power_cycle();
  void observe_flash_clock(FlashClockRates rates);
  void configure_flash_timing(FlashTimingInputs inputs);
  void attach_flash_store(std::shared_ptr<VirtualDeviceState> store);
  void advance_flash_time(FlashClockBoundary boundary);
  [[nodiscard]] std::optional<std::uint64_t> next_flash_event_ticks() const;
  void advance_flash(FlashMilestone milestone);
  void observe_flash_stop(FlashStopTransition transition,
                          FlashStopContract contract = FlashStopContract::unqualified);

private:
  friend class McuMachine;
  void notify(const BusEvent &event);
  void check_cop_reset() const;
  void reset_modules();

  struct CloneState {};

  struct PeripheralPinState {
    std::uint8_t routing = 0;
    PinFunction timer;
    bool sci_receive = false;
    std::optional<bool> sci_transmit;

    bool operator==(const PeripheralPinState &) const = default;
  };

  McuBus(const McuBus &source, CloneState /*tag*/);
  [[nodiscard]] std::uint8_t read_register(std::uint16_t offset);
  [[nodiscard]] std::string_view write_register(ByteWrite write);
  [[nodiscard]] static bool reserved_register(std::uint16_t offset);
  void sync_peripheral_pins();
  void check_timer_outputs() const;
  void invalidate_code(MemoryRead access);
  void stage_flash(MemoryWrite access);
  std::uint64_t revision_ = 0;
  std::uint64_t code_revision_ = 0;
  mutable std::bitset<65536> decoded_bytes_;
  McuMemory memory_;
  McuFlash flash_;
  McuGpio gpio_;
  McuClock clock_;
  McuSci sci_;
  McuTimer timer_;
  // Derived routing only; reset and cloned buses rebuild it on the next sync.
  std::optional<PeripheralPinState> peripheral_pins_;
  McuUsb usb_;
  McuInterrupts interrupts_;
  ExecutionContext context_{};
  bool reset_asserted_ = false;
  bool notifying_ = false;
  std::function<void(const BusEvent &)> observer_;
};

} // namespace blofeld
