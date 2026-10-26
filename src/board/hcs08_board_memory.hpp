// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_cop.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_icg.hpp"
#include "keyboard/hcs08_icg_history.hpp"
#include "keyboard/hcs08_known_interrupts.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_tpm.hpp"

#include <array>
#include <bitset>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
enum class Hcs08BoardMemoryContract : std::uint8_t {
  unqualified,
  candidate_gt8a_untimed,
  candidate_gt8a_reference_bus
};

// Caller-observed fault events over one entire supplied BUSCLK interval.
// no_fault does not establish LOCK,CLKST,frequency or other live status bits.
enum class Hcs08BoardClockObservation : std::uint8_t {
  unobserved,
  no_fault,
  loss_of_lock,
  loss_of_clock,
  loss_of_clock_and_lock
};

struct Hcs08BoardPeriods {
  std::uint64_t attempted = 0, completed = 0, cop_services = 0;
  std::uint8_t service_known = Hcs08KnownByte::all;
  bool operator==(const Hcs08BoardPeriods &) const = default;
};

// Memory/peripheral ownership for the candidate chip, independent of CPU and
// host transports. Reference mode consumes explicit caller-supplied BUSCLK
// periods; no oscillator,pin sample,factory calibration or NVM completion is supplied.
class Hcs08BoardMemory final : public Bus,
                               public Hcs08KnownMemory,
                               public Hcs08KnownInterrupts,
                               public Hcs08BdcMemory {
public:
  Hcs08BoardMemory(const Hcs08FlashImage &image, Hcs08BoardMemoryContract contract);
  void reset_peripherals(Hcs08IcgReset cause,
                         std::optional<std::uint8_t> reset_sources,
                         Hcs08FlashAbort retention = Hcs08FlashAbort::unqualified_retention);
  void attach_pad_source(Hcs08GpioPort port, GpioPadReader reader);
  // Supplied reference periods order COP,TPM,ADC before native CPU retirement.
  // No oscillator or physical MMIO/arbitration contract is established.
  void advance_reference_bus(
      ExecutionContext context,
      Hcs08BoardClockObservation observation = Hcs08BoardClockObservation::unobserved);
  void observe_adc_reference(Hcs08AdcReference reference);
  void observe_adc_input(unsigned channel, std::optional<std::uint32_t> microvolts);
  void observe_timer_phase(unsigned phase);
  [[nodiscard]] Hcs08BoardPeriods periods() const;
  [[nodiscard]] Hcs08KnownByte inspect_ram(std::uint16_t address) const;
  [[nodiscard]] bool secured(ExecutionContext context) const override;
  [[nodiscard]] std::uint8_t read(std::uint16_t address, ExecutionContext context) override;
  void write(ByteWrite access, ExecutionContext context) override;
  [[nodiscard]] std::uint8_t read(std::uint16_t address) override;
  void write(std::uint16_t address, std::uint8_t value) override;
  [[nodiscard]] Hcs08KnownByte read_known_byte(std::uint16_t address) override;
  void write_known_byte(ByteWrite access, std::uint8_t known) override;
  [[nodiscard]] std::uint8_t peek(std::uint16_t address) const override;
  [[nodiscard]] std::uint64_t revision() const override;
  [[nodiscard]] std::uint64_t code_revision() const override;
  void note_code_byte(std::uint16_t address) const override;
  void check_execute(InstructionFetch fetch) const override;
  void begin_instruction(ExecutionContext context) override;
  void begin_reset(ExecutionContext context) override;
  void end_reset() override;
  [[nodiscard]] std::optional<InterruptRequest> interrupt_request(std::uint8_t ccr) const override;
  [[nodiscard]] std::optional<InterruptRequest>
  interrupt_request(const Hcs08State &state) const override;
  void begin_interrupt(InterruptRequest request, bool reuse_stack) override;
  void acknowledge_interrupt(std::uint16_t vector) override;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] const Hcs08FlashImage &flash_image() const;
  [[nodiscard]] Hcs08IcgState icg_state() const;
  [[nodiscard]] Hcs08AdcState adc_state() const;
  [[nodiscard]] Hcs08TpmState timer_state() const;
  [[nodiscard]] Hcs08CopState cop_state() const;
  [[nodiscard]] Hcs08SpiState spi_state() const;
  [[nodiscard]] Hcs08FlashState flash_state() const;

  static constexpr unsigned ram_first = 0x80, ram_end = 0x480, ram_bytes = ram_end - ram_first;

private:
  friend class Hcs08BoardSerial;
  friend class Hcs08Board;
  bool serial_period_owned_ = false, observed_cpu_period_ = false;
  void healthy(const std::string &operation, unsigned address) const;
  void set_context(ExecutionContext context);
  void observe_clock_interval(Hcs08BoardClockObservation observation);
  void data_access(const std::string &operation, unsigned address) const;
  void change(unsigned address);
  [[nodiscard]] std::uint8_t read_peripheral(std::uint16_t address);
  void write_peripheral(ByteWrite access, std::uint8_t known);
  [[noreturn]] void
  fail(const std::string &operation, unsigned address, const std::string &reason) const;
  Hcs08Flash flash_;
  Hcs08Gpio gpio_{Hcs08GpioContract::candidate_gt8a};
  Hcs08Icg icg_{Hcs08IcgContract::candidate_gt8a_no_clock_events, Hcs08IcgReset::power_on};
  Hcs08IcgHistory history_{Hcs08IcgHistoryContract::candidate_gt8a_unknown_events};
  Hcs08Spi spi_{Hcs08SpiContract::candidate_gt8a_reference_master_edges};
  Hcs08Adc adc_{Hcs08AdcContract::candidate_gt8a_reference_stable_input};
  Hcs08Tpm timer_{Hcs08TpmContract::candidate_gt8a_run_bus_edges};
  Hcs08Cop cop_{Hcs08CopContract::candidate_gt8a_run_bus_edges};
  std::array<Hcs08KnownByte, ram_bytes> ram_;
  mutable std::bitset<ram_bytes> decoded_;
  ExecutionContext context_;
  Hcs08BoardMemoryContract contract_;
  Hcs08BoardPeriods periods_;
  std::uint64_t generation_ = 1, code_generation_ = 1;
  bool healthy_ = true, resetting_ = false;
};
} // namespace blofeld
