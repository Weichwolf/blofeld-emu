// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board_memory.hpp"
#include "common/bus.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_cop.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_icg.hpp"
#include "keyboard/hcs08_icg_events.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_tpm.hpp"

#include <cstdint>
#include <limits>
#include <optional>

namespace blofeld {
void Hcs08BoardMemory::advance_reference_bus(const ExecutionContext context,
                                             const Hcs08BoardClockObservation observation) {
  data_access("reference bus period", hcs08_icg::status1);
  healthy_ = false;
  set_context(context);
  if (contract_ != Hcs08BoardMemoryContract::candidate_gt8a_reference_bus) {
    fail("clock",
         hcs08_icg::status1,
         "reference BUSCLK input requires its explicit memory contract");
  }
  if (periods_.attempted == std::numeric_limits<std::uint64_t>::max()) {
    fail("clock", hcs08_icg::status1, "reference period horizon exhausted");
  }
  ++periods_.attempted;
  observe_clock_interval(observation);
  if (history_.action(true) == Hcs08IcgFaultAction::reset) {
    fail("clock", hcs08_icg::status1, "pending ICG reset requires coordinated board handling");
  }
  if (cop_.advance_bus(1) != 1 || cop_.state().reset_requested) {
    fail("clock", Hcs08Cop::status, "pending COP reset requires coordinated board handling");
  }
  timer_.advance_bus(1);
  adc_.advance_bus(1);
  const auto spi = spi_.state();
  if ((spi.active || spi.transmit) && !serial_period_owned_) {
    fail(
        "clock", hcs08_spi::status, "SPI promotion/serial clock and pad ownership are unqualified");
  }
  const auto flash = flash_.state();
  if (flash.active || flash.buffer == Hcs08FlashBuffer::launched) {
    fail("clock", hcs08_flash::status, "FLASH transfer/completion clock ownership is unqualified");
  }
  ++periods_.completed;
  healthy_ = true;
}

void Hcs08BoardMemory::observe_clock_interval(const Hcs08BoardClockObservation observation) {
  if (observation == Hcs08BoardClockObservation::unobserved) {
    history_.unobserved_interval();
    return;
  }
  // An observed interval still invalidates unobserved instantaneous status.
  history_.observe({});
  switch (observation) {
    case Hcs08BoardClockObservation::no_fault: return;
    case Hcs08BoardClockObservation::loss_of_lock:
      history_.signal(Hcs08IcgFault::loss_of_lock);
      return;
    case Hcs08BoardClockObservation::loss_of_clock:
      history_.signal(Hcs08IcgFault::loss_of_clock);
      return;
    case Hcs08BoardClockObservation::loss_of_clock_and_lock:
      history_.signal(Hcs08IcgFault::loss_of_clock_and_lock);
      return;
    case Hcs08BoardClockObservation::unobserved:
    default: fail("clock observation", hcs08_icg::status1, "unknown interval observation");
  }
}

void Hcs08BoardMemory::observe_adc_reference(const Hcs08AdcReference reference) {
  data_access("observe ADC reference", hcs08_adc::control);
  healthy_ = false;
  adc_.observe_reference(reference);
  healthy_ = true;
}

void Hcs08BoardMemory::observe_adc_input(const unsigned channel,
                                         const std::optional<std::uint32_t> microvolts) {
  data_access("observe ADC input", hcs08_adc::pins);
  healthy_ = false;
  adc_.observe_input(channel, microvolts);
  healthy_ = true;
}

void Hcs08BoardMemory::observe_timer_phase(const unsigned phase) {
  data_access("observe timer phase", Hcs08Tpm::status);
  healthy_ = false;
  timer_.observe_bus_phase(phase);
  healthy_ = true;
}

Hcs08BoardPeriods Hcs08BoardMemory::periods() const {
  return periods_;
}

Hcs08KnownByte Hcs08BoardMemory::inspect_ram(const std::uint16_t address) const {
  if (address < ram_first || address >= ram_end) {
    fail("inspect RAM", address, "outside the physical candidate SRAM");
  }
  return ram_.at(address - ram_first);
}
} // namespace blofeld
