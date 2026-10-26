// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board_memory.hpp"
#include "common/bus.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_cop.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_icg.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_system_options.hpp"
#include "keyboard/hcs08_tpm.hpp"

#include <cstdint>
#include <limits>

namespace blofeld {
namespace {
bool timer_register(const unsigned address) {
  constexpr unsigned modulus_low = 0x34;
  return address >= Hcs08Tpm::status && address <= modulus_low;
}

bool icg_register(const unsigned address) {
  return address >= hcs08_icg::control1 && address <= hcs08_icg::trim;
}

bool clock_status(const unsigned address) {
  return address == hcs08_icg::status1 || address == hcs08_icg::status2;
}

bool cop_register(const unsigned address) {
  return address == Hcs08Cop::status || address == Hcs08SystemOptions::address;
}

bool flash_register(const unsigned address) {
  return address >= hcs08_flash::divider && address <= hcs08_flash::command;
}
} // namespace

std::uint8_t Hcs08BoardMemory::read_peripheral(const std::uint16_t address) {
  if (clock_status(address)) { return history_.read(address); }
  if (icg_register(address)) { return icg_.read(address); }
  if (Hcs08Gpio::contains(address)) { return gpio_.read(address, context_); }
  if (Hcs08Adc::contains(address)) {
    if (contract_ == Hcs08BoardMemoryContract::candidate_gt8a_untimed && adc_.state().active &&
        address >= hcs08_adc::status && address <= hcs08_adc::low) {
      fail("read", address, "live ADC requires elapsed periods and reference/input observations");
    }
    return adc_.read(address);
  }
  if (Hcs08Spi::contains(address)) {
    const auto spi = spi_.state();
    if (!serial_period_owned_ && (spi.active || spi.transmit) &&
        (address == hcs08_spi::status || address == hcs08_spi::data)) {
      fail("read", address, "live SPI requires qualified transfer and serial clock ownership");
    }
    return spi_.read(address);
  }
  if (cop_register(address)) { return cop_.read(address); }
  if (timer_register(address)) {
    constexpr unsigned bus_clock = 8;
    if (contract_ == Hcs08BoardMemoryContract::candidate_gt8a_untimed &&
        (timer_.state().control & bus_clock) != 0) {
      fail("read", address, "live TPM requires elapsed bus periods and prescaler phase");
    }
    return timer_.read(address);
  }
  if (flash_register(address)) {
    const auto pending = flash_.state();
    if (address == hcs08_flash::status &&
        (pending.active || pending.buffer == Hcs08FlashBuffer::launched)) {
      fail("read", address, "live FLASH requires a qualified command clock/completion owner");
    }
    return flash_.read(address);
  }
  fail("read", address, "unimplemented memory or peripheral");
}

void Hcs08BoardMemory::write_peripheral(const ByteWrite access, const std::uint8_t known) {
  if (clock_status(access.address)) {
    history_.write(access, known);
    return;
  }
  if (access.address == Hcs08Cop::status) {
    if (periods_.cop_services == std::numeric_limits<std::uint64_t>::max()) {
      fail("write", access.address, "COP diagnostic counter exhausted");
    }
    cop_.write(access); // Every SRS data bit is ignored, including unknown payload bits.
    ++periods_.cop_services;
    periods_.service_known = known;
    return;
  }
  if (known != Hcs08KnownByte::all) {
    fail("write", access.address, "unknown peripheral operand bits");
  }
  if (access.address == hcs08_icg::control2) {
    auto next = history_;
    next.control2(access.value);
    icg_.write(access);
    history_ = next;
  } else if (icg_register(access.address)) {
    icg_.write(access);
  } else if (Hcs08Gpio::contains(access.address)) {
    gpio_.write(access, context_);
  } else if (Hcs08Adc::contains(access.address)) {
    adc_.write(access);
  } else if (Hcs08Spi::contains(access.address)) {
    spi_.write(access);
  } else if (cop_register(access.address)) {
    cop_.write(access);
  } else if (timer_register(access.address)) {
    timer_.write(access);
  } else if (flash_register(access.address)) {
    flash_.write(access);
  } else {
    fail("write", access.address, "unimplemented memory or peripheral");
  }
}
} // namespace blofeld
