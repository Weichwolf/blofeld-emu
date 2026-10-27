// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board_memory.hpp"

#include "board/gpio_pads.hpp"
#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_cop.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_icg.hpp"
#include "keyboard/hcs08_icg_events.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_tpm.hpp"

#include <cstdint>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace blofeld {
namespace {
constexpr unsigned address_space = 65536, normal_vector = 0xfffe;
constexpr unsigned power_on_status = 0x80, interrupt_mask = 8, spi_vector = 0xffe0;
constexpr unsigned adc_vector = 0xffd0, timer_vector = 0xffee;

bool ram_address(const unsigned address) {
  return address >= Hcs08BoardMemory::ram_first && address < Hcs08BoardMemory::ram_end;
}

bool pending_source(const std::uint16_t vector,
                    const Hcs08Tpm &timer,
                    const Hcs08Spi &spi,
                    const Hcs08Adc &adc) {
  switch (vector) {
    case timer_vector: return timer.interrupt_requested();
    case spi_vector: return spi.interrupt_requested();
    case adc_vector: return adc.interrupt_requested();
    default: return false;
  }
}
} // namespace

Hcs08BoardMemory::Hcs08BoardMemory(const Hcs08FlashImage &image,
                                   const Hcs08BoardMemoryContract contract)
    : flash_(image, Hcs08FlashContract::candidate_gt8a), contract_(contract) {
  if (contract != Hcs08BoardMemoryContract::candidate_gt8a_untimed &&
      contract != Hcs08BoardMemoryContract::candidate_gt8a_reference_bus) {
    fail("configure", ram_first, "requires explicit candidate GT8A memory/clock contract");
  }
  reset_peripherals(Hcs08IcgReset::power_on, power_on_status);
}

void Hcs08BoardMemory::reset_peripherals(const Hcs08IcgReset cause,
                                         const std::optional<std::uint8_t> reset_sources,
                                         const Hcs08FlashAbort retention) {
  if (resetting_) { fail("reset", normal_vector, "CPU vector transaction is incomplete"); }
  if (cause != Hcs08IcgReset::power_on && cause != Hcs08IcgReset::warm) {
    fail("reset", normal_vector, "reset cause is unqualified");
  }
  auto next_flash = flash_;
  next_flash.reset(retention);
  auto next_cop = cop_;
  next_cop.reset(reset_sources);
  if (generation_ == std::numeric_limits<std::uint64_t>::max() ||
      code_generation_ == std::numeric_limits<std::uint64_t>::max()) {
    fail("reset", normal_vector, "revision horizon exhausted");
  }
  ++generation_;
  ++code_generation_;
  flash_ = next_flash;
  flash_.context(context_);
  cop_ = next_cop;
  gpio_.reset();
  icg_.reset(cause);
  history_.reset();
  spi_.reset();
  adc_.reset();
  timer_.reset();
  if (cause == Hcs08IcgReset::power_on) { ram_.fill({.known = 0}); }
  serial_period_owned_ = false;
  observed_cpu_period_ = false;
  periods_ = {};
  healthy_ = true;
}

void Hcs08BoardMemory::attach_pad_source(const Hcs08GpioPort port, GpioPadReader reader) {
  healthy("attach pads", ram_first);
  gpio_.attach_pad_source(port, std::move(reader));
}

void Hcs08BoardMemory::healthy(const std::string &operation, const unsigned address) const {
  if (!healthy_) {
    fail(operation, address, "incomplete board access requires coordinated recovery");
  }
}

void Hcs08BoardMemory::data_access(const std::string &operation, const unsigned address) const {
  healthy(operation, address);
  if (resetting_) {
    fail(operation, address, "reset transaction permits only actual vector reads");
  }
}

void Hcs08BoardMemory::change(const unsigned address) {
  if (generation_ == std::numeric_limits<std::uint64_t>::max() ||
      (ram_address(address) && decoded_[address - ram_first] &&
       code_generation_ == std::numeric_limits<std::uint64_t>::max())) {
    fail("write", address, "revision horizon exhausted");
  }
  ++generation_;
  if (ram_address(address) && decoded_[address - ram_first]) { ++code_generation_; }
}

std::uint8_t Hcs08BoardMemory::read(const std::uint16_t address) {
  healthy("read", address);
  if (resetting_) {
    if (address < normal_vector) { fail("read", address, "outside the actual reset vector"); }
    return flash_.read_reset_vector_byte(address);
  }
  healthy_ = false;
  std::uint8_t result = 0;
  if (address >= hcs08_flash::first) {
    result = flash_.read_array(address);
  } else if (ram_address(address)) {
    const auto byte = ram_.at(address - ram_first);
    if (byte.known != Hcs08KnownByte::all) { fail("read", address, "unknown SRAM bits"); }
    result = byte.value;
  } else {
    result = read_peripheral(address);
  }
  healthy_ = true;
  return result;
}

std::uint8_t Hcs08BoardMemory::read_interrupt_vector_byte(const std::uint16_t address) {
  data_access("interrupt vector read", address);
  healthy_ = false;
  const auto value = flash_.read_interrupt_vector_byte(address);
  healthy_ = true;
  return value;
}

Hcs08KnownByte Hcs08BoardMemory::read_known_byte(const std::uint16_t address) {
  data_access("masked read", address);
  if (ram_address(address)) { return ram_.at(address - ram_first); }
  if (Hcs08Gpio::contains(address)) {
    healthy_ = false;
    const auto byte = gpio_.read_known_byte(address, context_);
    healthy_ = true;
    return byte;
  }
  if (address == hcs08_icg::status1 || address == hcs08_icg::status2) {
    healthy_ = false;
    const auto byte = history_.read_known(address);
    healthy_ = true;
    return byte;
  }
  return {.value = read(address)};
}

void Hcs08BoardMemory::write(const std::uint16_t address, const std::uint8_t value) {
  write_known_byte({.address = address, .value = value}, Hcs08KnownByte::all);
}

void Hcs08BoardMemory::write_known_byte(const ByteWrite access, const std::uint8_t known) {
  data_access("write", access.address);
  healthy_ = false;
  if (ram_address(access.address)) {
    change(access.address);
    ram_.at(access.address - ram_first) = {.value = static_cast<std::uint8_t>(access.value & known),
                                           .known = known};
  } else if (access.address >= hcs08_flash::first) {
    if (known != Hcs08KnownByte::all) { fail("write", access.address, "unknown FLASH data bits"); }
    change(access.address);
    flash_.stage(access);
  } else {
    change(access.address);
    write_peripheral(access, known);
  }
  healthy_ = true;
}

std::uint8_t Hcs08BoardMemory::peek(const std::uint16_t address) const {
  if (ram_address(address)) {
    const auto byte = ram_.at(address - ram_first);
    return byte.known == Hcs08KnownByte::all ? byte.value : 0;
  }
  if (address >= hcs08_flash::first) {
    const auto offset = address - hcs08_flash::first;
    return flash_.image().known[offset] ? flash_.image().data[offset] : 0;
  }
  return 0; // Decoder lookahead only. Actual execution checks every consumed byte.
}

void Hcs08BoardMemory::check_execute(const InstructionFetch fetch) const {
  data_access("fetch", fetch.address);
  if (fetch.length == 0 || fetch.length > address_space - fetch.address) {
    fail("fetch", fetch.address, "empty or wrapping instruction span");
  }
  for (unsigned index = 0; index < fetch.length; ++index) {
    const auto address = static_cast<std::uint16_t>(fetch.address + index);
    if (ram_address(address)) {
      if (ram_.at(address - ram_first).known != Hcs08KnownByte::all) {
        fail("fetch", address, "unknown SRAM opcode bits");
      }
    } else if (address >= hcs08_flash::first) {
      static_cast<void>(flash_.read_array(address));
    } else {
      fail("fetch", address, "not implemented executable memory");
    }
  }
}

std::uint64_t Hcs08BoardMemory::revision() const {
  return generation_;
}

std::uint64_t Hcs08BoardMemory::code_revision() const {
  return code_generation_;
}

void Hcs08BoardMemory::note_code_byte(const std::uint16_t address) const {
  if (ram_address(address)) { decoded_.set(address - ram_first); }
}

void Hcs08BoardMemory::begin_instruction(const ExecutionContext context) {
  data_access("begin instruction", context.pc);
  healthy_ = false;
  set_context(context);
  if (!observed_cpu_period_) { history_.unobserved_interval(); }
  if (history_.action(true) == Hcs08IcgFaultAction::reset || cop_.state().reset_requested) {
    fail(
        "begin instruction", context.pc, "pending reset requires coordinated CPU/peripheral reset");
  }
  healthy_ = true;
}

void Hcs08BoardMemory::set_context(const ExecutionContext context) {
  context_ = context;
  flash_.context(context);
  icg_.context(context);
  history_.context(context);
  spi_.context(context);
  adc_.context(context);
  timer_.context(context);
  cop_.context(context);
}

void Hcs08BoardMemory::begin_reset(const ExecutionContext context) {
  data_access("begin reset", normal_vector);
  context_ = context;
  resetting_ = true;
}

void Hcs08BoardMemory::end_reset() {
  healthy("end reset", normal_vector);
  if (!resetting_) { fail("end reset", normal_vector, "no active vector transaction"); }
  resetting_ = false;
}

std::optional<InterruptRequest> Hcs08BoardMemory::interrupt_request(const std::uint8_t ccr) const {
  fail("IRQ query", hcs08_icg::status1, "actual CPU mask knowledge required ccr=" + hex(ccr));
}

std::optional<InterruptRequest> Hcs08BoardMemory::interrupt_request(const Hcs08State &state) const
    try {
  data_access("IRQ query", hcs08_icg::status1);
  const bool masked = (state.ccr_known & interrupt_mask) != 0 && (state.ccr & interrupt_mask) != 0;
  auto sample = history_;
  sample.context({.pc = state.pc, .ccr = state.ccr});
  if (sample.action(masked) != Hcs08IcgFaultAction::none || cop_.state().reset_requested) {
    throw Fault("HCS08 board IRQ/reset query address=" + hex(hcs08_icg::status1) +
                " pc=" + hex(state.pc) + " ccr=" + hex(state.ccr) +
                " ccr-known=" + hex(state.ccr_known) + " agent=CPU: coordinated service required");
  }
  if (masked) { return {}; }
  // GT16A/GT8A Table5-1:TPM1 overflow(priority8),SPI(15),ATD(23).
  for (const auto vector : {timer_vector, spi_vector, adc_vector}) {
    if (pending_source(static_cast<std::uint16_t>(vector), timer_, spi_, adc_)) {
      return InterruptRequest{.vector = static_cast<std::uint16_t>(vector),
                              .kind = InterruptKind::maskable};
    }
  }
  return {};
} catch (const Fault &error) {

  throw Fault("HCS08 board IRQ query address=" + hex(hcs08_icg::status1) + " pc=" +
              (state.pc_known ? hex(state.pc) : "unknown pc-known=0") + " ccr=" + hex(state.ccr) +
              " ccr-known=" + hex(state.ccr_known) + " agent=CPU; source detail: " + error.what());
}

void Hcs08BoardMemory::begin_interrupt(const InterruptRequest request, const bool reuse_stack) {
  data_access("IRQ begin", request.vector);
  if (reuse_stack || request.kind != InterruptKind::maskable ||
      !pending_source(request.vector, timer_, spi_, adc_)) {
    fail("IRQ begin", request.vector, "unsupported source, frame reuse or vector");
  }
}

void Hcs08BoardMemory::acknowledge_interrupt(const std::uint16_t vector) {
  data_access("IRQ acknowledge", vector);
  if (!pending_source(vector, timer_, spi_, adc_)) {
    fail("IRQ acknowledge", vector, "source is absent or unqualified");
  }
  // Native vector acceptance does not clear TOF,SPRF/SPTEF or COCO.
  // Only the corresponding real peripheral read/write sequence acknowledges it.
}

bool Hcs08BoardMemory::faulted() const {
  return !healthy_;
}

const Hcs08FlashImage &Hcs08BoardMemory::flash_image() const {
  return flash_.image();
}

Hcs08IcgState Hcs08BoardMemory::icg_state() const {
  return icg_.state();
}

Hcs08AdcState Hcs08BoardMemory::adc_state() const {
  return adc_.state();
}

Hcs08TpmState Hcs08BoardMemory::timer_state() const {
  return timer_.state();
}

Hcs08CopState Hcs08BoardMemory::cop_state() const {
  return cop_.state();
}

Hcs08SpiState Hcs08BoardMemory::spi_state() const {
  return spi_.state();
}

Hcs08FlashState Hcs08BoardMemory::flash_state() const {
  return flash_.state();
}

[[noreturn]] void Hcs08BoardMemory::fail(const std::string &operation,
                                         const unsigned address,
                                         const std::string &reason) const {
  throw Fault("HCS08 board " + operation + " address=" + hex(address) + " pc=" + hex(context_.pc) +
              " ccr=" + hex(context_.ccr) + " agent=CPU: " + reason);
}
} // namespace blofeld
