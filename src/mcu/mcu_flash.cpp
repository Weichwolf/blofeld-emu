// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/mcu_flash.hpp"

#include "common/bus.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_flash_config.hpp"
#include "mcu/mcu_flash_timing_state.hpp"
#include "mcu/mcu_memory.hpp"
#include "storage/flash_image.hpp"

#include <cstdint>
#include <string>
#include <string_view>

namespace blofeld {
namespace {
constexpr unsigned base = 0x100;
constexpr unsigned end = 0x110;
constexpr unsigned configuration = 0x103;
constexpr unsigned status_register = 0x105;
constexpr unsigned command_register = 0x106;
constexpr unsigned buffer_empty = 0x80;
constexpr unsigned command_complete = 0x40;
constexpr unsigned violation = 0x20;
constexpr unsigned access_error = 0x10;
constexpr unsigned blank = 4;
constexpr unsigned command_mask = 0x65;
constexpr unsigned word_program = 0x20;
constexpr unsigned sector_erase = 0x40;
constexpr unsigned mass_erase = 0x41;
constexpr unsigned erase_verify = 5;
constexpr unsigned array_bytes = 32768;
constexpr unsigned sector_bytes = 512;
constexpr unsigned word_mask = 0xffff;
constexpr unsigned erased = 0xff;
}

FlashOptions McuFlash::options(const McuMemory &memory) {
  constexpr unsigned mapped_base = 0x8000;
  return {.protection = memory.read_flash(hc12::flash_protection_field - mapped_base),
          .security = memory.read_flash(hc12::flash_security_field - mapped_base)};
}

McuFlash::McuFlash(McuMemory &memory) : memory_(memory), config_(options(memory)) {}

McuFlash::McuFlash(McuMemory &memory, const McuFlash &source)
    : memory_(memory),
      config_(source.config_),
      buffered_(source.buffered_),
      active_(source.active_),
      buffer_state_(source.buffer_state_),
      control_(source.control_),
      command_(source.command_),
      errors_(source.errors_),
      blank_(source.blank_),
      key_words_(source.key_words_),
      key_locked_(source.key_locked_),
      clock_(source.clock_),
      stopped_(source.stopped_),
      command_known_(source.command_known_),
      timing_(source.timing_),
      store_(source.store_),
      store_writer_(source.store_writer_) {}

FlashRange McuFlash::affected(const Job job) {
  if (job.command == mass_erase) { return {.size = array_bytes}; }
  if (job.command == sector_erase) {
    return {.offset = static_cast<std::uint16_t>((job.offset / sector_bytes) * sector_bytes),
            .size = sector_bytes};
  }
  return {.offset = job.offset, .size = 2};
}

void McuFlash::reset() {
  require_storage();
  cancel_timing();
  if (active_ && active_->command != erase_verify) { memory_.invalidate_flash(affected(*active_)); }
  active_.reset();
  buffer_state_ = BufferState::empty;
  buffered_ = {};
  control_ = 0;
  command_ = 0;
  errors_ = 0;
  blank_ = false;
  key_words_ = 0;
  key_locked_ = false;
  clock_.reset();
  stopped_ = false;
  command_known_ = true;
  config_.reset(options(memory_));
}

void McuFlash::require_running() const {
  require_storage();
  if (timing_.faulted) {
    throw Fault("MCU flash clock failure requires coordinated reset or restore");
  }
  if (stopped_) { throw Fault("Unqualified MCU flash access while nominal STOP is held"); }
}

void McuFlash::observe_stop(const FlashStopTransition transition,
                            const FlashStopContract contract) {
  require_storage();
  if (contract != FlashStopContract::nominal_active_abort) {
    throw Fault("MCU flash STOP requires the explicit nominal active-abort contract");
  }
  if (transition == FlashStopTransition::resume) {
    if (!stopped_) { throw Fault("MCU flash STOP resume requires an observed entry"); }
    stopped_ = false;
    return;
  }
  if (transition != FlashStopTransition::enter || stopped_) {
    throw Fault("Invalid or duplicate MCU flash STOP entry");
  }
  if ((!active_ && buffer_state_ != BufferState::launched) || buffer_state_ == BufferState::word ||
      buffer_state_ == BufferState::command) {
    throw Fault("Unqualified MCU flash STOP without active command or with partial successor");
  }
  // S12FTS32KV1 section4.3: active data is lost; pending work must not execute.
  if (active_ && active_->command != erase_verify) { memory_.invalidate_flash(affected(*active_)); }
  active_.reset();
  buffer_state_ = BufferState::empty;
  buffered_ = {};
  cancel_timing();
  errors_ |= access_error;
  command_known_ = false; // No post-abort FCMD readback value is inferred.
  stopped_ = true;
}

std::string_view McuFlash::register_name(const std::uint16_t offset) {
  const auto name = McuFlashConfig::register_name(offset);
  if (!name.empty()) { return name; }
  switch (offset) {
    case configuration: return "FCNFG";
    case status_register: return "FSTAT";
    case command_register: return "FCMD";
    default: return offset >= base && offset < end ? "FTS32K normal-mode read-zero" : "";
  }
}

std::uint8_t McuFlash::status() const {
  const bool pending = buffer_state_ == BufferState::launched;
  return static_cast<std::uint8_t>((pending ? 0 : buffer_empty) |
                                   (!pending && !active_ ? command_complete : 0) | errors_ |
                                   (blank_ ? blank : 0));
}

std::uint8_t McuFlash::read(const std::uint16_t offset) const {
  require_running();
  if (offset == command_register && !command_known_) {
    throw Fault("Unknown MCU flash FCMD readback after STOP abort");
  }
  constexpr unsigned complete_keys = 4;
  if (offset == hc12::flash_security && (control_ & key_access) != 0 &&
      key_words_ == complete_keys && !key_locked_) {
    throw Fault("Unqualified MCU flash FSEC.SEC read after four key matches before KEYACC clear");
  }
  if (!McuFlashConfig::register_name(offset).empty()) { return config_.read(offset); }
  switch (offset) {
    case configuration: return control_;
    case status_register: return status();
    case command_register: return command_;
    default:
      if (offset >= base && offset < end) { return 0; }
      throw Fault("Unknown MCU flash register read at " + hex(offset));
  }
}

void McuFlash::reject_sequence(const std::uint8_t flag) {
  errors_ |= flag;
  if (buffer_state_ != BufferState::launched) {
    buffer_state_ = BufferState::empty;
    buffered_ = {};
  }
}

void McuFlash::stage(const MemoryWrite word) {
  require_running();
  if ((control_ & key_access) != 0) {
    write_key(word);
    return;
  }
  if (buffer_state_ == BufferState::launched) {
    throw Fault("Unqualified MCU flash write while a launched buffer is occupied");
  }
  if (errors_ != 0) { return; }
  if (!config_.oscillator_divisor() || word.size != 2 || (word.address & 1U) != 0 ||
      word.address >= array_bytes - 1 || buffer_state_ != BufferState::empty) {
    reject_sequence(access_error);
    return;
  }
  if (config_.protected_byte(word.address)) {
    reject_sequence(violation);
    return;
  }
  buffered_ = {.offset = word.address, .data = static_cast<std::uint16_t>(word.value & word_mask)};
  buffer_state_ = BufferState::word;
}

void McuFlash::write_command(const std::uint8_t value) {
  if (errors_ != 0) { return; }
  if (buffer_state_ != BufferState::word) {
    reject_sequence(access_error);
    return;
  }
  command_ = static_cast<std::uint8_t>(value & command_mask);
  command_known_ = true;
  if (command_ != word_program && command_ != sector_erase && command_ != mass_erase &&
      command_ != erase_verify) {
    reject_sequence(access_error);
    return;
  }
  if (command_ == mass_erase && !config_.whole_array_unprotected()) {
    reject_sequence(violation);
    return;
  }
  buffered_.command = command_;
  buffer_state_ = BufferState::command;
}

void McuFlash::write_status(const std::uint8_t value) {
  if (buffer_state_ == BufferState::empty && (value & buffer_empty) != 0) {
    throw Fault("Unqualified MCU flash idle CBEIF write without a staged command");
  }
  if (buffer_state_ == BufferState::launched && (value & buffer_empty) != 0) {
    throw Fault("Unqualified duplicate MCU flash launch while buffer is occupied");
  }
  errors_ = static_cast<std::uint8_t>(errors_ & ~(value & (access_error | violation)));
  if (buffer_state_ == BufferState::launched) { return; }
  if (buffer_state_ == BufferState::empty) { return; }
  if ((value & buffer_empty) == 0 || buffer_state_ != BufferState::command) {
    reject_sequence(access_error);
    return;
  }
  if (errors_ != 0) { return; }
  const auto next = plan_launch();
  blank_ = false;
  buffer_state_ = BufferState::launched;
  timing_ = next;
}

void McuFlash::write(const ByteWrite access) {
  require_running();
  if (register_name(access.address).empty()) {
    throw Fault("Unknown MCU flash register write at " + hex(access.address));
  }
  if (access.address == configuration && ((access.value | control_) & key_access) != 0) {
    write_key_configuration(access.value);
    return;
  }
  if ((control_ & key_access) != 0) {
    throw Fault("Unqualified MCU flash register write during key access");
  }
  if (active_ && access.address == hc12::flash_protection) {
    throw Fault("Unqualified MCU flash protection change during an executing command");
  }
  if (buffer_state_ == BufferState::launched && access.address != status_register) {
    throw Fault("Unqualified MCU flash register write while launched buffer is occupied");
  }
  const bool word_waiting = buffer_state_ == BufferState::word;
  const bool command_waiting = buffer_state_ == BufferState::command;
  if ((word_waiting && access.address != command_register && access.address != status_register) ||
      (command_waiting && access.address != status_register)) {
    reject_sequence(access_error);
    return;
  }
  if (access.address == status_register) {
    write_status(access.value);
    return;
  }
  if (access.address == command_register) {
    write_command(access.value);
    return;
  }
  if (access.address == configuration) {
    control_ = static_cast<std::uint8_t>(access.value & (buffer_empty | command_complete));
    return;
  }
  if (!McuFlashConfig::register_name(access.address).empty()) { config_.write(access); }
}

void McuFlash::transfer_state() {
  if (active_ || buffer_state_ != BufferState::launched) {
    throw Fault("MCU flash buffer-transfer milestone without an available executing slot");
  }
  if (timing_.inputs.contract == FlashTimingContract::unqualified && !clock_) {
    throw Fault("Unknown MCU flash operating clocks at command transfer");
  }
  if (timing_.inputs.contract == FlashTimingContract::unqualified) {
    constexpr unsigned minimum_nvm_hz = 150000;
    constexpr unsigned maximum_nvm_hz = 200000;
    const auto divisor = config_.oscillator_divisor();
    if (!divisor) { throw Fault("MCU flash transfer lost its configured divider"); }
    if (clock_->oscillator_hz < minimum_nvm_hz * *divisor ||
        clock_->oscillator_hz > maximum_nvm_hz * *divisor) {
      throw Fault("Unqualified MCU flash operating clock outside 150..200 kHz");
    }
  }
  if (buffered_.command == word_program) { memory_.check_flash_program(buffered_.offset); }
  active_ = buffered_;
  buffer_state_ = BufferState::empty;
  buffered_ = {};
}

void McuFlash::complete_state() {
  if (!active_) { throw Fault("MCU flash completion milestone without an executing command"); }
  const auto job = *active_;
  if (job.command == word_program) {
    memory_.program_flash({.address = job.offset, .size = 2, .value = job.data});
  } else if (job.command == erase_verify) {
    bool erased_array = true;
    for (unsigned offset = 0; offset < array_bytes; ++offset) {
      if (memory_.read_flash(static_cast<std::uint16_t>(offset)) != erased) {
        erased_array = false;
        break;
      }
    }
    blank_ = erased_array;
  } else {
    memory_.erase_flash(affected(job));
  }
  active_.reset();
}

void McuFlash::advance(const FlashMilestone milestone) {
  require_running();
  if (timing_.inputs.contract != FlashTimingContract::unqualified) {
    throw Fault("Manual milestones cannot advance a clock-owned MCU flash controller");
  }
  std::string target = "none";
  if (milestone == FlashMilestone::transfer_buffer) {
    if (buffer_state_ == BufferState::launched) { target = hex(buffered_.offset); }
  } else if (active_) {
    target = hex(active_->offset);
  }
  try {
    switch (milestone) {
      case FlashMilestone::transfer_buffer: transfer(); return;
      case FlashMilestone::complete_command: complete(); return;
      default: throw Fault("Invalid MCU flash reference milestone");
    }
  } catch (const Fault &fault) {
    throw Fault("Physical MCU flash operation target=" + target + ": " + fault.what());
  }
}

void McuFlash::check_array_read() const {
  require_running();
  if ((control_ & key_access) != 0) {
    throw Fault("Invalid MCU flash array read while KEYACC is set");
  }
  if (active_ || buffer_state_ == BufferState::launched) {
    throw Fault("Invalid MCU flash array read while a command is in progress");
  }
}

bool McuFlash::interrupt_pending() const {
  require_storage();
  if (stopped_) {
    // CCIF is set on abort. Only CBEIF is unspecified before STOP exit.
    if ((control_ & command_complete) != 0) { return true; }
    if ((control_ & buffer_empty) != 0) {
      throw Fault("Unknown MCU flash CBEIF interrupt while nominal STOP is held");
    }
    return false;
  }
  return (control_ & status() & (buffer_empty | command_complete)) != 0;
}

McuFlashImage McuFlash::power_off_image() const {
  require_storage();
  auto image = memory_.flash_image();
  if (active_ && active_->command != erase_verify) { image.invalidate(affected(*active_)); }
  return image;
}

void McuFlash::observe_clock(const FlashClockRates rates) {
  require_running();
  if (timing_.inputs.contract != FlashTimingContract::unqualified) {
    throw Fault("MCU flash clock rates belong to the configured native owner");
  }
  if (active_ || buffer_state_ == BufferState::launched) {
    throw Fault("Unqualified MCU flash clock change while a command is in progress");
  }
  constexpr unsigned minimum_oscillator_hz = 500000;
  constexpr unsigned maximum_oscillator_hz = 60000000;
  constexpr unsigned minimum_bus_hz = 1000000;
  if (rates.oscillator_hz < minimum_oscillator_hz || rates.oscillator_hz > maximum_oscillator_hz ||
      rates.bus_hz < minimum_bus_hz) {
    throw Fault("Unqualified MCU flash oscillator/bus programming clock limits");
  }
  clock_ = rates;
}

}
