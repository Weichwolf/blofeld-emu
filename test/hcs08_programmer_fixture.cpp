// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_programmer_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_flash.hpp"
#include "blofeld/hcs08_icg.hpp"
#include "blofeld/hcs08_system_options.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_flash_fixture.hpp"

#include <cstdint>
#include <fstream>
#include <ios>
#include <ostream>
#include <string>
#include <vector>

namespace blofeld::test::hcs08_programmer {
namespace {
constexpr auto contract = Hcs08FlashContract::candidate_gt8a;
constexpr unsigned transfer_delay = 7;
constexpr unsigned option_mask = 0x80;

bool flash_register(const unsigned address) {
  return address >= hf::divider && address <= hf::command;
}

bool icg_register(const unsigned address) {
  return address >= hcs08_icg::control1 && address <= hcs08_icg::trim;
}
} // namespace

Memory::Memory(const Hcs08IdleLaunch idle, const unsigned delay)
    : flash(hcs08_nvm::initial(), contract, idle), options(contract), completion_delay(delay) {
  for (unsigned n = ram_first; n < ram_end; ++n) {
    writable.set(n);
    if (n < algorithm) {
      known.set(n); // Explicit pre-entry HC12 clear of0080..009f only.
      bytes[n] = 0;
    }
  }
  flash.observe_bus_clock(reference_hz);
}

std::uint8_t Memory::read(const std::uint16_t address) {
  if (resetting) {
    const auto value = flash.read_reset_vector_byte(address);
    accesses.push_back({.address = address, .value = value, .write = false});
    return value;
  }
  if (!flash_register(address) && !icg_register(address) && address < hf::first &&
      address != Hcs08SystemOptions::address) {
    return hcs08::Memory::read(address);
  }
  std::uint8_t result = 0;
  if (icg_register(address)) {
    result = icg.read(address);
  } else if (address == Hcs08SystemOptions::address) {
    result = options.read();
  } else if (address >= hf::first) {
    result = flash.read_array(address);
  } else {
    result = flash.read(address);
  }
  accesses.push_back({.address = address, .value = result, .write = false});
  return result;
}

void Memory::write(const std::uint16_t address, const std::uint8_t value) {
  if (resetting) {
    throw Fault("HCS08 write during reset address=" + hex(address) + " pc=" + hex(context.pc));
  }
  if (icg_register(address)) {
    icg.write({.address = address, .value = value});
  } else if (address >= hf::first) {
    flash.stage({.address = address, .value = value});
    ++array_writes;
  } else if (flash_register(address)) {
    flash.write({.address = address, .value = value});
    command_writes += static_cast<unsigned>(address == hf::command);
    status_writes += static_cast<unsigned>(address == hf::status);
  } else if (address == Hcs08SystemOptions::address) {
    options.write({.address = address, .value = value}, context);
    ++sopt_writes;
  } else {
    hcs08::require(address >= ram_first && address <= marker + 2,
                   "Unexpected original HCS08 programmer RAM write at " + hex(address) +
                       " pc=" + hex(context.pc));
    hcs08::Memory::write(address, value);
    return;
  }
  accesses.push_back({.address = address, .value = value, .write = true});
}

void Memory::begin_instruction(const ExecutionContext value) {
  if (resetting) { throw Fault("HCS08 instruction during incomplete reset pc=" + hex(value.pc)); }
  hcs08::Memory::begin_instruction(value);
  flash.context(value);
  icg.context(value);
}

void Memory::reset_peripherals(const Hcs08FlashAbort retention) {
  if (resetting) {
    throw Fault("HCS08 peripheral reset address=" + hex(Hcs08SystemOptions::address) +
                " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr) +
                " agent=CPU: during CPU vector transaction");
  }
  flash.reset(retention);
  options.reset();
  icg.reset(Hcs08IcgReset::warm);
  transfer_due.reset();
  complete_due.reset();
}

void Memory::begin_reset(const ExecutionContext value) {
  begin_instruction(value);
  resetting = true;
}

void Memory::end_reset() {
  hcs08::require(resetting, "HCS08 reset completion without a matching reset boundary");
  resetting = false;
}

void Memory::check_execute(const InstructionFetch fetch) const {
  if (resetting) {
    throw Fault("HCS08 fetch during reset address=" + hex(fetch.address) +
                " pc=" + hex(context.pc));
  }
  hcs08::require(fetch.address >= algorithm && fetch.address < algorithm + algorithm_bytes &&
                     fetch.length <= algorithm + algorithm_bytes - fetch.address,
                 "Original page programmer executes only its pinned132-byte body");
  hcs08::Memory::check_execute(fetch);
}

void Memory::retire_instruction() {
  if (resetting) { throw Fault("HCS08 instruction retirement during reset pc=" + hex(context.pc)); }
  hcs08::Memory::retire_instruction();
  if (complete_due && retired >= *complete_due) {
    flash.advance(Hcs08FlashMilestone::complete_command);
    complete_due.reset();
    ++completions;
  }
  const auto state = flash.state();
  if (!state.active && state.buffer == Hcs08FlashBuffer::launched) {
    if (!transfer_due) { transfer_due = retired + transfer_delay; }
    if (retired >= *transfer_due) {
      hcs08::require((options.read() & option_mask) == 0,
                     "The original SOPT store disables COP before any reference FLASH transfer");
      flash.advance(Hcs08FlashMilestone::transfer_buffer);
      transfer_due.reset();
      complete_due = retired + completion_delay;
      ++transfers;
    }
  }
  cuts += static_cast<unsigned>(cuts_after_retirement());
}

bool Memory::cuts_after_retirement() const {
  const auto state = flash.state();
  if (!state.active && state.buffer != Hcs08FlashBuffer::launched) { return false; }
  const auto current = flash.image();
  const auto conservative = flash.power_off_image();
  const auto bounded = flash.power_off_image(Hcs08FlashAbort::reference_command_range);
  hcs08::require(
      conservative.known.none() && conservative.fresh.none(),
      "Every actual pending/active P/E boundary retains unqualified physical corruption bounds");
  auto expected_known = current.known;
  auto expected_fresh = current.fresh;
  if (state.active) {
    const auto job = *state.active;
    const auto offset = job.address - hf::first;
    const auto begin = job.command == hf::erase_page ? (offset / page_bytes) * page_bytes : offset;
    const auto count = job.command == hf::erase_page ? page_bytes : 1U;
    for (unsigned n = begin; n < begin + count; ++n) {
      expected_known.reset(n);
      expected_fresh.reset(n);
    }
  }
  hcs08::require(
      bounded.data == current.data && bounded.known == expected_known &&
          bounded.fresh == expected_fresh,
      "Every actual boundary preserves data and marks only declared reference-command uncertainty");
  return true;
}

void step(Hcs08 &cpu, Memory &memory, std::ostream &trace) {
  const auto before = cpu.state();
  memory.accesses.clear();
  cpu.step();
  const auto after = cpu.state();
  const auto state = memory.flash.state();
  trace << "STEP n=" << memory.retired << " pc=" << before.pc << " next=" << after.pc
        << " a=" << static_cast<unsigned>(after.a) << " hx=" << after.hx << " sp=" << after.sp
        << " ccr=" << static_cast<unsigned>(after.ccr)
        << " buffer=" << static_cast<unsigned>(state.buffer)
        << " active=" << (state.active ? state.active->address : 0)
        << " transfers=" << memory.transfers << " completions=" << memory.completions << '\n';
  for (const auto &access : memory.accesses) {
    trace << "ACCESS address=" << access.address << " value=" << access.value
          << " write=" << access.write << '\n';
  }
}

void finish_pending(Memory &memory) {
  if (!memory.flash.state().active) { return; }
  hcs08::require(memory.complete_due.has_value(),
                 "Reference completion has an actual independently scheduled event");
  memory.flash.advance(Hcs08FlashMilestone::complete_command);
  memory.complete_due.reset();
  ++memory.completions;
}

std::vector<std::uint8_t> read_input(const char *path, const unsigned size) {
  std::ifstream stream(path, std::ios::binary);
  std::vector<std::uint8_t> result(size);
  stream.read(reinterpret_cast<char *>(result.data()), static_cast<std::streamsize>(size));
  hcs08::require(stream.gcount() == static_cast<std::streamsize>(size) &&
                     stream.peek() == std::char_traits<char>::eof(),
                 "Original input has its exact declared length");
  return result;
}
} // namespace blofeld::test::hcs08_programmer
