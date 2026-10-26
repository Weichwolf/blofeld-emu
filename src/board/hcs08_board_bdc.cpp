// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board_memory.hpp"
#include "common/bus.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <cstdint>
#include <string>

namespace blofeld {
namespace {
bool ram(const unsigned address) {
  return address >= Hcs08BoardMemory::ram_first && address < Hcs08BoardMemory::ram_end;
}

bool registers(const unsigned address) {
  return address >= hcs08_flash::divider && address <= hcs08_flash::command;
}

[[noreturn]] void failure(const ExecutionContext context,
                          const std::string &operation,
                          const unsigned address,
                          const std::string &reason) {
  throw Fault("HCS08 board BDC " + operation + " address=" + hex(address) +
              " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr) + " agent=BDC: " + reason);
}
} // namespace

bool Hcs08BoardMemory::secured(const ExecutionContext context) const try {
  data_access("BDC security query", hcs08_flash::options);
  auto source = flash_;
  source.context(context, Hcs08FlashAgent::background);
  return source.security_engaged();
} catch (const Fault &error) {

  failure(context, "security query", hcs08_flash::options, error.what());
}

std::uint8_t Hcs08BoardMemory::read(const std::uint16_t address, const ExecutionContext context) {
  data_access("BDC read", address);
  context_ = context;
  flash_.context(context, Hcs08FlashAgent::background);
  healthy_ = false;
  try {
    std::uint8_t value = 0;
    if (ram(address)) {
      if (!flash_.security_engaged()) {
        const auto byte = ram_.at(address - ram_first);
        if (byte.known != Hcs08KnownByte::all) {
          failure(context, "read", address, "unknown SRAM operand bits");
        }
        value = byte.value;
      } // GT4.5: secured BDC reads of RAM return zero.
    } else if (address >= hcs08_flash::first) {
      value = flash_.read_array(address);
    } else if (registers(address)) {
      value = read_peripheral(address);
    } else {
      failure(context, "read", address, "peripheral/debugger interaction is unqualified");
    }
    flash_.context(context_);
    healthy_ = true;
    return value;
  } catch (const Fault &error) {
    flash_.context(context_);
    failure(context, "read", address, error.what());
  }
}

void Hcs08BoardMemory::write(const ByteWrite access, const ExecutionContext context) {
  data_access("BDC write", access.address);
  context_ = context;
  flash_.context(context, Hcs08FlashAgent::background);
  healthy_ = false;
  try {
    if (ram(access.address)) {
      if (!flash_.security_engaged()) {
        change(access.address);
        ram_.at(access.address - ram_first) = {.value = access.value};
      } // GT4.5: secured BDC writes of RAM are ignored.
    } else if (access.address >= hcs08_flash::first) {
      change(access.address);
      flash_.stage(access);
    } else if (registers(access.address)) {
      change(access.address);
      flash_.write(access);
    } else {
      failure(context, "write", access.address, "peripheral/debugger interaction is unqualified");
    }
    flash_.context(context_);
    healthy_ = true;
  } catch (const Fault &error) {
    flash_.context(context_);
    failure(context, "write", access.address, error.what());
  }
}
} // namespace blofeld
