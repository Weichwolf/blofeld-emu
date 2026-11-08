// SPDX-License-Identifier: GPL-3.0-only
#include "io/mcu_flash.hpp"
#include "storage/flash_image.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
using blofeld::vi::McuFlash;
constexpr std::uint16_t divider = 0x100, security = 0x101, configuration = 0x103;
constexpr std::uint16_t protection = 0x104, status = 0x105, command = 0x106;
constexpr std::uint16_t protect_byte = 0x7f0d, secure_byte = 0x7f0f;
constexpr std::uint8_t ready = 0xc0, access_error = 0x10, violation = 0x20;

constexpr std::uint8_t all_bits = 0xff, unsecured = 0xfe;
constexpr std::uint8_t prescaled_divisor = 0x43, loaded_divisor = 0xc3;
constexpr std::uint8_t readonly_status = 0x4f;
constexpr std::uint8_t protect_top = 0xc7, change_top_size = 0xdf, protect_all = 0x7f;
constexpr std::uint16_t reserved_first = 0x107, reserved_last = 0x10f;
constexpr std::uint16_t array_last = 0x7fff, top_range_first = 0x7800, datum = 0x1234;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::unique_ptr<blofeld::vi::FlashImage> image() {
  auto result = std::make_unique<blofeld::vi::FlashImage>();
  result->bytes.fill(all_bits);
  result->known.fill(1);
  result->bytes[secure_byte] = unsecured;
  return result;
}

std::uint8_t read(const McuFlash &flash, const std::uint16_t offset) {
  std::uint8_t value = 0;
  require(flash.read(offset, value), "Read a known normal-mode Flash register");
  return value;
}

void reset_and_registers() {
  auto persistent = image();
  McuFlash flash(*persistent);
  require(read(flash, divider) == 0 && read(flash, status) == ready,
          "Reset leaves the divider unloaded and both command buffers idle");
  require(read(flash, security) == unsecured && read(flash, protection) == all_bits,
          "Security and protection reset values come from the actual image");
  require(flash.write({.offset = security, .value = 0}) && read(flash, security) == unsecured,
          "FSEC is read-only, including the security bits");
  require(flash.write({.offset = divider, .value = prescaled_divisor}) &&
              read(flash, divider) == loaded_divisor,
          "The first divider write loads FDIV/PRDIV8 and sets FDIVLD");
  require(flash.write({.offset = divider, .value = 0}) && read(flash, divider) == loaded_divisor,
          "Subsequent divider writes have no effect until reset");
  for (std::uint16_t offset = reserved_first; offset <= reserved_last; ++offset) {
    require(flash.write({.offset = offset, .value = all_bits}) && read(flash, offset) == 0,
            "Reserved and normal-mode FADDR/FDATA bits stay zero");
  }
  persistent->bytes[secure_byte] = 0;
  require(read(flash, security) == unsecured,
          "Reset-latched security does not follow array changes");
  flash.reset(*persistent);
  require(read(flash, security) == 0 && read(flash, divider) == 0,
          "Reset reloads the persistent configuration and unlocks FCLKDIV");
  persistent->known[secure_byte] = 0;
  persistent->known[protect_byte] = 0;
  flash.reset(*persistent);
  std::uint8_t value = 0;
  require(!flash.read(security, value) && !flash.read(protection, value),
          "Missing configuration bytes are never replaced with defaults");
}

void invalid_array_writes() {
  const auto persistent = image();
  McuFlash flash(*persistent);
  require(flash.write_array({.offset = array_last, .value = 8}),
          "A byte write is handled by the Flash command state machine");
  require(read(flash, status) == (ready | access_error) && persistent->bytes.back() == all_bits,
          "The early uninitialized-stack store sets ACCERR without changing the reset vector");
  require(flash.write({.offset = status, .value = readonly_status}) &&
              read(flash, status) == (ready | access_error),
          "Read-only and reserved FSTAT bits cannot change or clear ACCERR");
  require(flash.write({.offset = status, .value = access_error}) && read(flash, status) == ready,
          "ACCERR is cleared by writing one, without altering readiness");
  require(flash.write_array({.offset = 0, .value = datum, .bytes = 2}) &&
              read(flash, status) == (ready | access_error),
          "An aligned word before loading FCLKDIV also sets ACCERR");
  require(flash.write({.offset = status, .value = access_error}) &&
              flash.write({.offset = divider, .value = 4}),
          "Clear the error and initialize the divider");
  require(flash.write_array({.offset = 1, .value = datum, .bytes = 2}) &&
              read(flash, status) == (ready | access_error),
          "A misaligned word is invalid even after clock-divider initialization");
  require(persistent->bytes[0] == all_bits && persistent->bytes[1] == all_bits &&
              persistent->erased_words[0] == 0 && persistent->known[0] == 1,
          "Rejected writes preserve data, knowledge and erase eligibility");
}

void protection_locks() {
  const auto persistent = image();
  McuFlash flash(*persistent);
  require(flash.write({.offset = divider, .value = 4}), "Initialize the Flash divider");
  require(flash.write({.offset = protection, .value = protect_top}) &&
              read(flash, protection) == protect_top,
          "An open array can acquire top 2 KiB protection");
  require(flash.write_array({.offset = top_range_first, .value = 0, .bytes = 2}) &&
              read(flash, status) == (ready | violation),
          "A protected word sets PVIOL rather than writing data");
  require(flash.write({.offset = status, .value = violation}) && read(flash, status) == ready,
          "PVIOL is independently write-one-to-clear");
  require(flash.write({.offset = protection, .value = all_bits}) &&
              read(flash, protection) == protect_top,
          "A locked high range cannot be disabled");
  require(flash.write({.offset = protection, .value = change_top_size}) &&
              read(flash, protection) == protect_top,
          "The locked high range size cannot change");
  flash.reset(*persistent);
  require(flash.write({.offset = divider, .value = 4}) &&
              flash.write({.offset = protection, .value = protect_all}),
          "Select full protection by closing both disabled ranges");
  require(flash.write({.offset = protection, .value = all_bits}) &&
              read(flash, protection) == protect_all,
          "FPOPEN cannot return from zero to one");
  require(flash.write_array({.offset = 0, .value = 0, .bytes = 2}) &&
              read(flash, status) == (ready | violation),
          "Full-array protection includes the lower alias offset");
}

void unsupported_is_explicit() {
  const auto persistent = image();
  McuFlash flash(*persistent);
  require(flash.write({.offset = divider, .value = 4}), "Initialize the Flash divider");
  require(!flash.write_array({.offset = 0, .value = datum, .bytes = 2}) &&
              !flash.write({.offset = command, .value = 0x20}) &&
              !flash.write({.offset = status, .value = 0x80}),
          "Valid program/erase sequences fail explicitly until clocked commands are bound");
  require(!flash.write({.offset = configuration, .value = 0x80}) &&
              !flash.write({.offset = configuration, .value = 0x40}) &&
              !flash.write({.offset = configuration, .value = 0x20}),
          "Unbound interrupt delivery and security-key mode are not silently enabled");
  require(read(flash, status) == ready && read(flash, command) == 0 &&
              read(flash, configuration) == 0 && persistent->bytes[0] == all_bits,
          "Refused paths do not manufacture completion or mutate persistent state");
  require(flash.write({.offset = command, .value = all_bits}) &&
              read(flash, status) == (ready | access_error),
          "An illegal command encoding sets the documented access-error flag");
  std::uint8_t value = 0;
  require(!flash.read(all_bits, value) && !flash.write({.offset = reserved_last + 1}) &&
              !flash.write_array({.offset = 0x8000}) &&
              !flash.write({.offset = divider, .value = 0x100}),
          "Register and array boundaries do not swallow unknown accesses");
}
}

int main() try {
  reset_and_registers();
  invalid_array_writes();
  protection_locks();
  unsupported_is_explicit();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
