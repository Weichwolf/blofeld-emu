// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_flash.hpp"

#include "common/bus.hpp"

#include <cstdint>
#include <string>

namespace blofeld {
namespace hf = hcs08_flash;

namespace {
constexpr unsigned option_mask = 0xc3, security_mask = 3, unsecured = 2;
constexpr unsigned ram_first = 0x80, ram_end = 0x480, address_end = 65536;
constexpr unsigned maximum_bus_hz = 20000000;
}

Hcs08Flash::Hcs08Flash(const Hcs08FlashImage &image,
                       const Hcs08FlashContract contract,
                       const Hcs08IdleLaunch idle)
    : image_(image), idle_(idle) {
  if (contract != Hcs08FlashContract::candidate_gt8a) {
    fail("configure", hf::first, "requires the explicit candidate GT8A memory contract");
  }
  if (idle != Hcs08IdleLaunch::unqualified && idle != Hcs08IdleLaunch::reference_ignore_empty) {
    fail("configure", hf::status, "invalid idle-launch contract");
  }
  for (unsigned n = 0; n < hf::bytes; ++n) {
    if (image_.fresh[n] && (!image_.known[n] || image_.data[n] != hf::erased)) {
      fail("configure", hf::first + n, "fresh erase history requires known erased data");
    }
  }
  load_configuration();
}

void Hcs08Flash::context(const ExecutionContext value, const Hcs08FlashAgent agent) {
  if (agent != Hcs08FlashAgent::cpu && agent != Hcs08FlashAgent::background) {
    fail("context", value.pc, "invalid access agent");
  }
  context_ = value;
  agent_ = agent;
}

void Hcs08Flash::load_configuration() {
  const auto load = [this](const unsigned address, const unsigned mask) {
    const auto offset = address - hf::first;
    return KnownByte{.value = static_cast<std::uint8_t>(image_.data[offset] & mask),
                     .known = static_cast<std::uint8_t>(image_.known[offset] ? mask : 0)};
  };
  options_ = load(hf::nv_options, option_mask);
  protection_ = load(hf::nv_protection, hf::erased);
}

unsigned Hcs08Flash::index(const std::uint16_t address) const {
  if (address < hf::first) { fail("array access", address, "outside candidate FLASH array"); }
  return address - hf::first;
}

std::uint8_t Hcs08Flash::known_register(const KnownByte value, const RegisterRead access) const {
  if ((value.known & access.mask) != access.mask) {
    fail("register read", access.address, "unknown reset-latched bits");
  }
  return static_cast<std::uint8_t>(value.value & access.mask);
}

bool Hcs08Flash::secure() const {
  return known_register(options_, {.address = hf::options, .mask = security_mask}) != unsecured;
}

bool Hcs08Flash::blocked() const {
  const bool secure_cpu =
      agent_ == Hcs08FlashAgent::cpu &&
      ((context_.pc >= ram_first && context_.pc < ram_end) || context_.pc >= hf::first);
  return !secure_cpu && secure();
}

bool Hcs08Flash::protected_address(const unsigned address) const {
  const auto value = known_register(protection_, {.address = hf::protection, .mask = hf::erased});
  const auto boundary = (static_cast<unsigned>(value & 0xfeU) << 8U) + hf::page_bytes;
  return (value & 1U) == 0 && boundary < address_end && address >= boundary;
}

bool Hcs08Flash::command_protected(const Hcs08FlashJob job) const {
  if (job.command == hf::verify) { return false; }
  return protected_address(job.command == hf::erase_all ? address_end - 1 : job.address);
}

std::uint8_t Hcs08Flash::status() const {
  const bool available = state_.buffer != Hcs08FlashBuffer::launched &&
                         (!state_.active || state_.active->command == hf::burst);
  const bool done = !state_.active && state_.buffer != Hcs08FlashBuffer::launched;
  return static_cast<std::uint8_t>((available ? hf::empty : 0) | (done ? hf::complete : 0) |
                                   state_.errors | (state_.blank ? hf::blank : 0));
}

Hcs08FlashState Hcs08Flash::state() const {
  return state_;
}

bool Hcs08Flash::security_engaged() const {
  return secure();
}

const Hcs08FlashImage &Hcs08Flash::image() const {
  return image_;
}

void Hcs08Flash::observe_bus_clock(const std::uint32_t hz) {
  if (state_.active || state_.buffer == Hcs08FlashBuffer::launched) {
    fail("clock", hf::divider, "unqualified clock change during a command");
  }
  if (hz == 0 || hz > maximum_bus_hz) {
    fail("clock", hf::divider, "outside the candidate bus clock limit");
  }
  bus_hz_ = hz;
}

[[noreturn]] void Hcs08Flash::fail(const std::string &operation,
                                   const unsigned address,
                                   const std::string &reason) const {
  throw Fault("HCS08 FLASH " + operation + " address=" + hex(address) + " pc=" + hex(context_.pc) +
              " ccr=" + hex(context_.ccr) +
              (agent_ == Hcs08FlashAgent::cpu ? " agent=CPU: " : " agent=BDC: ") + reason);
}
} // namespace blofeld
