// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <cstdint>

namespace blofeld {
namespace hf = hcs08_flash;

namespace {
constexpr unsigned first_vector = 0xffc0, last_vector = 0xfffd, address_end = 65536;
constexpr unsigned disable_redirection = 0x40, disable_protection = 1, protection_mask = 0xfe;
}

std::uint8_t Hcs08Flash::read_interrupt_vector_byte(const std::uint16_t address) const try {
  if (agent_ != Hcs08FlashAgent::cpu || address < first_vector || address > last_vector) {
    fail("interrupt vector read", address, "requires an actual CPU interrupt vector fetch");
  }
  const bool known_disabled = ((options_.known & options_.value & disable_redirection) != 0) ||
                              ((protection_.known & protection_.value & disable_protection) != 0);
  if (known_disabled) { return read_array(address); }
  const auto protection =
      known_register(protection_, {.address = hf::protection, .mask = hf::erased});
  const auto boundary =
      (static_cast<unsigned>(protection & protection_mask) << 8U) + hf::page_bytes;
  // GT8A: a fully protected or entirely unprotected array does not redirect.
  if (boundary <= hf::first || boundary >= address_end) { return read_array(address); }
  const auto options =
      known_register(options_, {.address = hf::options, .mask = disable_redirection});
  if ((options & disable_redirection) != 0) { return read_array(address); }
  constexpr unsigned vector_page = 256;
  constexpr unsigned vector_offset = 0xff;
  const auto physical = boundary - vector_page + (address & vector_offset);
  return read_array(static_cast<std::uint16_t>(physical));
} catch (const Fault &error) { fail("interrupt vector read", address, error.what()); }
} // namespace blofeld
