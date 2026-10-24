// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hcs08_flash.hpp"

#include <cstdint>

namespace blofeld {
namespace hf = hcs08_flash;

namespace {
constexpr unsigned minimum_fclk = 150000, maximum_fclk = 200000;
constexpr unsigned reset_vector = 0xfffe;
constexpr unsigned prescaler = 0x40, divisor_mask = 0x3f, security_mask = 3;
}

std::uint8_t Hcs08Flash::read_reset_vector_byte(const std::uint16_t address) const {
  if (agent_ != Hcs08FlashAgent::cpu || (address != reset_vector && address != reset_vector + 1)) {
    fail("reset vector read", address, "requires CPU reset fetch at FFFE/FFFF");
  }
  if (state_.active || state_.buffer == Hcs08FlashBuffer::launched) {
    fail("reset vector read",
         address,
         "array read during an executing/pending command is unqualified");
  }
  const auto offset = index(address);
  if (!image_.known[offset]) { fail("reset vector read", address, "unknown nonvolatile contents"); }
  return image_.data[offset];
}

std::uint8_t Hcs08Flash::read_array(const std::uint16_t address) const {
  const auto offset = index(address);
  if (blocked()) { return 0; } // Documented secured-resource read, not unknown-data fill.
  if (state_.active || state_.buffer == Hcs08FlashBuffer::launched) {
    fail("array read", address, "array read during an executing/pending command is unqualified");
  }
  if (!image_.known[offset]) { fail("array read", address, "unknown nonvolatile contents"); }
  return image_.data[offset];
}

void Hcs08Flash::stage(const ByteWrite access) {
  static_cast<void>(index(access.address));
  // Secured BDC still needs the controller's address/data latch for blank/mass
  // erase. FCMD rejects its other commands; this write never changes array data.
  if (agent_ != Hcs08FlashAgent::background && blocked()) { return; }
  if (state_.errors != 0) { return; }
  if (!state_.divider_loaded || (status() & hf::empty) == 0 ||
      state_.buffer != Hcs08FlashBuffer::empty) {
    reject(hf::access_error);
    return;
  }
  state_.staged = {.address = access.address, .value = access.value};
  state_.buffer = Hcs08FlashBuffer::data;
}

void Hcs08Flash::transfer() {
  if (state_.active || state_.buffer != Hcs08FlashBuffer::launched) {
    fail("transfer", hf::status, "no available array slot and launched command");
  }
  const auto divisor =
      ((state_.divider & divisor_mask) + 1U) * ((state_.divider & prescaler) != 0 ? 8U : 1U);
  if (!bus_hz_ || !state_.divider_loaded) {
    fail("transfer", hf::divider, "unknown operating clock or unloaded divisor");
  }
  if (*bus_hz_ < minimum_fclk * divisor || *bus_hz_ > maximum_fclk * divisor) {
    fail("transfer", hf::divider, "NVM clock outside150..200kHz");
  }
  if ((state_.staged.command == hf::program || state_.staged.command == hf::burst) &&
      !image_.fresh[index(state_.staged.address)]) {
    fail("transfer",
         state_.staged.address,
         "reprogramming without known successful erase is unqualified");
  }
  state_.active = state_.staged;
  state_.buffer = Hcs08FlashBuffer::empty;
  state_.staged = {};
}

void Hcs08Flash::finish() {
  if (!state_.active) { fail("complete", hf::status, "no executing command"); }
  const auto job = *state_.active;
  const auto offset = index(job.address);
  if (job.command == hf::program || job.command == hf::burst) {
    image_.data[offset] = job.value;
    image_.known.set(offset);
    image_.fresh.reset(offset);
  } else if (job.command == hf::verify) {
    bool all_blank = true;
    for (unsigned n = 0; n < hf::bytes; ++n) {
      if (!image_.known[n]) { fail("blank check", hf::first + n, "unknown nonvolatile contents"); }
      if (image_.data[n] != hf::erased) { all_blank = false; }
    }
    state_.blank = all_blank;
    if (all_blank) {
      options_.value = static_cast<std::uint8_t>((options_.value & ~security_mask) | 2U);
      options_.known = static_cast<std::uint8_t>(options_.known | security_mask);
    }
  } else {
    const auto begin =
        job.command == hf::erase_all ? 0U : (offset / hf::page_bytes) * hf::page_bytes;
    const auto count = job.command == hf::erase_all ? hf::bytes : hf::page_bytes;
    for (unsigned n = begin; n < begin + count; ++n) {
      image_.data[n] = hf::erased;
      image_.known.set(n);
      image_.fresh.set(n);
    }
  }
  state_.active.reset();
}

void Hcs08Flash::advance(const Hcs08FlashMilestone milestone) {
  switch (milestone) {
    case Hcs08FlashMilestone::transfer_buffer: transfer(); return;
    case Hcs08FlashMilestone::complete_command: finish(); return;
    default: fail("advance", hf::status, "unknown external milestone");
  }
}
} // namespace blofeld
