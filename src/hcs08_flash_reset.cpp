// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hcs08_flash.hpp"

namespace blofeld {
namespace hf = hcs08_flash;

Hcs08FlashImage Hcs08Flash::power_off_image(const Hcs08FlashAbort contract) const {
  if (contract != Hcs08FlashAbort::unqualified_retention &&
      contract != Hcs08FlashAbort::reference_command_range) {
    fail("power loss", hf::status, "invalid retention contract");
  }
  auto result = image_;
  const auto invalidate = [&result, contract](const Hcs08FlashJob job) {
    if (job.command == hf::verify) { return; }
    if (contract == Hcs08FlashAbort::unqualified_retention) {
      result.known.reset();
      result.fresh.reset();
      return;
    }
    const auto offset = job.address - hf::first;
    auto begin = offset;
    unsigned count = 1;
    if (job.command == hf::erase_all) {
      begin = 0;
      count = hf::bytes;
    } else if (job.command == hf::erase_page) {
      begin = (offset / hf::page_bytes) * hf::page_bytes;
      count = hf::page_bytes;
    }
    for (unsigned n = begin; n < begin + count; ++n) {
      result.known.reset(n);
      result.fresh.reset(n);
    }
  };
  if (state_.active) { invalidate(*state_.active); }
  if (contract == Hcs08FlashAbort::unqualified_retention &&
      state_.buffer == Hcs08FlashBuffer::launched) {
    invalidate(state_.staged);
  }
  return result;
}

void Hcs08Flash::reset(const Hcs08FlashAbort contract) {
  image_ = power_off_image(contract);
  state_ = {};
  bus_hz_.reset();
  load_configuration();
}
} // namespace blofeld
