// SPDX-License-Identifier: GPL-3.0-only
#include "board/nand_bus_decode.hpp"

#include "common/bus.hpp"

namespace blofeld {
namespace {
constexpr unsigned controls_mask = 0xf00, strobes = 0x300;
}

NandBusDecode::NandBusDecode(const NandBusContract contract) : contract_(contract) {
  if (contract != NandBusContract::ordered_pins &&
      contract != NandBusContract::functional_transactions) {
    throw Fault("Invalid NAND shared-bus contract");
  }
}

unsigned NandBusDecode::observe(const unsigned controls) {
  if ((controls & ~controls_mask) != 0) { throw Fault("Invalid NAND bus control mask"); }
  const auto current = controls & strobes;
  const auto previous = previous_.value_or(strobes) & strobes;
  const bool active = current != 0 && current != strobes;
  const bool was_active = previous != 0 && previous != strobes;
  if (contract_ == NandBusContract::functional_transactions &&
      ((current == 0 && was_active) || (previous == 0 && active))) {
    throw Fault("NAND functional bus operation=decode at=X:0xffffbd previous=" +
                hex(previous_.value_or(strobes), 3) + " controls=" + hex(controls, 3) +
                ": neutral phase requires complete inactive transfers");
  }
  previous_ = controls;
  return contract_ == NandBusContract::functional_transactions && current == 0 ? controls | strobes
                                                                               : controls;
}

NandBusContract NandBusDecode::contract() const {
  return contract_;
}

bool NandBusDecode::neutral() const {
  return contract_ == NandBusContract::functional_transactions && previous_ &&
         (*previous_ & strobes) == 0;
}
}
