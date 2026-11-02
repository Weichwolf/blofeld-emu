// SPDX-License-Identifier: GPL-3.0-only
#include "board/nand_bus_decode.hpp"

#include "common/bus.hpp"

namespace blofeld {
namespace {
constexpr unsigned controls_mask = 0xf00, strobes = 0x300, active_write = 0x100;
}

NandBusDecode::NandBusDecode(const NandBusContract contract) : contract_(contract) {
  if (contract != NandBusContract::ordered_pins &&
      contract != NandBusContract::functional_transactions) {
    throw Fault("Invalid NAND shared-bus contract");
  }
}

unsigned NandBusDecode::observe(const unsigned controls, const bool host_outputs) {
  if ((controls & ~controls_mask) != 0) { throw Fault("Invalid NAND bus control mask"); }
  const auto current = controls & strobes;
  const auto previous = previous_.value_or(strobes) & strobes;
  const bool active = current != 0 && current != strobes;
  const bool functional = contract_ == NandBusContract::functional_transactions;
  if (functional && (host_outputs || host_owned_)) {
    if (!host_owned_ && previous == active_write) {
      throw Fault("NAND functional bus operation=decode at=X:0xffffbd previous=" +
                  hex(previous_.value_or(strobes), 3) + " controls=" + hex(controls, 3) +
                  ": host ownership interrupts an unfinished NAND write");
    }
    // Returning the GPIO byte to input is not itself a new NAND read pulse.
    // Retained active strobes stay deselected until a complete inactive phase.
    host_owned_ = host_outputs || current != strobes;
    previous_ = controls;
    return host_owned_ ? strobes : controls;
  }
  if (contract_ == NandBusContract::functional_transactions &&
      ((current == 0 && previous == active_write) || (previous == 0 && active))) {
    throw Fault("NAND functional bus operation=decode at=X:0xffffbd previous=" +
                hex(previous_.value_or(strobes), 3) + " controls=" + hex(controls, 3) +
                ": neutral phase requires a completed write and inactive transfer entry");
  }
  previous_ = controls;
  return contract_ == NandBusContract::functional_transactions && current == 0 ? controls | strobes
                                                                               : controls;
}

NandBusContract NandBusDecode::contract() const {
  return contract_;
}

bool NandBusDecode::neutral() const {
  return contract_ == NandBusContract::functional_transactions &&
         (host_owned_ || (previous_ && (*previous_ & strobes) == 0));
}
}
