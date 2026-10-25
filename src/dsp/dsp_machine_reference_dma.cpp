// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <array>
#include <variant>

namespace blofeld {

DspReferenceDmaProgress DspMachine::advance_reference_dma(const DspDmaInputs inputs,
                                                          const DspReferenceTicket &ticket) {
  if (clocked()) {
    throw Fault("Native DMA memory admission requires an unclocked reference owner");
  }
  const auto pending = reference_pending();
  if (!pending || pending->ticket != ticket) {
    throw Fault("Native DMA memory admission requires the current owner ticket");
  }
  const auto *access = std::get_if<DspNativeAccess>(&pending->effect);
  if (access == nullptr) {
    throw Fault("Unqualified native DMA admission of reset or interrupt acknowledgement");
  }
  switch (access->kind) {
    case DspNativeAccessKind::opcode:
    case DspNativeAccessKind::extension:
    case DspNativeAccessKind::vector:
    case DspNativeAccessKind::read:
    case DspNativeAccessKind::write: break;
    case DspNativeAccessKind::bit:
      throw Fault("Unqualified indivisible native bit operation in DMA memory admission at " +
                  hex(access->target.address, 6) + " pc=" + hex(access->pc, 6) +
                  " omr=" + hex(access->omr, 6));
    default: throw Fault("Invalid native operation in DMA memory admission");
  }
  const std::array footprint{DspMemoryAccess{.target = access->target,
                                             .operation = access->kind == DspNativeAccessKind::write
                                                              ? DspMemoryOperation::write
                                                              : DspMemoryOperation::read}};
  DspReferenceDmaProgress result;
  result.dma = advance_dma(
      inputs, {.accesses = footprint, .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy});
  if (!result.dma.core_stall) { result.core = advance_reference_access(ticket); }
  return result;
}

}
