// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_dax.hpp"
#include "blofeld/dsp_dma.hpp"
#include "blofeld/dsp_esai.hpp"
#include "blofeld/dsp_gpio.hpp"

#include <array>
#include <optional>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned io_base = 0xffff80;

bool controller_read(const DspMemoryAccess access) {
  return access.operation == DspMemoryOperation::read && access.target.space == DspSpace::x &&
         DspDma::handles(access.target.address);
}

DspSpace space(const DspDmaSpace value) {
  switch (value) {
    case DspDmaSpace::x: return DspSpace::x;
    case DspDmaSpace::y: return DspSpace::y;
    case DspDmaSpace::p: return DspSpace::p;
    default: throw Fault("Invalid DSP DMA memory space");
  }
}

std::string describe(const DspAddress address) {
  constexpr std::array names{"P:", "X:", "Y:"};
  return std::string(names.at(static_cast<unsigned>(address.space))) + hex(address.address, 6);
}

bool core_conflict(const DspBus &bus, const DspCoreBusUse core, const DspMemoryAccess target) {
  bool conflict = false;
  for (const auto access : core.accesses) {
    // Controller inspection does not use the DMA source/destination data path.
    if (controller_read(access)) { continue; }
    const auto relation = bus.memory_conflict({.core = access, .dma = target});
    if (relation == DspMemoryConflict::unqualified) {
      const bool write = target.operation == DspMemoryOperation::write;
      throw Fault("Unqualified DSP core/DMA bank contention at " +
                  std::string(write ? "destination write " : "source read ") +
                  describe(target.target) + " core=" + describe(access.target));
    }
    conflict = conflict || relation == DspMemoryConflict::protected_partition;
  }
  return conflict;
}

void check_access(const DspDmaAddress address, const bool write) {
  if (address.space == DspDmaSpace::p || address.address < io_base) { return; }
  constexpr unsigned primary_tx = 0xffffa0;
  constexpr unsigned secondary_tx = 0xffff80;
  constexpr unsigned dax_non_audio = 0xffffd1;
  constexpr unsigned dax_audio_a = 0xffffd2;
  constexpr unsigned dax_audio_b = 0xffffd3;
  const bool x = address.space == DspDmaSpace::x;
  const auto first = x ? primary_tx : secondary_tx;
  if (write && ((address.address >= first && address.address - first < 6) ||
                (x && (address.address == dax_non_audio || address.address == dax_audio_a ||
                       address.address == dax_audio_b)))) {
    return;
  }
  throw Fault(std::string("Unsupported DSP DMA peripheral ") + (write ? "write at " : "read at ") +
              (x ? "X:" : "Y:") + hex(address.address, 6));
}
}

std::optional<DspDmaWord> DspBus::advance_dma(DspDmaInputs inputs) {
  return advance_dma_with_admission(inputs, {});
}

std::optional<DspDmaWord> DspBus::advance_dma_with_admission(DspDmaInputs inputs,
                                                             const DspDmaAdmission &admission) {
  constexpr unsigned external = 15;
  constexpr unsigned primary = 1U << 12U;
  constexpr unsigned secondary = 1U << 22U;
  constexpr unsigned dax_request = 1U << 10U;
  if (((inputs.known_requests | inputs.requests) & ~external) != 0) {
    throw Fault("External DSP DMA requests may drive only IRQA..IRQD, not internal status");
  }
  const auto required = dma_.requested_sources();
  if ((required & ~(external | primary | secondary | dax_request)) != 0) {
    throw Fault("Unsupported/unobserved DSP DMA request source at X:0xfffff4");
  }
  if ((required & dax_request) != 0 || gpio(DspPort::d).outputs().peripheral == 0) {
    sync_dax();
    inputs.known_requests |= dax_request;
    if (dax_.request()) { inputs.requests |= dax_request; }
  }
  for (const auto instance : {DspEsaiInstance::primary, DspEsaiInstance::secondary}) {
    const auto bit = instance == DspEsaiInstance::primary ? primary : secondary;
    // Preserve a real low baseline before DE, without reading SAISR or arming TUE clear.
    const auto &serial = esai_.at(instance == DspEsaiInstance::primary ? 0U : 1U);
    if ((required & bit) != 0 || !serial.needs_transmit_clock()) {
      inputs.known_requests |= bit;
      if (serial.transmit_request()) { inputs.requests |= bit; }
    }
  }
  return dma_.advance(
      inputs,
      {.read =
           [this](const DspDmaAddress address) {
             check_access(address, false);
             return read({.space = space(address.space), .address = address.address});
           },
       .write =
           [this](const DspDmaAddress address, const auto value) {
             check_access(address, true);
             write({.space = space(address.space), .address = address.address, .value = value});
           },
       .admit = admission});
}

DspDmaBusResult DspBus::advance_dma(const DspDmaInputs inputs, const DspCoreBusUse core) {
  if (core.knowledge != DspCoreBusKnowledge::reference_cycle_occupancy) {
    throw Fault("DSP DMA bank arbitration requires explicitly known core-cycle occupancy");
  }
  for (const auto access : core.accesses) {
    if (!controller_read(access)) { static_cast<void>(memory_resource(access)); }
  }
  DspDmaBusResult result;
  const DspDmaAdmission admission = [&](const DspDmaPhase &phase) {
    result.phase = phase;
    const bool write = phase.kind == DspDmaPhaseKind::destination_write;
    const auto selected = write ? phase.destination : phase.source;
    const DspMemoryAccess target{
        .target = {.space = space(selected.space), .address = selected.address},
        .operation = write ? DspMemoryOperation::write : DspMemoryOperation::read};
    if (selected.space != DspDmaSpace::p && selected.address >= io_base) {
      // Core footprints contain physical memory or DMA-controller reads, not data I/O.
      // FM10.2/APR23 section5: these resources do not contend with a data register.
      check_access(selected, write);
      return DspDmaGrant::available;
    }
    const bool conflict = core_conflict(*this, core, target);
    result.core_stall = write && conflict;
    return !write && conflict ? DspDmaGrant::wait : DspDmaGrant::available;
  };
  result.word = advance_dma_with_admission(inputs, admission);
  return result;
}
}
