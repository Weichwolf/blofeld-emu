// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_dma.hpp"
#include "blofeld/dsp_esai.hpp"

#include <optional>
#include <string>

namespace blofeld {
namespace {
DspSpace space(const DspDmaSpace value) {
  switch (value) {
    case DspDmaSpace::x: return DspSpace::x;
    case DspDmaSpace::y: return DspSpace::y;
    case DspDmaSpace::p: return DspSpace::p;
    default: throw Fault("Invalid DSP DMA memory space");
  }
}

void check_access(const DspDmaAddress address, const bool write) {
  constexpr unsigned io_base = 0xffff80;
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
  constexpr unsigned external = 15;
  constexpr unsigned primary = 1U << 12U;
  constexpr unsigned secondary = 1U << 22U;
  if (((inputs.known_requests | inputs.requests) & ~external) != 0) {
    throw Fault("External DSP DMA requests may drive only IRQA..IRQD, not internal status");
  }
  const auto required = dma_.requested_sources();
  if ((required & ~(external | primary | secondary)) != 0) {
    throw Fault("Unsupported/unobserved DSP DMA request source at X:0xfffff4");
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
           }});
}
}
