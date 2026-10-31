// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
class OwnDspMachine;
enum class OwnDspNativeCycleModel : std::uint8_t;

struct OwnDspFetchCycleResult {
  DspDmaBusResult dma;
  bool captured = false;
  std::optional<dsp56300::InstructionProgress> native;
};

// Untimed F1/F2 owner. The machine must outlive this object. Reset and restore
// invalidate its identity; a new owner must establish a fresh fetch context.
class OwnDspFetch {
public:
  explicit OwnDspFetch(OwnDspMachine &machine);
  OwnDspFetch(const OwnDspFetch &) = delete;
  OwnDspFetch &operator=(const OwnDspFetch &) = delete;
  [[nodiscard]] DspProgramFetchView view() const;
  void issue();
  void capture();
  // Explicit DMA-first cycle for the actual issued F2 read; the CPU is held.
  [[nodiscard]] OwnDspFetchCycleResult advance_dma_capture(DspDmaInputs inputs);
  // Explicit DMA, F2, native-effect order with a global DMA destination stall.
  [[nodiscard]] OwnDspFetchCycleResult advance_dma_capture_native(DspDmaInputs inputs,
                                                                  OwnDspNativeCycleModel model);
  // Admission uses the actual owned foreground F2 words. Vector reads remain
  // native staged bus effects and are acknowledged by the interrupt owner.
  [[nodiscard]] bool admit_interrupt();
  [[nodiscard]] dsp56300::InstructionProgress begin();
  // Call only after the admitted instruction's native retirement.
  void synchronize();

private:
  void check_owner() const;
  void check_sample_context() const;
  [[nodiscard]] OwnDspFetchCycleResult capture_cycle(DspDmaInputs inputs, bool admit_native);
  [[nodiscard]] dsp56300::InstructionProgress admit(const DspFetchedInstruction &captured);
  [[nodiscard]] dsp56300::InstructionProgress begin_native();
  void finish_interrupt_vector();
  OwnDspMachine &machine_;
  std::shared_ptr<const bool> identity_;
  DspProgramFetch fetch_;
  std::optional<DspFetchedInstruction> admitted_;
  std::optional<DspFetchedInstruction> repeated_;
  std::uint64_t instructions_ = 0;
  std::optional<std::uint32_t> interrupt_vector_;
  bool in_flight_ = false;
  bool native_fetch_ = false;
  bool native_latch_ = false;
};
}
