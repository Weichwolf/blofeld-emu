// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/gpio_pads.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp/dsp_timers.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace blofeld {

enum class DspSpace : std::uint8_t { p, x, y };

struct DspAddress {
  DspSpace space = DspSpace::p;
  std::uint32_t address = 0;
};

struct DspStore {
  DspSpace space = DspSpace::p;
  std::uint32_t address = 0;
  std::uint32_t value = 0;
};

enum class DspMemoryOperation : std::uint8_t { read, write };
enum class DspMemoryPhase : std::uint8_t { execute, peripheral_sample };
enum class DspMemoryConflict : std::uint8_t { independent, protected_partition, unqualified };
enum class DspCoreBusKnowledge : std::uint8_t { unqualified, reference_cycle_occupancy };

struct DspMemoryAccess {
  DspAddress target{};
  DspMemoryOperation operation = DspMemoryOperation::read;
  DspMemoryPhase phase = DspMemoryPhase::execute;
};

struct DspMemoryPair {
  DspMemoryAccess core{};
  DspMemoryAccess dma{};
};

// Physical storage identity resolved by the strict map, including OMR aliases.
// block_words=0 means same-bank contention geometry is not qualified.
struct DspMemoryResource {
  DspSpace bank = DspSpace::p;
  bool rom = false;
  std::uint32_t word = 0;
  unsigned block_words = 0;
  bool operator==(const DspMemoryResource &) const = default;
};

struct DspCoreBusUse {
  std::span<const DspMemoryAccess> accesses;
  DspCoreBusKnowledge knowledge = DspCoreBusKnowledge::unqualified;
};

struct DspDmaBusResult {
  std::optional<DspDmaWord> word = std::nullopt;
  std::optional<DspDmaPhase> phase = std::nullopt;
  bool core_stall = false;
};

enum class DspBitOperation : std::uint8_t { test, clear, set, change };

struct DspBitAccess {
  DspSpace space = DspSpace::x;
  std::uint32_t address = 0;
  unsigned bit = 0;
  DspBitOperation operation = DspBitOperation::test;
};

struct DspRom {
  DspSpace space = DspSpace::p;
  std::uint32_t start = 0;
  std::span<const std::uint32_t> words;
};

struct DspCodeChanges {
  bool all = false;
  std::vector<std::uint32_t> addresses;
};

class DspBus {
public:
  DspBus();
  void reset();
  void reset_peripherals();
  [[nodiscard]] std::uint32_t read(DspAddress access);
  void write(DspStore access);
  [[nodiscard]] std::optional<bool> access_peripheral_bit(DspBitAccess access);
  [[nodiscard]] static bool handles_peripheral_bit(DspAddress access);
  [[nodiscard]] DspMemoryResource memory_resource(DspMemoryAccess access) const;
  [[nodiscard]] bool stable_program_context(std::uint32_t address, std::uint32_t omr) const;
  [[nodiscard]] DspMemoryConflict memory_conflict(DspMemoryPair pair) const;
  // Decoder lookahead may substitute an unknown word; never use it to admit execution.
  [[nodiscard]] std::uint32_t peek_program(std::uint32_t address) const;
  // Strict, side-effect-free P-space observation, including map/knowledge checks.
  [[nodiscard]] std::uint32_t inspect_program(std::uint32_t address) const;

  [[nodiscard]] std::optional<std::uint64_t> last_dma_cycle() const { return dma_.last_cycle(); }

  [[nodiscard]] bool has_active_dma() const { return dma_.active(); }

  void install_rom(DspRom image);
  void observe_omr(std::uint32_t value);
  void advance_instruction_cycles(unsigned cycles);
  [[nodiscard]] DspCodeChanges take_code_changes();
  [[nodiscard]] std::optional<DspInterruptRequest> interrupt_request(unsigned mask) const;
  void observe_irqa(std::optional<bool> level);
  // Post-boot IRQA driver plus the DSP56371UM table2-6 internal pull-up.
  // Opposed passive pulls or unknown active drivers preserve an unknown level.
  void observe_irqa_driver(GpioNetDriver driver);
  void acknowledge_interrupt(DspInterruptRequest request);
  // Explicit nominal admission: later higher-priority requests stay pending.
  void validate_admitted_interrupt(DspInterruptRequest request) const;
  void acknowledge_admitted_interrupt(DspInterruptRequest request);

  [[nodiscard]] DspShi &shi() { return shi_; }

  [[nodiscard]] DspPll &pll() { return pll_; }

  [[nodiscard]] const DspPll &pll() const { return pll_; }

  [[nodiscard]] DspGpio &gpio(DspPort port);

  [[nodiscard]] DspTimers &timers() { return timers_; }

  void observe_esai_transmit(DspEsaiInstance instance, DspEsaiInputs inputs);
  [[nodiscard]] DspEsaiOutputs esai_transmit_outputs(DspEsaiInstance instance) const;
  [[nodiscard]] DspEsaiClockOutputs esai_clock_outputs(DspEsaiInstance instance) const;
  // Explicit standalone reference phases; automatic native DMA bus timing rejects.
  [[nodiscard]] std::optional<DspDmaWord> advance_dma(DspDmaInputs inputs);
  [[nodiscard]] DspDmaBusResult advance_dma(DspDmaInputs inputs, DspCoreBusUse core);
  void observe_dax(DspDaxInputs inputs);
  [[nodiscard]] DspDaxOutput dax_output() const;

  [[nodiscard]] std::optional<std::uint32_t> once_latch() const { return once_latch_; }

private:
  friend class DspReferenceTape;
  friend class DspFetchRun;
  void advance_reference_cycle_delays(std::uint64_t admitted_dma_cycle, bool switch_was_pending);
  void advance_core_delays(unsigned cycles);
  void advance_memory_switch(unsigned cycles);
  void acknowledge_latched_interrupt(DspInterruptRequest request);
  [[nodiscard]] bool interrupt_source_pending(DspInterruptRequest request) const;
  enum class Operation : std::uint8_t { read, write };

  struct Memory {
    std::vector<std::uint32_t> words;
    std::vector<std::uint8_t> known;
  };

  struct Location {
    std::size_t bank = 0;
    std::size_t offset = 0;
    bool rom = false;
  };

  struct MemoryLayout {
    std::uint32_t program_end = 0;
    std::uint32_t x_end = 0;
    std::uint32_t y_end = 0;
  };

  [[nodiscard]] static MemoryLayout memory_layout(unsigned mode);
  [[nodiscard]] bool switching_address(DspAddress access) const;
  [[nodiscard]] Location locate(DspAddress access, Operation operation = Operation::read) const;
  [[nodiscard]] Location locate_program(std::uint32_t address, Operation operation) const;
  [[nodiscard]] Location locate_data(DspAddress access, Operation operation) const;
  [[nodiscard]] std::uint32_t read_peripheral(DspAddress access);
  void write_peripheral(DspStore access);
  [[nodiscard]] DspEsai *esai_register(DspAddress access);
  [[nodiscard]] DspEsai &sync_esai(DspEsaiInstance instance);
  void sync_esai_port(DspPort changed);
  void sync_dax();
  [[nodiscard]] std::optional<DspDmaWord>
  advance_dma_with_admission(DspDmaInputs inputs, const DspDmaAdmission &admission);
  std::array<Memory, 3> ram_;
  std::array<Memory, 3> rom_;
  DspShi shi_;
  DspPll pll_;
  DspTimers timers_;
  DspInterrupts interrupts_;
  DspDma dma_;
  DspDax dax_;
  std::array<DspEsai, 2> esai_{DspEsai(DspEsaiInstance::primary),
                               DspEsai(DspEsaiInstance::secondary)};
  std::array<DspGpio, 4> gpio_{
      DspGpio(DspPort::c), DspGpio(DspPort::e), DspGpio(DspPort::f), DspGpio(DspPort::d)};
  std::uint32_t bus_control_ = 0;
  DspCodeChanges changes_;
  unsigned memory_mode_ = 0;
  unsigned requested_memory_mode_ = 0;
  unsigned switch_delay_ = 0;
  std::optional<std::uint32_t> once_latch_;
};

}
